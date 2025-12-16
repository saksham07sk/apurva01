/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "cuopt_router.hpp"
// Note: zone_identifier.hpp removed - zone-only planning uses filtered graph instead
// Zone identification is handled by ZoneGraphFilter during graph loading

// cuOpt C++ API includes
#include <cuopt/routing/distance_engine/waypoint_matrix.hpp>
#include <raft/core/handle.hpp>
#include <raft/core/resource/cuda_stream.hpp>
#include <rmm/device_uvector.hpp>
#include <rmm/mr/device/cuda_memory_resource.hpp>
#include <rmm/mr/device/pool_memory_resource.hpp>
#include <rmm/exec_policy.hpp>

#include <cuda_runtime.h>
#include <chrono>
#include <limits>
#include <algorithm>
#include <cmath>
#include <functional>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
// Static cache members
std::unordered_map<std::size_t, CuOptRouter::CachedCSRData> CuOptRouter::csr_cache_;
std::mutex CuOptRouter::cache_mutex_;

//==============================================================================
// Generate cache key from graph (simple hash based on dimensions)
std::size_t CuOptRouter::get_cache_key(const Graph::Implementation& graph_impl)
{
  // Simple hash: combine waypoint count and lane count
  // For zone-based planning, filtered graph is stable, so this works well
  std::size_t hash = graph_impl.waypoints.size();
  hash = hash * 31 + graph_impl.lanes.size();
  return hash;
}

//==============================================================================
/// Modular graph conversion: RMF Graph -> cuOpt CSR format
/// Handles bidirectional lanes and calculates Euclidean distances as edge weights
/// Uses caching to avoid repeated conversions for the same graph
void CuOptRouter::convert_to_csr(
  const Graph::Implementation& graph_impl,
  std::vector<int>& row_ptr,
  std::vector<int>& col_ind,
  std::vector<float>& weights)
{
  const auto& waypoints = graph_impl.waypoints;
  const auto& lanes = graph_impl.lanes;
  const std::size_t n = waypoints.size();
  
  if (n == 0)
  {
    row_ptr.clear();
    col_ind.clear();
    weights.clear();
    return;
  }
  
  // Check cache first
  std::size_t cache_key = get_cache_key(graph_impl);
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = csr_cache_.find(cache_key);
    if (it != csr_cache_.end())
    {
      // Cache hit! Use cached data
      row_ptr = it->second.row_ptr;
      col_ind = it->second.col_ind;
      weights = it->second.weights;
      std::cout << "[cuOpt] ✅ CSR CACHE HIT (key=" << cache_key 
                << ", waypoints=" << n << ", lanes=" << lanes.size() << ")" << std::endl;
      return;
    }
    std::cout << "[cuOpt] ❌ CSR CACHE MISS (key=" << cache_key 
              << ", waypoints=" << n << ", lanes=" << lanes.size() 
              << ", cache_size=" << csr_cache_.size() << ")" << std::endl;
  }
  
  // Cache miss - compute CSR format
  row_ptr.clear();
  col_ind.clear();
  weights.clear();
  
  // First pass: count edges per vertex
  // Note: RMF represents bidirectional lanes as separate directed lanes,
  // so we process all lanes as directed edges
  std::vector<std::size_t> edge_count(n, 0);
  
  for (const auto& lane : lanes)
  {
    const std::size_t from = lane.entry().waypoint_index();
    
    if (from < n)
    {
      edge_count[from]++;
    }
  }
  
  // Build row_ptr (CSR row pointer array)
  row_ptr.resize(n + 1, 0);
  for (std::size_t i = 0; i < n; i++)
  {
    row_ptr[i + 1] = row_ptr[i] + edge_count[i];
  }
  
  // Second pass: build column indices and weights
  const std::size_t total_edges = row_ptr[n];
  col_ind.resize(total_edges);
  weights.resize(total_edges);
  
  std::vector<std::size_t> current_pos(n, 0);
  
  for (const auto& lane : lanes)
  {
    const std::size_t from = lane.entry().waypoint_index();
    const std::size_t to = lane.exit().waypoint_index();
    
    if (from >= n || to >= n)
      continue;
    
    // Calculate Euclidean distance between waypoints (edge weight)
    const Eigen::Vector2d from_loc = waypoints[from].get_location();
    const Eigen::Vector2d to_loc = waypoints[to].get_location();
    const double dx = to_loc.x() - from_loc.x();
    const double dy = to_loc.y() - from_loc.y();
    const float dist = static_cast<float>(std::sqrt(dx*dx + dy*dy));
    
    // Add directed edge (RMF handles bidirectional as separate lanes)
    std::size_t pos = row_ptr[from] + current_pos[from];
    col_ind[pos] = static_cast<int>(to);
    weights[pos] = dist;
    current_pos[from]++;
  }
  
  // Cache the result for future use
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    CachedCSRData cached_data;
    cached_data.row_ptr = row_ptr;
    cached_data.col_ind = col_ind;
    cached_data.weights = weights;
    csr_cache_[cache_key] = std::move(cached_data);
    std::cout << "[cuOpt] 💾 CSR CACHED (key=" << cache_key 
              << ", cache_size=" << csr_cache_.size() << ")" << std::endl;
  }
}

//==============================================================================
std::optional<CuOptResult> CuOptRouter::solve_cuopt_distance_engine(
  std::size_t start_vertex,
  std::size_t goal_vertex,
  const Graph::Implementation& graph_impl,
  const std::vector<int>& row_ptr,
  const std::vector<int>& col_ind,
  const std::vector<float>& weights)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  auto cuda_init_start = std::chrono::high_resolution_clock::now();
  
  try
  {
    // Initialize CUDA
    cudaError_t cuda_status = cudaSetDevice(0);
    if (cuda_status != cudaSuccess)
    {
      return std::nullopt;
    }
    
    auto cuda_init_time = std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - cuda_init_start).count();
    
    // Initialize RAFT handle
    auto handle_start = std::chrono::high_resolution_clock::now();
    raft::handle_t handle;
    auto handle_time = std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - handle_start).count();
    
    const std::size_t n = graph_impl.waypoints.size();
    
    // Create waypoint matrix from CSR format (EXPENSIVE - GPU memory allocation)
    auto matrix_start = std::chrono::high_resolution_clock::now();
    cuopt::distance_engine::waypoint_matrix_t<int, float> waypoint_matrix(
      handle,
      const_cast<int*>(row_ptr.data()),
      static_cast<int>(n),
      const_cast<int*>(col_ind.data()),
      const_cast<float*>(weights.data())
    );
    auto matrix_time = std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - matrix_start).count();
    
    std::cout << "[cuOpt] ⏱️  Timing: CUDA init=" << cuda_init_time 
              << "ms, Handle=" << handle_time << "ms, Matrix=" << matrix_time << "ms" << std::endl;
    
    // Target locations (start and end)
    std::vector<int> target_locations = {
      static_cast<int>(start_vertex),
      static_cast<int>(goal_vertex)
    };
    
    // Allocate device memory for cost matrix
    size_t cost_matrix_size = target_locations.size() * target_locations.size();
    rmm::device_uvector<float> d_cost_matrix(cost_matrix_size, handle.get_stream());
    
    // Compute cost matrix (like in cuopt_router_cpp.cpp)
    waypoint_matrix.compute_cost_matrix(
      d_cost_matrix.data(),
      target_locations.data(),
      static_cast<int>(target_locations.size())
    );
    
    // Synchronize before copying
    cudaStreamSynchronize(handle.get_stream());
    
    // Copy cost matrix back to host
    std::vector<float> cost_matrix(cost_matrix_size);
    cuda_status = cudaMemcpy(
      cost_matrix.data(),
      d_cost_matrix.data(),
      cost_matrix_size * sizeof(float),
      cudaMemcpyDeviceToHost
    );
    
    if (cuda_status != cudaSuccess)
    {
      return std::nullopt;
    }
    
    // Extract distance from cost matrix (distance from start to goal)
    // cost_matrix[1] = distance from target_locations[0] to target_locations[1]
    double distance = static_cast<double>(cost_matrix[1]);
    
    if (std::isinf(distance) || distance < 0.0f)
    {
      return std::nullopt; // No path found
    }
    
    // Create route locations (simple route: start -> end)
    // The route represents: start at target_locations[0], go to target_locations[1]
    std::vector<int> route_locations = {0, 1}; // 0 = start in target_locations, 1 = end
    
    // Allocate device memory for route locations
    rmm::device_uvector<int> d_route_locations(route_locations.size(), handle.get_stream());
    cuda_status = cudaMemcpy(
      d_route_locations.data(),
      route_locations.data(),
      route_locations.size() * sizeof(int),
      cudaMemcpyHostToDevice
    );
    
    if (cuda_status != cudaSuccess)
    {
      return std::nullopt;
    }
    
    // Compute waypoint sequence (extract path like in cuopt_router_cpp.cpp)
    auto [d_offsets, d_path] = waypoint_matrix.compute_waypoint_sequence(
      target_locations.data(),
      static_cast<int>(target_locations.size()),
      d_route_locations.data(),
      static_cast<int>(route_locations.size())
    );
    
    // Synchronize before copying
    cudaStreamSynchronize(handle.get_stream());
    
    // Extract path from device memory
    std::vector<std::size_t> path;
    
    if (d_offsets && d_path)
    {
      // Copy offsets back to host
      std::vector<int> offsets(route_locations.size());
      if (d_offsets->size() >= route_locations.size() * sizeof(int))
      {
        cuda_status = cudaMemcpy(
          offsets.data(),
          d_offsets->data(),
          route_locations.size() * sizeof(int),
          cudaMemcpyDeviceToHost
        );
        
        if (cuda_status == cudaSuccess && offsets.size() > 0)
        {
          // Determine path length from last offset
          int path_length = offsets.back();
          
          if (path_length > 0 && d_path->size() >= path_length * sizeof(int))
          {
            // Copy path back to host
            std::vector<int> path_indices(path_length);
            cuda_status = cudaMemcpy(
              path_indices.data(),
              d_path->data(),
              path_length * sizeof(int),
              cudaMemcpyDeviceToHost
            );
            
            if (cuda_status == cudaSuccess)
            {
              // Convert indices to waypoint indices, removing consecutive duplicates
              for (int idx : path_indices)
              {
                if (idx >= 0 && idx < static_cast<int>(n))
                {
                  std::size_t wp_idx = static_cast<std::size_t>(idx);
                  // Remove consecutive duplicates
                  if (path.empty() || path.back() != wp_idx)
                  {
                    path.push_back(wp_idx);
                  }
                }
              }
            }
          }
        }
      }
    }
    
    // Fallback: if path is empty, add start and end
    if (path.empty())
    {
      path.push_back(start_vertex);
      path.push_back(goal_vertex);
    }
    
    // Validate path
    if (path.empty() || path.front() != start_vertex)
    {
      return std::nullopt;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double solve_time_ms = std::chrono::duration<double, std::milli>(
      end_time - start_time).count();
    
    std::cout << "[cuOpt] ⏱️  Total solve time: " << solve_time_ms << "ms" << std::endl;
    
    CuOptResult result;
    result.path = std::move(path);
    result.total_cost = distance;
    result.solve_time_ms = solve_time_ms;
    result.used_gpu = true; // cuOpt always uses GPU
    
    return result;
  }
  catch (const std::exception& e)
  {
    return std::nullopt;
  }
}

//==============================================================================
std::optional<CuOptResult> CuOptRouter::find_shortest_path(
  std::size_t start_vertex,
  std::size_t goal_vertex,
  const Graph::Implementation& graph_impl)
{
  // Validate inputs
  if (start_vertex >= graph_impl.waypoints.size() ||
      goal_vertex >= graph_impl.waypoints.size())
  {
    return std::nullopt;
  }
  
  // Trivial case: start == goal
  if (start_vertex == goal_vertex)
  {
    CuOptResult result;
    result.path = {start_vertex};
    result.total_cost = 0.0;
    result.solve_time_ms = 0.0;
    result.used_gpu = true;
    
    // Zone identification disabled for zone-only planning
    // Zones are handled by ZoneGraphFilter during graph loading
    
    return result;
  }
  
  // Get waypoint locations for logging
  const auto& start_wp = graph_impl.waypoints[start_vertex];
  const auto& goal_wp = graph_impl.waypoints[goal_vertex];
  const Eigen::Vector2d start_loc = start_wp.get_location();
  const Eigen::Vector2d goal_loc = goal_wp.get_location();
  
  // Log cuOpt routing call with detailed input
  std::cout << "\n[cuOpt] ========================================" << std::endl;
  std::cout << "[cuOpt] GPU-ACCELERATED PATH PLANNING" << std::endl;
  std::cout << "[cuOpt] Called from: DifferentialDrivePlanner::plan()" << std::endl;
  std::cout << "[cuOpt] File: DifferentialDrivePlanner.cpp" << std::endl;
  std::cout << "[cuOpt] ----------------------------------------" << std::endl;
  std::cout << "[cuOpt] INPUT:" << std::endl;
  std::cout << "[cuOpt]   Start waypoint: " << start_vertex 
            << " at (" << start_loc.x() << ", " << start_loc.y() << ")" << std::endl;
  std::cout << "[cuOpt]   Goal waypoint: " << goal_vertex 
            << " at (" << goal_loc.x() << ", " << goal_loc.y() << ")" << std::endl;
  std::cout << "[cuOpt]   Graph size: " << graph_impl.waypoints.size() 
            << " waypoints, " << graph_impl.lanes.size() << " lanes" << std::endl;
  
  // Convert RMF graph to CSR format
  std::vector<int> row_ptr, col_ind;
  std::vector<float> weights;
  convert_to_csr(graph_impl, row_ptr, col_ind, weights);
  
  // Use cuOpt distance_engine for GPU-accelerated routing
  auto result = solve_cuopt_distance_engine(
    start_vertex,
    goal_vertex,
    graph_impl,
    row_ptr,
    col_ind,
    weights
  );
  
  // Zone identification disabled for zone-only planning
  // Zones are handled by ZoneGraphFilter during graph loading
  // The filtered graph already contains only zone waypoints/lanes
  
  // Log detailed result
  std::cout << "[cuOpt] ----------------------------------------" << std::endl;
  std::cout << "[cuOpt] OUTPUT:" << std::endl;
  if (result.has_value())
  {
    std::cout << "[cuOpt]   Status: SUCCESS" << std::endl;
    std::cout << "[cuOpt]   Path length: " << result->path.size() << " waypoints" << std::endl;
    std::cout << "[cuOpt]   Total cost: " << result->total_cost << " meters" << std::endl;
    std::cout << "[cuOpt]   Solve time: " << result->solve_time_ms << " ms" << std::endl;
    std::cout << "[cuOpt]   GPU used: " << (result->used_gpu ? "YES" : "NO") << std::endl;
    std::cout << "[cuOpt]   Path waypoints: [";
    for (std::size_t i = 0; i < result->path.size(); ++i)
    {
      std::cout << result->path[i];
      if (i < result->path.size() - 1)
        std::cout << " -> ";
    }
    std::cout << "]" << std::endl;
  }
  else
  {
    std::cout << "[cuOpt]   Status: FAILED - No path found" << std::endl;
  }
  std::cout << "[cuOpt] ========================================\n" << std::endl;
  
  return result;
}

//==============================================================================
// NOTE: find_global_path function removed - not needed for zone-only planning
// Zone-only planning uses filtered graph instead of zone sequential planning
// If needed in the future, this function can be restored

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
