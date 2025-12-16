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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__CUOPT_ROUTER_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__CUOPT_ROUTER_HPP

#include <rmf_traffic/agv/Graph.hpp>
#include "../internal_Graph.hpp"
#include <optional>
#include <vector>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <mutex>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// Result of cuOpt routing using distance_engine
struct CuOptResult
{
  std::vector<std::size_t> path;  // Waypoint indices in order
  double total_cost;              // Total path cost/distance from cost matrix
  double solve_time_ms;           // Time taken to solve (milliseconds)
  bool used_gpu;                  // Whether GPU acceleration was used (always true for cuOpt)
};

//==============================================================================
/// CuOpt Router using NVIDIA cuOpt distance_engine C++ API
/// Completely replaces A* and Dijkstra algorithms with GPU-accelerated routing
class CuOptRouter
{
public:
  /// Find shortest path using cuOpt distance_engine (GPU-accelerated)
  /// 
  /// \param[in] start_vertex Starting waypoint index
  /// \param[in] goal_vertex Goal waypoint index  
  /// \param[in] graph_impl The RMF graph implementation containing waypoints and lanes
  /// \return Optional CuOptResult with path and metrics, or nullopt on error
  static std::optional<CuOptResult> find_shortest_path(
    std::size_t start_vertex,
    std::size_t goal_vertex,
    const Graph::Implementation& graph_impl);

private:
  /// Cached CSR data
  struct CachedCSRData
  {
    std::vector<int> row_ptr;
    std::vector<int> col_ind;
    std::vector<float> weights;
  };
  
  /// Convert RMF graph to CSR format for cuOpt (with caching)
  static void convert_to_csr(
    const Graph::Implementation& graph_impl,
    std::vector<int>& row_ptr,
    std::vector<int>& col_ind,
    std::vector<float>& weights);
  
  /// Solve using cuOpt distance_engine (GPU-accelerated)
  static std::optional<CuOptResult> solve_cuopt_distance_engine(
    std::size_t start_vertex,
    std::size_t goal_vertex,
    const Graph::Implementation& graph_impl,
    const std::vector<int>& row_ptr,
    const std::vector<int>& col_ind,
    const std::vector<float>& weights);
  
  /// Cache for CSR data (keyed by graph size hash)
  static std::unordered_map<std::size_t, CachedCSRData> csr_cache_;
  static std::mutex cache_mutex_;
  
  /// Generate cache key from graph (simple hash based on dimensions)
  static std::size_t get_cache_key(const Graph::Implementation& graph_impl);
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__CUOPT_ROUTER_HPP
