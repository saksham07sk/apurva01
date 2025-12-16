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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_GRAPH_FILTER_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_GRAPH_FILTER_HPP

#include <rmf_traffic/agv/Graph.hpp>
#include <optional>
#include <vector>
#include <set>
#include <map>
#include <cstddef>

namespace rmf_traffic {
namespace agv {
namespace planning {

// Forward declaration
class ZoneRegistry;

//==============================================================================
/// ZoneGraphFilter: Filters a navigation graph to include only zone waypoints and lanes
/// 
/// This class extracts waypoints and lanes that belong to defined zones,
/// creating a filtered graph that contains only zone-defined areas.
/// Non-zone areas are completely excluded from the filtered graph.
class ZoneGraphFilter
{
public:
  /// Constructor
  /// 
  /// \param[in] full_graph The complete navigation graph
  /// \param[in] zone_registry Registry containing zone definitions
  ZoneGraphFilter(
    const Graph::Implementation& full_graph,
    const ZoneRegistry& zone_registry);

  /// Get the filtered graph (zone waypoints and lanes only)
  /// 
  /// \return Reference to the filtered graph implementation
  const Graph::Implementation& get_filtered_graph() const;
  
  /// Get the filtered graph as a Graph object
  /// 
  /// \return The filtered graph
  Graph get_filtered_graph_object() const;

  /// Check if a waypoint (by full graph index) is in any zone
  /// 
  /// \param[in] full_index Waypoint index in the full graph
  /// \return True if the waypoint belongs to any zone
  bool is_waypoint_in_zone(std::size_t full_index) const;

  /// Convert full graph waypoint index to filtered graph index
  /// 
  /// \param[in] full_index Waypoint index in the full graph
  /// \return Filtered graph index if waypoint is in zone, nullopt otherwise
  std::optional<std::size_t> full_to_filtered(std::size_t full_index) const;

  /// Convert filtered graph waypoint index to full graph index
  /// 
  /// \param[in] filtered_index Waypoint index in the filtered graph
  /// \return Full graph index
  std::optional<std::size_t> filtered_to_full(std::size_t filtered_index) const;

  /// Get the number of zone waypoints in the filtered graph
  /// 
  /// \return Number of waypoints in zones
  std::size_t num_zone_waypoints() const;

  /// Get the number of zone lanes in the filtered graph
  /// 
  /// \return Number of lanes connecting zone waypoints
  std::size_t num_zone_lanes() const;

private:
  /// Extract all waypoints that belong to any zone
  /// 
  /// \param[in] zone_registry Registry containing zone definitions
  /// \return Set of waypoint indices that belong to zones
  std::set<std::size_t> extract_zone_waypoints(
    const ZoneRegistry& zone_registry) const;

  /// Extract lanes where both entry and exit waypoints are in zones
  /// 
  /// \param[in] zone_waypoints Set of waypoint indices that belong to zones
  /// \return Set of lane indices where both endpoints are in zones
  std::set<std::size_t> extract_zone_lanes(
    const std::set<std::size_t>& zone_waypoints) const;

  /// Build the filtered graph from zone waypoints and lanes
  /// 
  /// \param[in] zone_waypoints Set of waypoint indices in zones
  /// \param[in] zone_lanes Set of lane indices connecting zone waypoints
  void build_filtered_graph(
    const std::set<std::size_t>& zone_waypoints,
    const std::set<std::size_t>& zone_lanes);

  const Graph::Implementation& full_graph_;
  const ZoneRegistry& zone_registry_;
  
  // Filtered graph containing only zone waypoints and lanes
  Graph filtered_graph_wrapper_;
  Graph::Implementation* filtered_graph_impl_;
  
  // Index mapping: full graph index <-> filtered graph index
  std::map<std::size_t, std::size_t> full_to_filtered_map_;
  std::map<std::size_t, std::size_t> filtered_to_full_map_;
  
  // Set of zone waypoint indices (for quick lookup)
  std::set<std::size_t> zone_waypoints_;
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_GRAPH_FILTER_HPP

