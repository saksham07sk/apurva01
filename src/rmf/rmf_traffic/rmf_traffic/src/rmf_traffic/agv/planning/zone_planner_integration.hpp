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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_PLANNER_INTEGRATION_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_PLANNER_INTEGRATION_HPP

#include <rmf_traffic/agv/Graph.hpp>
#include "zone_registry.hpp"
#include "zone_graph_filter.hpp"
#include <optional>
#include <memory>
#include <cstddef>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// ZonePlannerIntegration: Manages zone-based planning integration
/// 
/// This class provides:
/// - Full graph (for visualization in RViz)
/// - Filtered graph (for planning - zone waypoints/lanes only)
/// - Index translation between full and filtered graphs
/// - Task validation (reject tasks outside zones)
/// - Charging station handling (always accessible regardless of zone)
class ZonePlannerIntegration
{
public:
  /// Constructor
  /// 
  /// \param[in] full_graph The complete navigation graph (all waypoints/lanes)
  /// \param[in] zone_registry Registry containing zone definitions
  /// \param[in] building_map_path Path to building.yaml (for zone loading)
  ZonePlannerIntegration(
    const Graph& full_graph,
    const ZoneRegistry& zone_registry,
    const std::string& building_map_path = "");

  /// Get the full graph (for visualization)
  /// 
  /// \return Reference to the full graph
  const Graph& get_full_graph() const;

  /// Get the filtered graph (for planning - zone waypoints/lanes only)
  /// 
  /// \return Reference to the filtered graph implementation
  const Graph::Implementation& get_filtered_graph() const;

  /// Get the filtered graph as a Graph object
  /// 
  /// \return The filtered graph
  Graph get_filtered_graph_object() const;

  /// Check if zones are enabled (filtered graph is active)
  /// 
  /// \return True if zones are defined and filtered graph is being used
  bool is_zone_planning_enabled() const;

  /// Convert full graph waypoint index to filtered graph index
  /// 
  /// \param[in] full_index Waypoint index in the full graph
  /// \return Filtered graph index if waypoint is in zone, nullopt otherwise
  std::optional<std::size_t> full_to_filtered_waypoint(
    std::size_t full_index) const;

  /// Convert filtered graph waypoint index to full graph index
  /// 
  /// \param[in] filtered_index Waypoint index in the filtered graph
  /// \return Full graph index
  std::optional<std::size_t> filtered_to_full_waypoint(
    std::size_t filtered_index) const;

  /// Validate if a waypoint (full graph index) is in any zone
  /// 
  /// \param[in] full_index Waypoint index in the full graph
  /// \return True if waypoint belongs to any zone
  bool is_waypoint_in_zone(std::size_t full_index) const;

  /// Validate if a task (start/goal waypoints) is valid for zone planning
  /// 
  /// \param[in] start_waypoint Start waypoint index (full graph)
  /// \param[in] goal_waypoint Goal waypoint index (full graph)
  /// \return True if both waypoints are in zones (or zones disabled)
  bool validate_task_waypoints(
    std::size_t start_waypoint,
    std::size_t goal_waypoint) const;

  /// Check if a waypoint is a charging station (always accessible)
  /// 
  /// \param[in] full_index Waypoint index in the full graph
  /// \return True if waypoint is a charging station
  bool is_charging_station(std::size_t full_index) const;

  /// Get zone registry
  /// 
  /// \return Reference to zone registry
  const ZoneRegistry& get_zone_registry() const;

  /// Get zone graph filter
  /// 
  /// \return Reference to zone graph filter
  const ZoneGraphFilter& get_zone_filter() const;

private:
  Graph full_graph_;  // Full graph for visualization
  const ZoneRegistry& zone_registry_;
  std::unique_ptr<ZoneGraphFilter> zone_filter_;  // Filtered graph for planning
  bool zones_enabled_;  // Whether zone planning is active
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_PLANNER_INTEGRATION_HPP

