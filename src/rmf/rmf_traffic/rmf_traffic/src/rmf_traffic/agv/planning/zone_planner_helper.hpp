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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_PLANNER_HELPER_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_PLANNER_HELPER_HPP

#include "zone_planner_integration.hpp"
#include <optional>
#include <memory>
#include <cstddef>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// ZonePlannerHelper: Global helper for zone-based planning
/// 
/// This singleton manages zone integration for the planner.
/// It allows the planner to access zone filtering without modifying
/// the Planner::Configuration interface.
class ZonePlannerHelper
{
public:
  /// Get singleton instance
  static ZonePlannerHelper& get_instance();

  /// Initialize zone integration
  /// 
  /// \param[in] full_graph The complete navigation graph
  /// \param[in] zone_registry Registry containing zone definitions
  /// \param[in] building_map_path Path to building.yaml
  void initialize(
    const Graph& full_graph,
    const ZoneRegistry& zone_registry,
    const std::string& building_map_path = "");

  /// Check if zones are enabled
  /// 
  /// \return True if zones are initialized and enabled
  bool is_enabled() const;

  /// Get zone integration (if initialized)
  /// 
  /// \return Pointer to zone integration, or nullptr if not initialized
  const ZonePlannerIntegration* get_integration() const;

  /// Translate full graph waypoint index to filtered graph index
  /// 
  /// \param[in] full_index Waypoint index in full graph
  /// \return Filtered graph index, or nullopt if not in zone
  std::optional<std::size_t> translate_to_filtered(std::size_t full_index) const;

  /// Translate filtered graph waypoint index to full graph index
  /// 
  /// \param[in] filtered_index Waypoint index in filtered graph
  /// \return Full graph index
  std::optional<std::size_t> translate_to_full(std::size_t filtered_index) const;

  /// Validate task waypoints
  /// 
  /// \param[in] start_waypoint Start waypoint (full graph index)
  /// \param[in] goal_waypoint Goal waypoint (full graph index)
  /// \return True if task is valid (waypoints in zones or charging stations)
  bool validate_task(std::size_t start_waypoint, std::size_t goal_waypoint) const;

  /// Get filtered graph for planning
  /// 
  /// \return Reference to filtered graph implementation, or full graph if zones disabled
  const Graph::Implementation& get_planning_graph(
    const Graph::Implementation& full_graph) const;

private:
  ZonePlannerHelper() = default;
  ~ZonePlannerHelper() = default;
  ZonePlannerHelper(const ZonePlannerHelper&) = delete;
  ZonePlannerHelper& operator=(const ZonePlannerHelper&) = delete;

  std::unique_ptr<ZonePlannerIntegration> integration_;
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_PLANNER_HELPER_HPP


