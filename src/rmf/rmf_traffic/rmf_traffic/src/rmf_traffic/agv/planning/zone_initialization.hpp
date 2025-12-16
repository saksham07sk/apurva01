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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_INITIALIZATION_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_INITIALIZATION_HPP

#include <rmf_traffic/agv/Graph.hpp>
#include "zone_registry.hpp"
#include "zone_planner_helper.hpp"
#include <string>
#include <memory>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// ZoneInitialization: Helper for initializing zone-based planning
/// 
/// This class provides static methods to initialize zone-based planning
/// from building.yaml files or programmatically.
class ZoneInitialization
{
public:
  /// Initialize zone-based planning from building.yaml file
  /// 
  /// This should be called once during system startup, before creating planners.
  /// 
  /// \param[in] full_graph The complete navigation graph (all waypoints/lanes)
  /// \param[in] building_map_path Path to building.yaml file containing zone definitions
  /// \param[in] level_name Level name (e.g., "L1") to load zones from
  /// \return True if zones were successfully loaded and initialized
  static bool initialize_from_building_map(
    const Graph& full_graph,
    const std::string& building_map_path,
    const std::string& level_name = "L1");

  /// Initialize zone-based planning from zone registry
  /// 
  /// \param[in] full_graph The complete navigation graph
  /// \param[in] zone_registry Registry containing zone definitions
  /// \param[in] building_map_path Optional path to building.yaml (for logging)
  /// \return True if zones were successfully initialized
  static bool initialize_from_registry(
    const Graph& full_graph,
    const ZoneRegistry& zone_registry,
    const std::string& building_map_path = "");

  /// Check if zones are initialized
  /// 
  /// \return True if zone-based planning is enabled
  static bool is_initialized();

  /// Get zone registry (if initialized)
  /// 
  /// \return Pointer to zone registry, or nullptr if not initialized
  static const ZoneRegistry* get_registry();

private:
  static std::unique_ptr<ZoneRegistry> registry_;
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_INITIALIZATION_HPP


