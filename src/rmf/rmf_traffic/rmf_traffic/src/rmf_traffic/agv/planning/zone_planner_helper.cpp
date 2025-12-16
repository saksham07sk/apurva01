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

#include "zone_planner_helper.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
ZonePlannerHelper& ZonePlannerHelper::get_instance()
{
  static ZonePlannerHelper instance;
  return instance;
}

//==============================================================================
void ZonePlannerHelper::initialize(
  const Graph& full_graph,
  const ZoneRegistry& zone_registry,
  const std::string& building_map_path)
{
  integration_ = std::make_unique<ZonePlannerIntegration>(
    full_graph,
    zone_registry,
    building_map_path);
}

//==============================================================================
bool ZonePlannerHelper::is_enabled() const
{
  return integration_ && integration_->is_zone_planning_enabled();
}

//==============================================================================
const ZonePlannerIntegration* ZonePlannerHelper::get_integration() const
{
  return integration_.get();
}

//==============================================================================
std::optional<std::size_t> ZonePlannerHelper::translate_to_filtered(
  std::size_t full_index) const
{
  if (!is_enabled() || !integration_)
  {
    return full_index;  // No translation if zones disabled
  }
  return integration_->full_to_filtered_waypoint(full_index);
}

//==============================================================================
std::optional<std::size_t> ZonePlannerHelper::translate_to_full(
  std::size_t filtered_index) const
{
  if (!is_enabled() || !integration_)
  {
    return filtered_index;  // No translation if zones disabled
  }
  return integration_->filtered_to_full_waypoint(filtered_index);
}

//==============================================================================
bool ZonePlannerHelper::validate_task(
  std::size_t start_waypoint,
  std::size_t goal_waypoint) const
{
  if (!is_enabled() || !integration_)
  {
    return true;  // All tasks valid if zones disabled
  }
  return integration_->validate_task_waypoints(start_waypoint, goal_waypoint);
}

//==============================================================================
const Graph::Implementation& ZonePlannerHelper::get_planning_graph(
  const Graph::Implementation& full_graph) const
{
  if (!is_enabled() || !integration_)
  {
    return full_graph;  // Use full graph if zones disabled
  }
  return integration_->get_filtered_graph();
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic


