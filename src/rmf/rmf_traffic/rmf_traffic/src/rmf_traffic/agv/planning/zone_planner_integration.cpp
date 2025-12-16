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

#include "zone_planner_integration.hpp"
#include "../internal_Graph.hpp"
#include <iostream>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
ZonePlannerIntegration::ZonePlannerIntegration(
  const Graph& full_graph,
  const ZoneRegistry& zone_registry,
  const std::string& /* building_map_path */)
: full_graph_(full_graph),
  zone_registry_(zone_registry),
  zones_enabled_(false)
{
  // Check if zones are defined
  if (zone_registry_.num_zones() == 0)
  {
    std::cout << "[ZonePlannerIntegration] No zones defined - using full graph for planning" << std::endl;
    zones_enabled_ = false;
    return;
  }

  // Get full graph implementation
  const auto& full_graph_impl = Graph::Implementation::get(full_graph_);

  // Create zone filter
  zone_filter_ = std::make_unique<ZoneGraphFilter>(full_graph_impl, zone_registry_);

  zones_enabled_ = true;

  std::cout << "\n[ZonePlannerIntegration] ========================================" << std::endl;
  std::cout << "[ZonePlannerIntegration] ✅ ZONE-BASED PLANNING ENABLED" << std::endl;
  std::cout << "[ZonePlannerIntegration]   Full graph: " << full_graph_impl.waypoints.size() 
            << " waypoints, " << full_graph_impl.lanes.size() << " lanes" << std::endl;
  std::cout << "[ZonePlannerIntegration]   Filtered graph: " 
            << zone_filter_->num_zone_waypoints() << " waypoints, " 
            << zone_filter_->num_zone_lanes() << " lanes" << std::endl;
  std::cout << "[ZonePlannerIntegration]   Zones: " << zone_registry_.num_zones() << std::endl;
  std::cout << "[ZonePlannerIntegration] ========================================\n" << std::endl;
}

//==============================================================================
const Graph& ZonePlannerIntegration::get_full_graph() const
{
  return full_graph_;
}

//==============================================================================
const Graph::Implementation& ZonePlannerIntegration::get_filtered_graph() const
{
  if (!zones_enabled_ || !zone_filter_)
  {
    // Fallback to full graph if zones not enabled
    return Graph::Implementation::get(full_graph_);
  }
  return zone_filter_->get_filtered_graph();
}

//==============================================================================
Graph ZonePlannerIntegration::get_filtered_graph_object() const
{
  if (!zones_enabled_ || !zone_filter_)
  {
    // Fallback to full graph if zones not enabled
    return full_graph_;
  }
  return zone_filter_->get_filtered_graph_object();
}

//==============================================================================
bool ZonePlannerIntegration::is_zone_planning_enabled() const
{
  return zones_enabled_;
}

//==============================================================================
std::optional<std::size_t> ZonePlannerIntegration::full_to_filtered_waypoint(
  std::size_t full_index) const
{
  if (!zones_enabled_ || !zone_filter_)
  {
    // If zones not enabled, indices are the same
    return full_index;
  }
  return zone_filter_->full_to_filtered(full_index);
}

//==============================================================================
std::optional<std::size_t> ZonePlannerIntegration::filtered_to_full_waypoint(
  std::size_t filtered_index) const
{
  if (!zones_enabled_ || !zone_filter_)
  {
    // If zones not enabled, indices are the same
    return filtered_index;
  }
  return zone_filter_->filtered_to_full(filtered_index);
}

//==============================================================================
bool ZonePlannerIntegration::is_waypoint_in_zone(std::size_t full_index) const
{
  if (!zones_enabled_)
  {
    // If zones not enabled, all waypoints are considered "in zone"
    return true;
  }
  return zone_filter_->is_waypoint_in_zone(full_index);
}

//==============================================================================
bool ZonePlannerIntegration::validate_task_waypoints(
  std::size_t start_waypoint,
  std::size_t goal_waypoint) const
{
  // If zones not enabled, all tasks are valid
  if (!zones_enabled_)
  {
    return true;
  }

  // Check if start waypoint is in zone
  bool start_in_zone = is_waypoint_in_zone(start_waypoint);
  
  // Check if goal waypoint is in zone
  bool goal_in_zone = is_waypoint_in_zone(goal_waypoint);
  
  // Check if goal is a charging station (charging stations outside zones are allowed)
  bool goal_is_charger = is_charging_station(goal_waypoint);
  bool start_is_charger = is_charging_station(start_waypoint);

  // VALIDATION RULES:
  // 1. If BOTH waypoints are in zones → VALID
  // 2. If goal is a charging station (even outside zone) → VALID (charging stations work outside zones)
  // 3. If start is a charging station and goal is in zone → VALID
  // 4. Otherwise → INVALID (at least one waypoint must be in zone, unless it's a charger)
  bool valid = (start_in_zone && goal_in_zone) || 
               goal_is_charger || 
               (start_is_charger && goal_in_zone);

  if (!valid)
  {
    std::cerr << "[ZonePlannerIntegration] ❌ TASK REJECTED: Waypoints outside zones" << std::endl;
    std::cerr << "[ZonePlannerIntegration]   Start waypoint " << start_waypoint 
              << (start_in_zone ? " ✓ in zone" : " ✗ NOT in zone")
              << (start_is_charger ? " (charger)" : "") << std::endl;
    std::cerr << "[ZonePlannerIntegration]   Goal waypoint " << goal_waypoint 
              << (goal_in_zone ? " ✓ in zone" : " ✗ NOT in zone")
              << (goal_is_charger ? " (charger)" : "") << std::endl;
    std::cerr << "[ZonePlannerIntegration]   Rule: Waypoints must be in zones OR be charging stations" << std::endl;
    
    // Debug: Check waypoint names if available
    const auto& full_graph_impl = Graph::Implementation::get(full_graph_);
    if (start_waypoint < full_graph_impl.waypoints.size())
    {
      const auto& start_wp = full_graph_impl.waypoints[start_waypoint];
      if (start_wp.name())
        std::cerr << "[ZonePlannerIntegration]   Start waypoint name: " << *start_wp.name() << std::endl;
    }
    if (goal_waypoint < full_graph_impl.waypoints.size())
    {
      const auto& goal_wp = full_graph_impl.waypoints[goal_waypoint];
      if (goal_wp.name())
        std::cerr << "[ZonePlannerIntegration]   Goal waypoint name: " << *goal_wp.name() << std::endl;
    }
  }

  return valid;
}

//==============================================================================
bool ZonePlannerIntegration::is_charging_station(std::size_t full_index) const
{
  const auto& full_graph_impl = Graph::Implementation::get(full_graph_);
  
  if (full_index >= full_graph_impl.waypoints.size())
  {
    return false;
  }

  return full_graph_impl.waypoints[full_index].is_charger();
}

//==============================================================================
const ZoneRegistry& ZonePlannerIntegration::get_zone_registry() const
{
  return zone_registry_;
}

//==============================================================================
const ZoneGraphFilter& ZonePlannerIntegration::get_zone_filter() const
{
  if (!zone_filter_)
  {
    throw std::runtime_error("Zone filter not initialized");
  }
  return *zone_filter_;
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

