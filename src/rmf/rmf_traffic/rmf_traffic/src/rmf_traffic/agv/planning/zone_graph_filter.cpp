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

#include "zone_graph_filter.hpp"
#include "zone_registry.hpp"
#include "../internal_Graph.hpp"
#include <rmf_traffic/agv/Graph.hpp>
#include <iostream>
#include <algorithm>
#include <iomanip>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
ZoneGraphFilter::ZoneGraphFilter(
  const Graph::Implementation& full_graph,
  const ZoneRegistry& zone_registry)
: full_graph_(full_graph),
  zone_registry_(zone_registry),
  filtered_graph_wrapper_(),
  filtered_graph_impl_(nullptr)
{
  std::cout << "\n[ZoneGraphFilter] ========================================" << std::endl;
  std::cout << "[ZoneGraphFilter] 🎯 ZONE GRAPH FILTERING" << std::endl;
  std::cout << "[ZoneGraphFilter]   Full graph: " << full_graph_.waypoints.size() 
            << " waypoints, " << full_graph_.lanes.size() << " lanes" << std::endl;
  std::cout << "[ZoneGraphFilter] ----------------------------------------" << std::endl;
  
  // Extract zone waypoints (includes charging stations outside zones)
  std::cout << "[ZoneGraphFilter] Step 1: Extracting zone waypoints..." << std::endl;
  zone_waypoints_ = extract_zone_waypoints(zone_registry);
  std::cout << "[ZoneGraphFilter]   ✓ Found " << zone_waypoints_.size() 
            << " waypoints (zones + charging stations)" << std::endl;
  
  // Step 1.5: Ensure all waypoints referenced by explicit lanes are included FIRST
  // This must happen BEFORE extracting lanes to ensure lane waypoints are available
  std::cout << "[ZoneGraphFilter] Step 1.5: Ensuring explicit lane waypoints are included..." << std::endl;
  std::size_t waypoints_added_from_lanes = 0;
  for (const auto& zone : zone_registry.get_all_zones())
  {
    for (std::size_t lane_idx : zone.lanes)
    {
      if (lane_idx < full_graph_.lanes.size())
      {
        const auto& lane = full_graph_.lanes[lane_idx];
        std::size_t entry_wp = lane.entry().waypoint_index();
        std::size_t exit_wp = lane.exit().waypoint_index();
        
        // Add entry waypoint if not already in zone_waypoints_ and is valid
        if (entry_wp < full_graph_.waypoints.size() && zone_waypoints_.count(entry_wp) == 0)
        {
          zone_waypoints_.insert(entry_wp);
          waypoints_added_from_lanes++;
        }
        
        // Add exit waypoint if not already in zone_waypoints_ and is valid
        if (exit_wp < full_graph_.waypoints.size() && zone_waypoints_.count(exit_wp) == 0)
        {
          zone_waypoints_.insert(exit_wp);
          waypoints_added_from_lanes++;
        }
      }
    }
  }
  if (waypoints_added_from_lanes > 0)
  {
    std::cout << "[ZoneGraphFilter]   ✓ Added " << waypoints_added_from_lanes 
              << " waypoints referenced by explicit lanes" << std::endl;
  }
  else
  {
    std::cout << "[ZoneGraphFilter]   ✓ All explicit lane waypoints already included" << std::endl;
  }
  
  // Extract zone lanes (use explicit lanes from YAML if available, otherwise calculate from waypoints)
  // Now zone_waypoints_ includes all waypoints from explicit lanes
  std::cout << "[ZoneGraphFilter] Step 2: Extracting zone lanes..." << std::endl;
  std::set<std::size_t> zone_lanes = extract_zone_lanes(zone_waypoints_);
  std::cout << "[ZoneGraphFilter]   ✓ Found " << zone_lanes.size() 
            << " lanes (using explicit lanes from YAML + calculated lanes)" << std::endl;
  
  // Build filtered graph
  std::cout << "[ZoneGraphFilter] Step 3: Building filtered graph..." << std::endl;
  build_filtered_graph(zone_waypoints_, zone_lanes);
  
  // Get pointer to filtered graph implementation
  filtered_graph_impl_ = &Graph::Implementation::get(filtered_graph_wrapper_);
  
  std::cout << "[ZoneGraphFilter] ----------------------------------------" << std::endl;
  std::cout << "[ZoneGraphFilter] ✅ FILTERED GRAPH CREATED" << std::endl;
  std::cout << "[ZoneGraphFilter]   Full graph: " << full_graph_.waypoints.size() 
            << " waypoints, " << full_graph_.lanes.size() << " lanes" << std::endl;
  std::cout << "[ZoneGraphFilter]   Filtered graph: " << filtered_graph_impl_->waypoints.size() 
            << " waypoints, " << filtered_graph_impl_->lanes.size() << " lanes" << std::endl;
  
  // Calculate reduction percentage
  if (full_graph_.waypoints.size() > 0)
  {
    double wp_reduction = 100.0 * (1.0 - static_cast<double>(filtered_graph_impl_->waypoints.size()) / full_graph_.waypoints.size());
    double lane_reduction = 100.0 * (1.0 - static_cast<double>(filtered_graph_impl_->lanes.size()) / full_graph_.lanes.size());
    std::cout << "[ZoneGraphFilter]   Reduction: " << std::fixed << std::setprecision(1)
              << wp_reduction << "% waypoints, " << lane_reduction << "% lanes" << std::endl;
  }
  std::cout << "[ZoneGraphFilter] ========================================\n" << std::endl;
}

//==============================================================================
const Graph::Implementation& ZoneGraphFilter::get_filtered_graph() const
{
  return *filtered_graph_impl_;
}

//==============================================================================
Graph ZoneGraphFilter::get_filtered_graph_object() const
{
  return filtered_graph_wrapper_;
}

//==============================================================================
bool ZoneGraphFilter::is_waypoint_in_zone(std::size_t full_index) const
{
  return zone_waypoints_.count(full_index) > 0;
}

//==============================================================================
std::optional<std::size_t> ZoneGraphFilter::full_to_filtered(std::size_t full_index) const
{
  const auto it = full_to_filtered_map_.find(full_index);
  if (it != full_to_filtered_map_.end())
  {
    return it->second;
  }
  return std::nullopt;
}

//==============================================================================
std::optional<std::size_t> ZoneGraphFilter::filtered_to_full(std::size_t filtered_index) const
{
  const auto it = filtered_to_full_map_.find(filtered_index);
  if (it != filtered_to_full_map_.end())
  {
    return it->second;
  }
  return std::nullopt;
}

//==============================================================================
std::size_t ZoneGraphFilter::num_zone_waypoints() const
{
  return filtered_graph_impl_->waypoints.size();
}

//==============================================================================
std::size_t ZoneGraphFilter::num_zone_lanes() const
{
  return filtered_graph_impl_->lanes.size();
}

//==============================================================================
std::set<std::size_t> ZoneGraphFilter::extract_zone_waypoints(
  const ZoneRegistry& zone_registry) const
{
  std::set<std::size_t> zone_waypoints;
  std::set<std::size_t> charging_stations;
  
  std::cout << "[ZoneGraphFilter]     Checking " << full_graph_.waypoints.size() 
            << " waypoints for zone membership..." << std::endl;
  
  // Iterate through all waypoints in the full graph
  for (std::size_t i = 0; i < full_graph_.waypoints.size(); ++i)
  {
    const auto& wp = full_graph_.waypoints[i];
    
    // Check if this waypoint belongs to any zone
    std::vector<std::string> zones = zone_registry.get_zones_for_waypoint(i);
    
    if (!zones.empty())
    {
      // Waypoint belongs to at least one zone
      zone_waypoints.insert(i);
    }
    
    // ALWAYS include charging stations, even if outside zones
    // This ensures robots can navigate to charging stations outside zones
    if (wp.is_charger())
    {
      charging_stations.insert(i);
    }
  }
  
  // Add all charging stations to the filtered graph (even if outside zones)
  std::size_t chargers_added = 0;
  for (const auto& charger_idx : charging_stations)
  {
    if (zone_waypoints.count(charger_idx) == 0)
    {
      zone_waypoints.insert(charger_idx);
      chargers_added++;
    }
  }
  
  std::cout << "[ZoneGraphFilter]     Result: " << zone_waypoints.size() 
            << " waypoints (zones: " << (zone_waypoints.size() - chargers_added)
            << ", charging stations outside zones: " << chargers_added << ")" << std::endl;
  
  return zone_waypoints;
}

//==============================================================================
std::set<std::size_t> ZoneGraphFilter::extract_zone_lanes(
  const std::set<std::size_t>& zone_waypoints) const
{
  std::set<std::size_t> zone_lanes;
  
  // FIRST: Use explicit lanes from zone definitions (from YAML)
  std::size_t explicit_lanes_count = 0;
  for (const auto& zone : zone_registry_.get_all_zones())
  {
    for (std::size_t lane_idx : zone.lanes)
    {
      if (lane_idx < full_graph_.lanes.size())
      {
        zone_lanes.insert(lane_idx);
        explicit_lanes_count++;
      }
      else
      {
        std::cerr << "[ZoneGraphFilter] ⚠️  Warning: Zone '" << zone.id 
                  << "' references invalid lane index " << lane_idx 
                  << " (graph has " << full_graph_.lanes.size() << " lanes)" << std::endl;
      }
    }
  }
  
  std::cout << "[ZoneGraphFilter]     Explicit lanes from YAML: " << explicit_lanes_count << std::endl;
  
  // SECOND: Also include lanes where BOTH endpoints are in zones
  // (This catches any lanes that might not be explicitly listed but connect zone waypoints)
  std::cout << "[ZoneGraphFilter]     Also checking lanes connecting zone waypoints..." << std::endl;
  std::cout << "[ZoneGraphFilter]     Criteria: Both entry AND exit waypoints must be in zones" << std::endl;
  
  std::size_t lanes_with_both_in_zones = 0;
  std::size_t lanes_with_one_in_zone = 0;
  std::size_t lanes_with_none_in_zone = 0;
  std::size_t additional_lanes_added = 0;
  
  // Iterate through all lanes in the full graph
  for (std::size_t i = 0; i < full_graph_.lanes.size(); ++i)
  {
    const auto& lane = full_graph_.lanes[i];
    
    // Get entry and exit waypoint indices
    std::size_t entry_wp = lane.entry().waypoint_index();
    std::size_t exit_wp = lane.exit().waypoint_index();
    
    bool entry_in_zone = zone_waypoints.count(entry_wp) > 0;
    bool exit_in_zone = zone_waypoints.count(exit_wp) > 0;
    
    // Include lane if BOTH endpoints are in zones
    if (entry_in_zone && exit_in_zone)
    {
      if (zone_lanes.insert(i).second)  // Only count if newly inserted
      {
        additional_lanes_added++;
      }
      lanes_with_both_in_zones++;
    }
    else if (entry_in_zone || exit_in_zone)
    {
      lanes_with_one_in_zone++;
    }
    else
    {
      lanes_with_none_in_zone++;
    }
  }
  
  std::cout << "[ZoneGraphFilter]     Result: " << zone_lanes.size() 
            << " total lanes included" << std::endl;
  std::cout << "[ZoneGraphFilter]     Breakdown:" << std::endl;
  std::cout << "[ZoneGraphFilter]       - Explicit lanes from YAML: " << explicit_lanes_count << std::endl;
  std::cout << "[ZoneGraphFilter]       - Additional lanes (both endpoints in zones): " << additional_lanes_added << std::endl;
  std::cout << "[ZoneGraphFilter]       - Lanes with one endpoint in zone: " << lanes_with_one_in_zone << std::endl;
  std::cout << "[ZoneGraphFilter]       - Lanes with neither endpoint in zone: " << lanes_with_none_in_zone << std::endl;
  
  return zone_lanes;
}

//==============================================================================
void ZoneGraphFilter::build_filtered_graph(
  const std::set<std::size_t>& zone_waypoints,
  const std::set<std::size_t>& zone_lanes)
{
  std::cout << "[ZoneGraphFilter]     Building filtered graph structure..." << std::endl;
  
  // Clear any existing mappings
  full_to_filtered_map_.clear();
  filtered_to_full_map_.clear();
  
  // Step 1: Add zone waypoints to filtered graph and build index mapping
  std::cout << "[ZoneGraphFilter]     Adding " << zone_waypoints.size() << " zone waypoints..." << std::endl;
  std::size_t filtered_idx = 0;
  for (std::size_t full_idx : zone_waypoints)
  {
    const auto& full_wp = full_graph_.waypoints[full_idx];
    
    // Get waypoint location and map name
    Eigen::Vector2d location = full_wp.get_location();
    std::string map_name = full_wp.get_map_name();
    
    // Add waypoint to filtered graph
    auto& filtered_wp = filtered_graph_wrapper_.add_waypoint(map_name, location);
    
    // Copy waypoint properties
    if (full_wp.is_holding_point())
      filtered_wp.set_holding_point(true);
    if (full_wp.is_passthrough_point())
      filtered_wp.set_passthrough_point(true);
    if (full_wp.is_parking_spot())
      filtered_wp.set_parking_spot(true);
    if (full_wp.is_charger())
      filtered_wp.set_charger(true);
    if (full_wp.in_lift())
      filtered_wp.set_in_lift(full_wp.in_lift());
    if (full_wp.in_mutex_group() != "")
      filtered_wp.set_in_mutex_group(full_wp.in_mutex_group());
    if (full_wp.merge_radius())
      filtered_wp.set_merge_radius(full_wp.merge_radius());
    
    // Copy waypoint name if it exists
    if (full_wp.name())
    {
      filtered_graph_wrapper_.add_key(*full_wp.name(), filtered_idx);
    }
    
    // Create index mapping
    full_to_filtered_map_[full_idx] = filtered_idx;
    filtered_to_full_map_[filtered_idx] = full_idx;
    
    filtered_idx++;
  }
  
  std::cout << "[ZoneGraphFilter]     ✓ Added " << filtered_idx 
            << " waypoints with index remapping" << std::endl;
  
  // Step 2: Add zone lanes to filtered graph (with remapped indices)
  std::cout << "[ZoneGraphFilter]     Adding " << zone_lanes.size() << " zone lanes..." << std::endl;
  std::size_t lanes_added = 0;
  std::size_t lanes_skipped = 0;
  
  for (std::size_t full_lane_idx : zone_lanes)
  {
    const auto& full_lane = full_graph_.lanes[full_lane_idx];
    
    // Get entry and exit waypoint indices in full graph
    std::size_t entry_full = full_lane.entry().waypoint_index();
    std::size_t exit_full = full_lane.exit().waypoint_index();
    
    // Get corresponding filtered indices
    auto entry_filtered_it = full_to_filtered_map_.find(entry_full);
    auto exit_filtered_it = full_to_filtered_map_.find(exit_full);
    
    if (entry_filtered_it == full_to_filtered_map_.end() ||
        exit_filtered_it == full_to_filtered_map_.end())
    {
      // Should not happen if filtering logic is correct
      std::cerr << "[ZoneGraphFilter] ⚠️  Warning: Lane " << full_lane_idx 
                << " references waypoint not in zone waypoints (skipping)" << std::endl;
      lanes_skipped++;
      continue;
    }
    
    std::size_t entry_filtered = entry_filtered_it->second;
    std::size_t exit_filtered = exit_filtered_it->second;
    
    // Create lane nodes with filtered indices
    // Copy entry node properties (event, orientation constraint)
    Graph::Lane::Node entry_node(
      entry_filtered,
      full_lane.entry().event() ? full_lane.entry().event()->clone() : nullptr,
      full_lane.entry().orientation_constraint() ? 
        full_lane.entry().orientation_constraint()->clone() : nullptr);
    
    // Copy exit node properties
    Graph::Lane::Node exit_node(
      exit_filtered,
      full_lane.exit().event() ? full_lane.exit().event()->clone() : nullptr,
      full_lane.exit().orientation_constraint() ? 
        full_lane.exit().orientation_constraint()->clone() : nullptr);
    
    // Copy lane properties
    Graph::Lane::Properties lane_properties;
    if (full_lane.properties().speed_limit())
      lane_properties.speed_limit(full_lane.properties().speed_limit());
    if (full_lane.properties().in_mutex_group() != "")
      lane_properties.set_in_mutex_group(full_lane.properties().in_mutex_group());
    
    // Add lane to filtered graph
    filtered_graph_wrapper_.add_lane(entry_node, exit_node, lane_properties);
    lanes_added++;
  }
  
  std::cout << "[ZoneGraphFilter]     ✓ Added " << lanes_added << " lanes" << std::endl;
  if (lanes_skipped > 0)
  {
    std::cout << "[ZoneGraphFilter]     ⚠️  Skipped " << lanes_skipped << " lanes (invalid references)" << std::endl;
  }
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

