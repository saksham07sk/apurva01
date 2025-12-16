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

#include "zone_registry.hpp"
#include <rmf_traffic/agv/Graph.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <algorithm>
#include <unordered_map>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
ZoneRegistry::ZoneRegistry()
{
  // Constructor - empty registry
}

//==============================================================================
void ZoneRegistry::load_from_yaml(
  const std::string& yaml_file,
  const std::string& level_name,
  const Graph* graph)
{
  // Load YAML file
  const YAML::Node config = YAML::LoadFile(yaml_file);
  if (!config)
  {
    std::cerr << "[ZoneRegistry] Warning: Failed to load YAML file [" 
              << yaml_file << "]" << std::endl;
    return;
  }

  // Navigate to levels -> level_name -> zones
  const YAML::Node levels = config["levels"];
  if (!levels || !levels.IsMap())
  {
    std::cout << "[ZoneRegistry] No zones found in file (no levels section)" << std::endl;
    return;
  }

  const YAML::Node level = levels[level_name];
  if (!level)
  {
    std::cout << "[ZoneRegistry] Level [" << level_name << "] not found in file" << std::endl;
    return;
  }

  const YAML::Node zones_node = level["zones"];
  if (!zones_node || !zones_node.IsSequence())
  {
    std::cout << "[ZoneRegistry] No zones defined in level [" << level_name << "]" << std::endl;
    return;
  }

  // ===========================================================================
  // CRITICAL FIX: Create mapping from YAML vertex indices to graph waypoint indices
  // ===========================================================================
  // The zone definition uses YAML vertex indices, but the graph uses graph waypoint indices.
  // We need to convert by matching waypoint names.
  std::unordered_map<std::size_t, std::size_t> yaml_to_graph_index_map;
  
  if (graph)
  {
    std::cout << "[ZoneRegistry] 🔄 Converting YAML vertex indices to graph waypoint indices..." << std::endl;
    
    // Get vertices from YAML
    const YAML::Node& vertices = level["vertices"];
    if (vertices && vertices.IsSequence())
    {
      for (std::size_t yaml_idx = 0; yaml_idx < vertices.size(); ++yaml_idx)
      {
        const auto& vertex = vertices[yaml_idx];
        
        // Extract waypoint name from YAML vertex
        // Format: [x, y, z, name] or [x, y, z, {name: "...", ...}]
        std::string wp_name;
        if (vertex.IsSequence() && vertex.size() > 3)
        {
          const auto& name_val = vertex[3];
          if (name_val.IsScalar())
          {
            wp_name = name_val.as<std::string>();
          }
          else if (name_val.IsMap() && name_val["name"])
          {
            wp_name = name_val["name"].as<std::string>();
          }
        }
        
        // If waypoint has a name, find it in the graph
        if (!wp_name.empty())
        {
          const auto* graph_wp = graph->find_waypoint(wp_name);
          if (graph_wp)
          {
            std::size_t graph_idx = graph_wp->index();
            yaml_to_graph_index_map[yaml_idx] = graph_idx;
          }
          else
          {
            // Waypoint has a name but not found in graph - use YAML index as fallback
            // This can happen if waypoint was removed from graph or graph was built differently
            yaml_to_graph_index_map[yaml_idx] = yaml_idx;
          }
        }
        else
        {
          // If no name, assume YAML index matches graph index (if no extra waypoints added)
          // This is a fallback for waypoints without names
          yaml_to_graph_index_map[yaml_idx] = yaml_idx;
        }
      }
    }
    
    std::cout << "[ZoneRegistry]   ✓ Created mapping for " << yaml_to_graph_index_map.size() 
              << " waypoints" << std::endl;
  }

  // Parse each zone
  std::cout << "[ZoneRegistry] ========================================" << std::endl;
  std::cout << "[ZoneRegistry] LOADING ZONES FROM YAML" << std::endl;
  std::cout << "[ZoneRegistry]   File: " << yaml_file << std::endl;
  std::cout << "[ZoneRegistry]   Level: " << level_name << std::endl;
  std::cout << "[ZoneRegistry]   Found " << zones_node.size() << " zone(s)" << std::endl;
  std::cout << "[ZoneRegistry] ----------------------------------------" << std::endl;
  
  for (const auto& zone_node : zones_node)
  {
    Zone zone;
    
    // Get zone ID
    if (zone_node["id"])
    {
      zone.id = zone_node["id"].as<std::string>();
    }
    else
    {
      std::cerr << "[ZoneRegistry] ⚠️  Warning: Zone missing 'id' field, skipping" << std::endl;
      continue;
    }

    // Get zone name (optional)
    if (zone_node["name"])
    {
      zone.name = zone_node["name"].as<std::string>();
    }
    else
    {
      zone.name = zone.id;  // Use ID as name if name not provided
    }

    // Get waypoints (optional) - CONVERT YAML INDICES TO GRAPH INDICES
    if (zone_node["waypoints"] && zone_node["waypoints"].IsSequence())
    {
      std::size_t converted_count = 0;
      std::size_t failed_count = 0;
      
      for (const auto& wp : zone_node["waypoints"])
      {
        std::size_t yaml_idx = wp.as<std::size_t>();
        
        if (graph && !yaml_to_graph_index_map.empty())
        {
          // Convert YAML vertex index to graph waypoint index
          const auto it = yaml_to_graph_index_map.find(yaml_idx);
          if (it != yaml_to_graph_index_map.end())
          {
            // Check if the mapped graph index is valid
            if (it->second < graph->num_waypoints())
            {
              zone.waypoints.push_back(it->second);
              converted_count++;
            }
            else
            {
              // Mapped index is out of bounds - use YAML index as fallback
              zone.waypoints.push_back(yaml_idx);
              failed_count++;
              std::cerr << "[ZoneRegistry] ⚠️  Warning: Mapped graph index " << it->second 
                        << " for YAML vertex index " << yaml_idx 
                        << " is out of bounds (graph has " << graph->num_waypoints() 
                        << " waypoints). Using YAML index as fallback." << std::endl;
            }
          }
          else
          {
            // Mapping not found - this shouldn't happen with the improved mapping logic,
            // but handle it gracefully
            // Check if YAML index is already a valid graph index
            if (yaml_idx < graph->num_waypoints())
            {
              zone.waypoints.push_back(yaml_idx);
              converted_count++;
            }
            else
            {
              zone.waypoints.push_back(yaml_idx);
              failed_count++;
              std::cerr << "[ZoneRegistry] ⚠️  Warning: No mapping found for YAML vertex index " 
                        << yaml_idx << " and index is out of bounds (graph has " 
                        << graph->num_waypoints() << " waypoints). Using as-is." << std::endl;
            }
          }
        }
        else
        {
          // No graph provided, use YAML index directly (assumes they match)
          zone.waypoints.push_back(yaml_idx);
        }
      }
      
      if (graph && converted_count > 0)
      {
        std::cout << "[ZoneRegistry]     - Converted " << converted_count 
                  << " waypoint indices (YAML → Graph)" << std::endl;
        if (failed_count > 0)
        {
          std::cerr << "[ZoneRegistry]     - ⚠️  Failed to convert " << failed_count 
                    << " waypoint indices" << std::endl;
        }
      }
    }

    // Get lanes (optional) - Lane indices should be the same in YAML and graph
    // (assuming lanes are added in the same order)
    if (zone_node["lanes"] && zone_node["lanes"].IsSequence())
    {
      for (const auto& lane : zone_node["lanes"])
      {
        zone.lanes.push_back(lane.as<std::size_t>());
      }
    }

    // Add zone to registry
    add_zone(zone);
    
    std::cout << "[ZoneRegistry]   ✓ Zone: " << zone.id 
              << " (name: " << zone.name << ")" << std::endl;
    std::cout << "[ZoneRegistry]     - Waypoints: " << zone.waypoints.size() << std::endl;
    if (!zone.waypoints.empty())
    {
      std::cout << "[ZoneRegistry]       [";
      for (std::size_t i = 0; i < zone.waypoints.size() && i < 10; ++i)
      {
        std::cout << zone.waypoints[i];
        if (i < zone.waypoints.size() - 1 && i < 9) std::cout << ", ";
      }
      if (zone.waypoints.size() > 10) std::cout << ", ...";
      std::cout << "]" << std::endl;
    }
    std::cout << "[ZoneRegistry]     - Lanes: " << zone.lanes.size() << std::endl;
    if (!zone.lanes.empty())
    {
      std::cout << "[ZoneRegistry]       [";
      for (std::size_t i = 0; i < zone.lanes.size() && i < 10; ++i)
      {
        std::cout << zone.lanes[i];
        if (i < zone.lanes.size() - 1 && i < 9) std::cout << ", ";
      }
      if (zone.lanes.size() > 10) std::cout << ", ...";
      std::cout << "]" << std::endl;
    }
  }

  std::cout << "[ZoneRegistry] ----------------------------------------" << std::endl;
  std::cout << "[ZoneRegistry] ✅ Successfully loaded " << zones_.size() << " zone(s)" << std::endl;
  std::cout << "[ZoneRegistry] ========================================" << std::endl;
}

//==============================================================================
void ZoneRegistry::add_zone(const Zone& zone)
{
  std::size_t zone_index = zones_.size();
  zones_.push_back(zone);
  zone_id_to_index_[zone.id] = zone_index;
  
  update_mappings(zone, zone_index);
}

//==============================================================================
std::vector<std::string> ZoneRegistry::get_zones_for_waypoint(
  std::size_t waypoint_index) const
{
  const auto it = waypoint_to_zones_.find(waypoint_index);
  if (it != waypoint_to_zones_.end())
  {
    return std::vector<std::string>(it->second.begin(), it->second.end());
  }
  return std::vector<std::string>();
}

//==============================================================================
std::vector<std::string> ZoneRegistry::get_zones_for_lane(
  std::size_t lane_index) const
{
  const auto it = lane_to_zones_.find(lane_index);
  if (it != lane_to_zones_.end())
  {
    return std::vector<std::string>(it->second.begin(), it->second.end());
  }
  return std::vector<std::string>();
}

//==============================================================================
const std::vector<ZoneRegistry::Zone>& ZoneRegistry::get_all_zones() const
{
  return zones_;
}

//==============================================================================
const ZoneRegistry::Zone* ZoneRegistry::get_zone(const std::string& zone_id) const
{
  const auto it = zone_id_to_index_.find(zone_id);
  if (it != zone_id_to_index_.end())
  {
    return &zones_[it->second];
  }
  return nullptr;
}

//==============================================================================
bool ZoneRegistry::is_waypoint_in_zone(std::size_t waypoint_index) const
{
  return waypoint_to_zones_.count(waypoint_index) > 0;
}

//==============================================================================
bool ZoneRegistry::is_lane_in_zone(std::size_t lane_index) const
{
  return lane_to_zones_.count(lane_index) > 0;
}

//==============================================================================
std::size_t ZoneRegistry::num_zones() const
{
  return zones_.size();
}

//==============================================================================
void ZoneRegistry::update_mappings(const Zone& zone, std::size_t /* zone_index */)
{
  // Update waypoint mappings
  for (std::size_t wp_idx : zone.waypoints)
  {
    waypoint_to_zones_[wp_idx].insert(zone.id);
  }

  // Update lane mappings
  for (std::size_t lane_idx : zone.lanes)
  {
    lane_to_zones_[lane_idx].insert(zone.id);
  }
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

