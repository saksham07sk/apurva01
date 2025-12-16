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

#include "zone_initialization.hpp"
#include <iostream>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
std::unique_ptr<ZoneRegistry> ZoneInitialization::registry_ = nullptr;

//==============================================================================
bool ZoneInitialization::initialize_from_building_map(
  const Graph& full_graph,
  const std::string& building_map_path,
  const std::string& level_name)
{
  std::cout << "\n[ZoneInitialization] ========================================" << std::endl;
  std::cout << "[ZoneInitialization] 🎯 INITIALIZING ZONE-BASED PLANNING" << std::endl;
  std::cout << "[ZoneInitialization]   Building map: " << building_map_path << std::endl;
  std::cout << "[ZoneInitialization]   Level: " << level_name << std::endl;
  std::cout << "[ZoneInitialization] ----------------------------------------" << std::endl;

  // Create zone registry
  registry_ = std::make_unique<ZoneRegistry>();

  // Load zones from building.yaml
  // CRITICAL: Pass the graph to convert YAML vertex indices to graph waypoint indices
  registry_->load_from_yaml(building_map_path, level_name, &full_graph);

  // Check if zones were loaded
  if (registry_->num_zones() == 0)
  {
    std::cout << "[ZoneInitialization] ⚠️  No zones found - zone-based planning disabled" << std::endl;
    std::cout << "[ZoneInitialization] ========================================\n" << std::endl;
    registry_.reset();
    return false;
  }

  // Initialize zone planner helper
  ZonePlannerHelper::get_instance().initialize(
    full_graph,
    *registry_,
    building_map_path);

  std::cout << "[ZoneInitialization] ✅ Zone-based planning initialized successfully" << std::endl;
  std::cout << "[ZoneInitialization]   Zones: " << registry_->num_zones() << std::endl;
  std::cout << "[ZoneInitialization] ========================================\n" << std::endl;

  return true;
}

//==============================================================================
bool ZoneInitialization::initialize_from_registry(
  const Graph& full_graph,
  const ZoneRegistry& zone_registry,
  const std::string& building_map_path)
{
  std::cout << "\n[ZoneInitialization] ========================================" << std::endl;
  std::cout << "[ZoneInitialization] 🎯 INITIALIZING ZONE-BASED PLANNING" << std::endl;
  std::cout << "[ZoneInitialization]   From zone registry" << std::endl;
  std::cout << "[ZoneInitialization]   Zones: " << zone_registry.num_zones() << std::endl;
  std::cout << "[ZoneInitialization] ----------------------------------------" << std::endl;

  // Check if zones are defined
  if (zone_registry.num_zones() == 0)
  {
    std::cout << "[ZoneInitialization] ⚠️  No zones in registry - zone-based planning disabled" << std::endl;
    std::cout << "[ZoneInitialization] ========================================\n" << std::endl;
    return false;
  }

  // Store reference to registry (create a copy)
  registry_ = std::make_unique<ZoneRegistry>(zone_registry);

  // Initialize zone planner helper
  ZonePlannerHelper::get_instance().initialize(
    full_graph,
    *registry_,
    building_map_path);

  std::cout << "[ZoneInitialization] ✅ Zone-based planning initialized successfully" << std::endl;
  std::cout << "[ZoneInitialization] ========================================\n" << std::endl;

  return true;
}

//==============================================================================
bool ZoneInitialization::is_initialized()
{
  return registry_ != nullptr && registry_->num_zones() > 0;
}

//==============================================================================
const ZoneRegistry* ZoneInitialization::get_registry()
{
  return registry_.get();
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic


