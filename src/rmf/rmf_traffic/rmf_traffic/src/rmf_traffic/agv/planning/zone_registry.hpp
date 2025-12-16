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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_REGISTRY_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_REGISTRY_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include <cstddef>

// Forward declaration
namespace rmf_traffic {
namespace agv {
class Graph;
} // namespace agv
} // namespace rmf_traffic

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// ZoneRegistry: Stores zone definitions loaded from building.yaml
/// 
/// Zones are defined in building.yaml under each level:
/// levels:
///   L1:
///     zones:
///       - id: zone_1
///         name: Zone 1
///         waypoints: [10, 11, 12, 13]
///         lanes: [5, 6, 7]
class ZoneRegistry
{
public:
  /// Zone definition structure
  struct Zone
  {
    std::string id;                    // Zone identifier
    std::string name;                  // Zone display name
    std::vector<std::size_t> waypoints; // Waypoint indices in this zone
    std::vector<std::size_t> lanes;    // Lane indices in this zone
  };

  /// Constructor - creates empty registry
  ZoneRegistry();

  /// Load zones from YAML node (from building.yaml)
  /// 
  /// \param[in] yaml_file Path to building.yaml file
  /// \param[in] level_name Name of the level (e.g., "L1")
  /// \param[in] graph Optional graph to convert YAML vertex indices to graph waypoint indices
  ///                  If provided, will convert indices using waypoint names
  void load_from_yaml(
    const std::string& yaml_file, 
    const std::string& level_name,
    const Graph* graph = nullptr);

  /// Add a zone to the registry
  /// 
  /// \param[in] zone Zone to add
  void add_zone(const Zone& zone);

  /// Get zones that contain a specific waypoint
  /// 
  /// \param[in] waypoint_index Index of the waypoint
  /// \return Vector of zone IDs that contain this waypoint
  std::vector<std::string> get_zones_for_waypoint(std::size_t waypoint_index) const;

  /// Get zones that contain a specific lane
  /// 
  /// \param[in] lane_index Index of the lane
  /// \return Vector of zone IDs that contain this lane
  std::vector<std::string> get_zones_for_lane(std::size_t lane_index) const;

  /// Get all zones
  /// 
  /// \return Vector of all zones
  const std::vector<Zone>& get_all_zones() const;

  /// Get a zone by ID
  /// 
  /// \param[in] zone_id Zone identifier
  /// \return Pointer to zone if found, nullptr otherwise
  const Zone* get_zone(const std::string& zone_id) const;

  /// Check if a waypoint belongs to any zone
  /// 
  /// \param[in] waypoint_index Index of the waypoint
  /// \return True if waypoint belongs to at least one zone
  bool is_waypoint_in_zone(std::size_t waypoint_index) const;

  /// Check if a lane belongs to any zone
  /// 
  /// \param[in] lane_index Index of the lane
  /// \return True if lane belongs to at least one zone
  bool is_lane_in_zone(std::size_t lane_index) const;

  /// Get number of zones
  /// 
  /// \return Number of zones in registry
  std::size_t num_zones() const;

private:
  std::vector<Zone> zones_;  // All zones
  
  // Maps waypoint index -> set of zone IDs
  std::unordered_map<std::size_t, std::set<std::string>> waypoint_to_zones_;
  
  // Maps lane index -> set of zone IDs
  std::unordered_map<std::size_t, std::set<std::string>> lane_to_zones_;
  
  // Map zone ID -> zone index in zones_ vector
  std::unordered_map<std::string, std::size_t> zone_id_to_index_;
  
  /// Update internal mappings after adding a zone
  void update_mappings(const Zone& zone, std::size_t zone_index);
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__ZONE_REGISTRY_HPP





