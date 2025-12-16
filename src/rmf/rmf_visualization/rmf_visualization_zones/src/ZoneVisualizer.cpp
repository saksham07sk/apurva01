#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include <optional>
#include <limits>
#include <cmath>
#include <map>
#include <set>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

struct Vertex
{
  double x;
  double y;
  double z;
  std::string name;
};

class ZoneVisualizerNode : public rclcpp::Node
{
public:
  ZoneVisualizerNode()
  : rclcpp::Node("zone_visualizer")
  {
    building_map_path_ = this->declare_parameter<std::string>(
      "building_map_path",
      "");
    nav_graph_path_ = this->declare_parameter<std::string>(
      "nav_graph_path",
      ""); // Path to nav_graphs/0.yaml
    level_name_ = this->declare_parameter<std::string>(
      "level_name",
      "L1");
    frame_id_ = this->declare_parameter<std::string>(
      "frame_id",
      "map");
    
    // Auto-detect frame from navgraph if map frame doesn't exist
    // This ensures compatibility with RMF simulations
    RCLCPP_INFO(
      get_logger(),
      "Zone visualizer using frame_id: [%s]",
      frame_id_.c_str());
    line_width_ = this->declare_parameter<double>(
      "line_width",
      0.2); // Increased for better visibility (0.2-0.3 range)
    marker_alpha_ = this->declare_parameter<double>(
      "alpha",
      0.6); // Slightly more transparent to see map through zone
    waypoint_scale_ = this->declare_parameter<double>(
      "waypoint_scale",
      0.8); // Scale for waypoint markers (increased for better visibility without zooming)
    
    // Coordinate offset to align with map (if map has origin offset)
    offset_x_ = this->declare_parameter<double>(
      "offset_x",
      0.0);
    offset_y_ = this->declare_parameter<double>(
      "offset_y",
      0.0);
    offset_z_ = this->declare_parameter<double>(
      "offset_z",
      0.0);
    
    // Scale factors (default 1.0 = no scaling)
    scale_x_ = 1.0;
    scale_y_ = 1.0;
    
    // Initialize zone bounds tracking
    zone_bounds_calculated_ = false;
    zone_actual_min_x_ = zone_actual_max_x_ = 0.0;
    zone_actual_min_y_ = zone_actual_max_y_ = 0.0;
    should_publish_zones_ = false; // Don't publish until navgraph alignment is done
    
    if (offset_x_ != 0.0 || offset_y_ != 0.0 || offset_z_ != 0.0)
    {
      RCLCPP_INFO(
        get_logger(),
        "Applying coordinate offset: x=%.2f, y=%.2f, z=%.2f",
        offset_x_, offset_y_, offset_z_);
    }

    if (building_map_path_.empty())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Parameter 'building_map_path' is empty. Set it to the building YAML path.");
      throw std::runtime_error("building_map_path not set");
    }

    // Auto-detect map offset from building map images
    auto_detect_map_offset();
    
    // Load nav_graphs waypoint positions (name -> position mapping)
    load_nav_graph_waypoints();

    // Use same QoS as navgraph visualizer for consistency
    const auto transient_qos = rclcpp::QoS(10).transient_local();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "zone_markers",
      transient_qos);

    // Subscribe to navgraph markers to extract actual waypoint positions and align zone
    navgraph_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/map_markers",
      rclcpp::QoS(10).transient_local(),
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
      {
        // Even if we prefer YAML vertices, keep navgraph data to compute scale/offset alignment.
        // Extract waypoint positions from navgraph markers
        // Navgraph visualizer publishes waypoints as POINTS markers where:
        // - namespace: "fleet_name/waypoints/map_name"
        // - marker.points array index corresponds to graph waypoint index
        // - All waypoints for a map are in a single POINTS marker
        for (const auto& marker : msg->markers)
        {
          // Look for waypoint markers (POINTS type with waypoints in namespace)
          if (marker.type == visualization_msgs::msg::Marker::POINTS &&
              marker.ns.find("/waypoints/") != std::string::npos &&
              !marker.points.empty())
          {
            // Store waypoint positions: array index = graph waypoint index
            for (std::size_t i = 0; i < marker.points.size(); ++i)
            {
              waypoint_positions_[i] = marker.points[i];
            }
            RCLCPP_INFO(
              get_logger(),
              "Extracted %zu waypoint positions from navgraph markers (namespace: %s)",
              marker.points.size(), marker.ns.c_str());
            
            // Debug: Show first few and last few waypoint positions
            if (marker.points.size() > 0)
            {
              RCLCPP_INFO(
                get_logger(),
                "  Sample waypoint positions: [0]=(%.2f,%.2f), [1]=(%.2f,%.2f), [%zu]=(%.2f,%.2f)",
                marker.points[0].x, marker.points[0].y,
                marker.points.size() > 1 ? marker.points[1].x : 0.0,
                marker.points.size() > 1 ? marker.points[1].y : 0.0,
                marker.points.size() - 1,
                marker.points.back().x, marker.points.back().y);
            }
          }
        }
        
        // If we have zone waypoint IDs and waypoint positions, calculate actual zone bounds
        if (!zone_waypoint_ids_.empty() && !waypoint_positions_.empty())
        {
          bool found_zone_waypoints = false;
          double zone_min_x = std::numeric_limits<double>::max();
          double zone_max_x = std::numeric_limits<double>::lowest();
          double zone_min_y = std::numeric_limits<double>::max();
          double zone_max_y = std::numeric_limits<double>::lowest();
          int matched_waypoints = 0;
          
          // For each zone, find the bounds of its waypoints
          for (const auto& [zone_id, waypoint_set] : zone_waypoint_ids_)
          {
            RCLCPP_INFO(
              get_logger(),
              "Checking zone [%s] with %zu waypoint IDs against %zu available waypoint positions",
              zone_id.c_str(), waypoint_set.size(), waypoint_positions_.size());
            
            // Debug: Show first 10 waypoint IDs from zone
            int sample_count = 0;
            std::vector<std::size_t> sample_wp_ids;
            for (const auto& wp_idx : waypoint_set)
            {
              if (sample_count < 10)
              {
                sample_wp_ids.push_back(wp_idx);
                sample_count++;
              }
              
              if (waypoint_positions_.find(wp_idx) != waypoint_positions_.end())
              {
                const auto& pos = waypoint_positions_[wp_idx];
                if (pos.x < zone_min_x) zone_min_x = pos.x;
                if (pos.x > zone_max_x) zone_max_x = pos.x;
                if (pos.y < zone_min_y) zone_min_y = pos.y;
                if (pos.y > zone_max_y) zone_max_y = pos.y;
                found_zone_waypoints = true;
                matched_waypoints++;
              }
              else
              {
                if (matched_waypoints < 5) // Only log first few misses
                {
                  RCLCPP_WARN(
                    get_logger(),
                    "Zone [%s] waypoint index %zu NOT FOUND in navgraph (max available: %zu)",
                    zone_id.c_str(), wp_idx, waypoint_positions_.empty() ? 0 : waypoint_positions_.rbegin()->first);
                }
              }
            }
            
            // Show sample waypoint IDs and their positions
            if (!sample_wp_ids.empty())
            {
              std::string sample_info = "Sample zone waypoint IDs: ";
              for (size_t i = 0; i < sample_wp_ids.size() && i < 5; ++i)
              {
                std::size_t wp_idx = sample_wp_ids[i];
                if (waypoint_positions_.find(wp_idx) != waypoint_positions_.end())
                {
                  const auto& pos = waypoint_positions_[wp_idx];
                  sample_info += std::to_string(wp_idx) + "=(" + 
                    std::to_string(pos.x) + "," + std::to_string(pos.y) + ") ";
                }
                else
                {
                  sample_info += std::to_string(wp_idx) + "=NOT_FOUND ";
                }
              }
              RCLCPP_INFO(get_logger(), "%s", sample_info.c_str());
            }
          }
          
          RCLCPP_INFO(
            get_logger(),
            "Zone waypoint matching: found=%d, matched=%d waypoints",
            found_zone_waypoints ? 1 : 0, matched_waypoints);
          
          if (found_zone_waypoints && matched_waypoints > 0)
          {
            // Calculate zone bounds from ACTUAL YAML vertices (NOT hardcoded!)
            // Load vertices from YAML to get actual zone bounds
            double zone_yaml_min_x = std::numeric_limits<double>::max();
            double zone_yaml_max_x = std::numeric_limits<double>::lowest();
            double zone_yaml_min_y = std::numeric_limits<double>::max();
            double zone_yaml_max_y = std::numeric_limits<double>::lowest();
            
            // Get zone vertex indices and calculate bounds from actual YAML coordinates
            for (const auto& [zone_id, waypoint_set] : zone_waypoint_ids_)
            {
              // Load YAML to get vertex coordinates
              YAML::Node root;
              try
              {
                root = YAML::LoadFile(building_map_path_);
                const auto levels = root["levels"];
                if (levels && levels.IsMap())
                {
                  const auto level = levels[level_name_];
                  if (level && level.IsMap())
                  {
                    const auto zones_node = level["zones"];
                    if (zones_node && zones_node.IsSequence())
                    {
                      for (const auto& zone_node : zones_node)
                      {
                        if (zone_node["id"] && zone_node["id"].as<std::string>() == zone_id)
                        {
                          // Get vertices for this zone
                          if (zone_node["vertices"] && zone_node["vertices"].IsSequence())
                          {
                            const auto vertices_node = level["vertices"];
                            if (vertices_node && vertices_node.IsSequence())
                            {
                              for (const auto& idx_node : zone_node["vertices"])
                              {
                                const std::size_t idx = idx_node.as<std::size_t>();
                                if (idx < vertices_node.size() && vertices_node[idx].IsSequence() && vertices_node[idx].size() >= 2)
                                {
                                  double x = vertices_node[idx][0].as<double>();
                                  double y = vertices_node[idx][1].as<double>();
                                  if (x < zone_yaml_min_x) zone_yaml_min_x = x;
                                  if (x > zone_yaml_max_x) zone_yaml_max_x = x;
                                  if (y < zone_yaml_min_y) zone_yaml_min_y = y;
                                  if (y > zone_yaml_max_y) zone_yaml_max_y = y;
                                }
                              }
                            }
                          }
                          break;
                        }
                      }
                    }
                  }
                }
              }
              catch (const std::exception& e)
              {
                RCLCPP_ERROR(get_logger(), "Failed to load zone vertices from YAML: %s", e.what());
                // Fallback to hardcoded values only if YAML load fails
                zone_yaml_min_x = 12.0;
                zone_yaml_min_y = -808.0;
                zone_yaml_max_x = 892.0;
                zone_yaml_max_y = -408.0;
              }
            }
            
            double zone_yaml_width = zone_yaml_max_x - zone_yaml_min_x;
            double zone_yaml_height = std::abs(zone_yaml_max_y - zone_yaml_min_y);
            
            RCLCPP_INFO(
              get_logger(),
              "Zone YAML bounds (from actual vertices): X[%.2f, %.2f] (w=%.2f), Y[%.2f, %.2f] (h=%.2f)",
              zone_yaml_min_x, zone_yaml_max_x, zone_yaml_width, zone_yaml_min_y, zone_yaml_max_y, zone_yaml_height);
            
            // Calculate actual zone size from waypoint positions
            double zone_actual_width = zone_max_x - zone_min_x;
            double zone_actual_height = std::abs(zone_max_y - zone_min_y);
            
            // Calculate scale factors: actual_size / yaml_size
            double scale_x = (zone_yaml_width > 0.1 && zone_actual_width > 0.1) ? 
                             zone_actual_width / zone_yaml_width : 1.0;
            double scale_y = (zone_yaml_height > 0.1 && zone_actual_height > 0.1) ? 
                             zone_actual_height / zone_yaml_height : 1.0;
            
            // Calculate offset: actual_zone_min - (yaml_zone_min * scale)
            double new_offset_x = zone_min_x - (zone_yaml_min_x * scale_x);
            double new_offset_y = zone_min_y - (zone_yaml_min_y * scale_y);
            
            // Store calculated bounds
            zone_actual_min_x_ = zone_min_x;
            zone_actual_max_x_ = zone_max_x;
            zone_actual_min_y_ = zone_min_y;
            zone_actual_max_y_ = zone_max_y;
            zone_bounds_calculated_ = true;
            
            // Only update if significantly different (avoid constant republishing)
            if (std::abs(new_offset_x - offset_x_) > 0.1 || std::abs(new_offset_y - offset_y_) > 0.1 ||
                std::abs(scale_x_ - scale_x) > 0.01 || std::abs(scale_y_ - scale_y) > 0.01)
            {
              offset_x_ = new_offset_x;
              offset_y_ = new_offset_y;
              scale_x_ = scale_x;
              scale_y_ = scale_y;
              
              RCLCPP_INFO(
                get_logger(),
                "✅ Auto-aligned zone with actual waypoint positions (matched %d waypoints):",
                matched_waypoints);
              RCLCPP_INFO(
                get_logger(),
                "  Actual zone bounds: X[%.2f, %.2f] (w=%.2f), Y[%.2f, %.2f] (h=%.2f)",
                zone_min_x, zone_max_x, zone_actual_width, zone_min_y, zone_max_y, zone_actual_height);
              RCLCPP_INFO(
                get_logger(),
                "  YAML zone bounds: X[%.2f, %.2f] (w=%.2f), Y[%.2f, %.2f] (h=%.2f)",
                zone_yaml_min_x, zone_yaml_max_x, zone_yaml_width, zone_yaml_min_y, zone_yaml_max_y, zone_yaml_height);
              RCLCPP_INFO(
                get_logger(),
                "  Scale factors: x=%.4f, y=%.4f, Offset: x=%.2f, y=%.2f",
                scale_x_, scale_y_, offset_x_, offset_y_);
              
              // CRITICAL: Set flag to allow publishing after alignment
              should_publish_zones_ = true;
              publish_zones(); // Republish with new offset and scale
            }
            else if (!should_publish_zones_)
            {
              // Alignment already done, just enable publishing
              should_publish_zones_ = true;
              RCLCPP_INFO(get_logger(), "✅ Zone alignment complete - enabling zone publishing");
              publish_zones();
            }
          }
          else if (!found_zone_waypoints)
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              5000,
              "Zone waypoints not found in navgraph markers. Waiting for navgraph data...");
          }
        }
      });
    
    // Subscribe to floorplan to get actual map origin (if available)
    floorplan_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/floorplan",
      rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
      {
        // Update offset based on actual floorplan origin
        if (msg->info.origin.position.x != 0.0 || msg->info.origin.position.y != 0.0)
        {
          offset_x_ = -msg->info.origin.position.x;
          offset_y_ = -msg->info.origin.position.y;
          RCLCPP_INFO(
            get_logger(),
            "Auto-detected map offset from floorplan: x=%.2f, y=%.2f",
            offset_x_, offset_y_);
          publish_zones(); // Republish with new offset
        }
      });

    // Load zone data (waypoint IDs) without publishing markers
    // This ensures zone_waypoint_ids_ is populated for navgraph matching
    load_zone_data();
    // If any zone has explicit vertex_names or user prefers YAML-only, publish immediately;
    // navgraph alignment will still run later and update scale/offset before republishing.
    if (prefer_yaml_vertices_ || any_zone_has_vertex_names_)
    {
      should_publish_zones_ = true;
      publish_zones();
    }
    if (prefer_yaml_vertices_)
    {
      // When using YAML vertices directly, publish immediately without waiting for navgraph
      should_publish_zones_ = true;
      publish_zones();
    }
    
    // Create a timer to republish zones periodically (every 5 seconds)
    // This ensures RViz gets the markers even if it connects late
    // Reduced frequency to avoid flickering - only republish every 30 seconds
    timer_ = this->create_wall_timer(
      std::chrono::seconds(30),
      [this]() { this->publish_zones(); });
  }

private:
  void load_zone_data()
  {
    // Load zone waypoint IDs from YAML without publishing markers
    YAML::Node root;
    try
    {
      root = YAML::LoadFile(building_map_path_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to load building map [%s]: %s",
        building_map_path_.c_str(),
        e.what());
      return;
    }

    const auto levels = root["levels"];
    if (!levels || !levels.IsMap())
    {
      RCLCPP_ERROR(get_logger(), "No 'levels' section in building map");
      return;
    }

    const auto level = levels[level_name_];
    if (!level || !level.IsMap())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Level [%s] not found in building map",
        level_name_.c_str());
      return;
    }

    const auto zones_node = level["zones"];
    if (!zones_node || !zones_node.IsSequence())
    {
      RCLCPP_ERROR(get_logger(), "Level [%s] has no zones", level_name_.c_str());
      return;
    }

    // Clear existing zone data to reload fresh
    zone_waypoint_ids_.clear();
    any_zone_has_vertex_names_ = false;
    
    // Load zone waypoint IDs and vertices
    for (const auto& zone_node : zones_node)
    {
      const std::string zone_id = zone_node["id"] ? zone_node["id"].as<std::string>() : "zone";
      
      // Store zone waypoint IDs for alignment with navgraph
      if (zone_node["waypoints"] && zone_node["waypoints"].IsSequence())
      {
        std::set<std::size_t> waypoint_set;
        for (const auto& wp_node : zone_node["waypoints"])
        {
          waypoint_set.insert(wp_node.as<std::size_t>());
        }
        zone_waypoint_ids_[zone_id] = waypoint_set;
        RCLCPP_INFO(
          get_logger(),
          "Loaded zone [%s] with %zu waypoint IDs",
          zone_id.c_str(), waypoint_set.size());
      }
      // Track if this zone provides vertex_names (forces YAML-only mode)
      if (zone_node["vertex_names"] && zone_node["vertex_names"].IsSequence())
      {
        any_zone_has_vertex_names_ = true;
      }
      
      // Also log zone vertices for debugging
      if (zone_node["vertices"] && zone_node["vertices"].IsSequence())
      {
        std::vector<std::size_t> vertex_indices;
        for (const auto& v_node : zone_node["vertices"])
        {
          vertex_indices.push_back(v_node.as<std::size_t>());
        }
        RCLCPP_INFO(
          get_logger(),
          "Zone [%s] vertices: [%s]",
          zone_id.c_str(),
          [&vertex_indices]() {
            std::string result;
            for (size_t i = 0; i < vertex_indices.size(); ++i)
            {
              if (i > 0) result += ", ";
              result += std::to_string(vertex_indices[i]);
            }
            return result;
          }().c_str());
      }
    }
  }

  void publish_zones()
  {
    // Reload zone data dynamically to handle zone changes in YAML
    // This ensures we always use the latest zone definition
    load_zone_data();
    
    // Always publish waypoints immediately - using direct YAML vertex positions
    // No need to wait for navgraph alignment since we use building.yaml directly
    should_publish_zones_ = true;
    
    // CRITICAL: Clear ALL previous markers EVERY TIME to avoid drawing extra lines from origin
    // RViz might cache old markers, so we need to aggressively clear them
    visualization_msgs::msg::MarkerArray clear_array;
    
    // Delete all zone_outline markers with different IDs (0-50 to support multiple zones)
    for (int i = 0; i < 50; ++i)
    {
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.header.frame_id = frame_id_;
      clear_marker.header.stamp = now();
      clear_marker.action = visualization_msgs::msg::Marker::DELETE;
      clear_marker.ns = "zone_outline";
      clear_marker.id = i;
      clear_array.markers.push_back(clear_marker);
    }
    
    // Also send DELETEALL to be safe - this clears ALL markers in the namespace
    visualization_msgs::msg::Marker deleteall_marker;
    deleteall_marker.header.frame_id = frame_id_;
    deleteall_marker.header.stamp = now();
    deleteall_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    deleteall_marker.ns = "zone_outline";
    clear_array.markers.push_back(deleteall_marker);
    
    // Clear labels too
    deleteall_marker.ns = "zone_label";
    clear_array.markers.push_back(deleteall_marker);
    
    // Clear waypoint dots too
    deleteall_marker.ns = "zone_waypoints";
    clear_array.markers.push_back(deleteall_marker);
    
    // Publish clear markers first
    marker_pub_->publish(clear_array);
    
    // Small delay to ensure markers are cleared before publishing new ones
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    YAML::Node root;
    try
    {
      root = YAML::LoadFile(building_map_path_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to load building map [%s]: %s",
        building_map_path_.c_str(),
        e.what());
      return;
    }

    const auto levels = root["levels"];
    if (!levels || !levels.IsMap())
    {
      RCLCPP_ERROR(get_logger(), "No 'levels' section in building map");
      return;
    }

    const auto level = levels[level_name_];
    if (!level || !level.IsMap())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Level [%s] not found in building map",
        level_name_.c_str());
      return;
    }

    // Load vertices
    std::vector<Vertex> vertices;
    std::map<std::string, std::size_t> name_to_index; // map vertex name -> index
    const auto vertices_node = level["vertices"];
    if (vertices_node && vertices_node.IsSequence())
    {
      vertices.reserve(vertices_node.size());
      std::size_t v_idx = 0;
      for (const auto& v : vertices_node)
      {
        Vertex vert{};
        if (v.IsSequence() && v.size() >= 3)
        {
          vert.x = v[0].as<double>();
          vert.y = v[1].as<double>();
          vert.z = v[2].as<double>();

          if (v.size() > 3)
          {
            const auto& name_val = v[3];
            if (name_val.IsScalar())
            {
              vert.name = name_val.as<std::string>();
              name_to_index[vert.name] = v_idx;
            }
            else if (name_val.IsMap() && name_val["name"])
            {
              vert.name = name_val["name"].as<std::string>();
              name_to_index[vert.name] = v_idx;
            }
          }
        }
        vertices.push_back(vert);
        ++v_idx;
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Level [%s] has no vertices", level_name_.c_str());
      return;
    }

    const auto zones_node = level["zones"];
    if (!zones_node || !zones_node.IsSequence())
    {
      RCLCPP_ERROR(get_logger(), "Level [%s] has no zones", level_name_.c_str());
      return;
    }

    visualization_msgs::msg::MarkerArray array;
    int marker_id = 0; // Start from 0, increment for each zone

    for (const auto& zone_node : zones_node)
    {
      const std::string zone_id = zone_node["id"] ? zone_node["id"].as<std::string>() : "zone";
      
      // Note: zone_waypoint_ids_ is already loaded by load_zone_data() at the start of publish_zones()
      // No need to reload here - this avoids duplicate work

      // Determine vertex indices for this zone:
      // Priority 1: vertex_names (preferred, name-based to avoid index mismatches)
      // Fallback: vertices (index-based)
      std::vector<std::size_t> vertex_indices;
      if (zone_node["vertex_names"] && zone_node["vertex_names"].IsSequence())
      {
        for (const auto& name_node : zone_node["vertex_names"])
        {
          const std::string name = name_node.as<std::string>();
          auto it = name_to_index.find(name);
          if (it != name_to_index.end())
          {
            vertex_indices.push_back(it->second);
          }
          else
          {
            RCLCPP_WARN(
              get_logger(),
              "Zone [%s]: vertex name [%s] not found in vertices list",
              zone_id.c_str(), name.c_str());
          }
        }
        RCLCPP_INFO(
          get_logger(),
          "Zone [%s] using vertex_names (resolved to indices): %s",
          zone_id.c_str(),
          [&vertex_indices]() {
            std::string result;
            for (size_t i = 0; i < vertex_indices.size(); ++i)
            {
              if (i > 0) result += ", ";
              result += std::to_string(vertex_indices[i]);
            }
            return result;
          }().c_str());
      }
      else if (zone_node["vertices"] && zone_node["vertices"].IsSequence())
      {
        for (const auto& idx_node : zone_node["vertices"])
          vertex_indices.push_back(idx_node.as<std::size_t>());
      }

      // Parse zone color from parameters.zone_color (format: [1, "#RRGGBB"])
      float zone_r = 0.3f, zone_g = 0.5f, zone_b = 1.0f, zone_a = 1.0f; // Default blue
      if (zone_node["parameters"] && zone_node["parameters"].IsMap())
      {
        const auto& params = zone_node["parameters"];
        if (params["zone_color"] && params["zone_color"].IsSequence() && params["zone_color"].size() >= 2)
        {
          const std::string color_hex = params["zone_color"][1].as<std::string>();
          if (color_hex.size() == 7 && color_hex[0] == '#')
          {
            // Parse hex color #RRGGBB
            unsigned int r, g, b;
            if (sscanf(color_hex.c_str(), "#%02x%02x%02x", &r, &g, &b) == 3)
            {
              zone_r = r / 255.0f;
              zone_g = g / 255.0f;
              zone_b = b / 255.0f;
              RCLCPP_INFO(
                get_logger(),
                "Zone [%s] color parsed from hex %s: RGB=(%.2f, %.2f, %.2f)",
                zone_id.c_str(), color_hex.c_str(), zone_r, zone_g, zone_b);
            }
          }
        }
      }

      // Place colored dots at zone waypoints (using zone color)
      // Read waypoint indices from zone's waypoints list
      std::vector<std::size_t> waypoint_indices;
      if (zone_node["waypoints"] && zone_node["waypoints"].IsSequence())
      {
        for (const auto& wp_node : zone_node["waypoints"])
        {
          waypoint_indices.push_back(wp_node.as<std::size_t>());
        }
        RCLCPP_INFO(
          get_logger(),
          "Zone [%s] loaded %zu waypoint indices from YAML",
          zone_id.c_str(), waypoint_indices.size());
      }
      
      if (!waypoint_indices.empty())
      {
        // Create SPHERE_LIST marker for waypoint dots
        visualization_msgs::msg::Marker waypoint_dots;
        waypoint_dots.header.frame_id = frame_id_;
        waypoint_dots.header.stamp = now();
        waypoint_dots.ns = "zone_waypoints";
        waypoint_dots.id = marker_id++;
        waypoint_dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        waypoint_dots.action = visualization_msgs::msg::Marker::ADD; // Use ADD to ensure visibility
        waypoint_dots.scale.x = waypoint_scale_; // Dot size
        waypoint_dots.scale.y = waypoint_scale_;
        waypoint_dots.scale.z = waypoint_scale_;
        // Use zone color for waypoint dots
        waypoint_dots.color.r = zone_r;
        waypoint_dots.color.g = zone_g;
        waypoint_dots.color.b = zone_b;
        waypoint_dots.color.a = zone_a;
        waypoint_dots.pose.orientation.w = 1.0;
        waypoint_dots.lifetime = rclcpp::Duration(0, 0); // Never expire
        
        // Use navgraph positions (correct map frame) when available, fallback to YAML
        int dots_added = 0;
        int skipped_out_of_bounds = 0;
        int from_navgraph = 0;
        int from_yaml = 0;
        
        RCLCPP_INFO(
          get_logger(),
          "Zone [%s] processing %zu waypoints, vertices.size()=%zu, navgraph_waypoints=%zu",
          zone_id.c_str(), waypoint_indices.size(), vertices.size(), waypoint_positions_.size());
        
        for (const auto& wp_idx : waypoint_indices)
        {
          // Step 1: Verify waypoint index exists in YAML vertices
          if (wp_idx >= vertices.size())
          {
            skipped_out_of_bounds++;
            RCLCPP_ERROR(
              get_logger(),
              "Zone [%s] waypoint index %zu is OUT OF BOUNDS (vertices size: %zu) - SKIPPING",
              zone_id.c_str(), wp_idx, vertices.size());
            continue;
          }
          
          // Step 2: Get vertex at this index from building.yaml (for name/verification)
          const auto& vertex = vertices[wp_idx];
          const std::string& waypoint_name = vertex.name;
          
          // Step 3: Get position - MATCH BY NAME to avoid index mismatches!
          // Priority order:
          // 1. nav_graph_positions_by_name_ (from nav_graphs/0.yaml by NAME - most reliable!)
          // 2. waypoint_positions_ (from /map_markers topic by index - fallback)
          // 3. building.yaml vertex (last resort)
          geometry_msgs::msg::Point p;
          if (!waypoint_name.empty() && nav_graph_positions_by_name_.find(waypoint_name) != nav_graph_positions_by_name_.end())
          {
            // BEST: Use position from nav_graphs/0.yaml matched by NAME (correct coordinate frame!)
            p = nav_graph_positions_by_name_[waypoint_name];
            p.z = p.z + 0.1;
            from_navgraph++;
          }
          else if (waypoint_positions_.find(wp_idx) != waypoint_positions_.end())
          {
            // FALLBACK: Use position from /map_markers by index (may have index mismatch)
            p = waypoint_positions_[wp_idx];
            p.z = p.z + 0.1;
            from_navgraph++;
            if (!waypoint_name.empty())
            {
              RCLCPP_DEBUG(
                get_logger(),
                "Zone [%s] waypoint %zu (%s) using /map_markers by index (name not found in nav_graph)",
                zone_id.c_str(), wp_idx, waypoint_name.c_str());
            }
          }
          else
          {
            // LAST RESORT: Use building.yaml position (may be wrong coordinate frame)
            p.x = vertex.x;
            p.y = vertex.y;
            p.z = vertex.z + 0.1;
            from_yaml++;
            RCLCPP_WARN(
              get_logger(),
              "Zone [%s] waypoint %zu (%s) NOT found by name in nav_graph or by index in /map_markers, using YAML pos (%.2f, %.2f)",
              zone_id.c_str(), wp_idx, waypoint_name.c_str(), p.x, p.y);
          }
          
          // Step 4: Add waypoint dot at this position
          waypoint_dots.points.push_back(p);
          dots_added++;
          
          // Log every waypoint for debugging (first 10, then every 10th)
          if (dots_added <= 10 || dots_added % 10 == 0)
          {
            RCLCPP_INFO(
              get_logger(),
              "Zone [%s] waypoint[%d/%zu] index=%zu name=%s pos=(%.2f, %.2f, %.2f) [%s]",
              zone_id.c_str(), dots_added, waypoint_indices.size(), wp_idx,
              waypoint_name.c_str(), p.x, p.y, p.z,
              (!waypoint_name.empty() && nav_graph_positions_by_name_.find(waypoint_name) != nav_graph_positions_by_name_.end()) ? "navgraph_by_name" : 
              (waypoint_positions_.find(wp_idx) != waypoint_positions_.end() ? "navgraph_markers" : "YAML"));
          }
        }
        
        RCLCPP_INFO(
          get_logger(),
          "Zone [%s] COMPLETE: %d waypoint dots added (navgraph: %d, navgraph_markers: %d, YAML: %d), %d skipped out of bounds",
          zone_id.c_str(), dots_added, from_navgraph, 
          (dots_added - from_navgraph - from_yaml), from_yaml, skipped_out_of_bounds);
        
        if (dots_added > 0)
        {
          array.markers.push_back(waypoint_dots);
          RCLCPP_INFO(
            get_logger(),
            "Zone [%s] ✅ Published %d waypoint dots (expected %zu) - scale=%.2f, color=(%.2f,%.2f,%.2f,%.2f)",
            zone_id.c_str(), dots_added, waypoint_indices.size(),
            waypoint_scale_, waypoint_dots.color.r, waypoint_dots.color.g, 
            waypoint_dots.color.b, waypoint_dots.color.a);
          
          if (dots_added != static_cast<int>(waypoint_indices.size()))
          {
            RCLCPP_WARN(
              get_logger(),
              "Zone [%s] ⚠️  Mismatch: expected %zu waypoints, but only %d were added!",
              zone_id.c_str(), waypoint_indices.size(), dots_added);
          }
        }
        else
        {
          RCLCPP_ERROR(
            get_logger(),
            "Zone [%s] ❌ NO waypoint dots added! Check waypoint indices and nav_graph loading.",
            zone_id.c_str());
        }
      }

      // Waypoint dots removed - user only wants the blue outline line

      // Text label
      visualization_msgs::msg::Marker label;
      label.header.frame_id = frame_id_;
      label.header.stamp = now();
      label.ns = "zone_label";
      label.id = marker_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.scale.z = 0.2; // Smaller text to match navgraph labels
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 0.9f;
      label.pose.orientation.w = 1.0;
      label.text = zone_id;

      // Place label at centroid of the resolved vertex set
      geometry_msgs::msg::Point centroid;
      {
        double cx = 0.0;
        double cy = 0.0;
        double cz = 0.0;
        std::size_t count = 0;
        for (const auto& idx : vertex_indices)
        {
          if (idx >= vertices.size())
            continue;
          cx += vertices[idx].x;
          cy += vertices[idx].y;
          cz += vertices[idx].z;
          ++count;
        }
        if (count > 0)
        {
          // Apply scaling first, then offset
          centroid.x = ((cx / count) * scale_x_) + offset_x_;
          centroid.y = ((cy / count) * scale_y_) + offset_y_;
          centroid.z = (cz / count) + offset_z_ + 0.1;
        }
      }
      label.pose.position = centroid;
      array.markers.push_back(label);
    }

    if (array.markers.empty())
    {
      RCLCPP_WARN(
        get_logger(),
        "No zone markers to publish for level [%s]",
        level_name_.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Publishing %zu zone markers for level [%s] in frame [%s]",
      array.markers.size(),
      level_name_.c_str(),
      frame_id_.c_str());

    // Log marker details for debugging
    for (const auto& marker : array.markers)
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Marker: ns=[%s], id=%d, type=%d, points=%zu",
        marker.ns.c_str(),
        marker.id,
        marker.type,
        marker.points.size());
    }

    marker_pub_->publish(array);
  }

  std::string building_map_path_;
  std::string nav_graph_path_;
  std::string level_name_;
  std::string frame_id_;
  
  // Waypoint positions from nav_graphs/0.yaml (name -> position)
  // Match by name to avoid index mismatches
  std::map<std::string, geometry_msgs::msg::Point> nav_graph_positions_by_name_;
  double line_width_;
  double marker_alpha_;
  double waypoint_scale_;
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double scale_x_;
  double scale_y_;
  bool prefer_yaml_vertices_;
  bool any_zone_has_vertex_names_{false};
  
  // Store zone waypoint IDs and their actual positions from navgraph
  std::map<std::string, std::set<std::size_t>> zone_waypoint_ids_; // zone_id -> waypoint indices
  std::map<std::size_t, geometry_msgs::msg::Point> waypoint_positions_; // waypoint index -> actual position
  bool zone_bounds_calculated_;
  double zone_actual_min_x_, zone_actual_max_x_;
  double zone_actual_min_y_, zone_actual_max_y_;
  bool should_publish_zones_; // Only publish after navgraph alignment is done

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr floorplan_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr navgraph_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void load_nav_graph_waypoints()
  {
    // If nav_graph_path not provided, try to auto-detect from building_map_path
    if (nav_graph_path_.empty())
    {
      // Try multiple locations:
      // 1. Relative to building.yaml: maps/four_d/nav_graphs/0.yaml
      std::string building_dir = building_map_path_;
      size_t last_slash = building_dir.find_last_of("/");
      if (last_slash != std::string::npos)
      {
        building_dir = building_dir.substr(0, last_slash);
        nav_graph_path_ = building_dir + "/nav_graphs/0.yaml";
        
        // If not found, try install directory
        if (access(nav_graph_path_.c_str(), F_OK) != 0)
        {
          // Try install path: install/rmf_demos_maps/share/rmf_demos_maps/maps/four_d/nav_graphs/0.yaml
          const char* colcon_prefix = std::getenv("COLCON_PREFIX_PATH");
          if (colcon_prefix)
          {
            std::string install_path = colcon_prefix;
            size_t colon = install_path.find(':');
            if (colon != std::string::npos)
              install_path = install_path.substr(0, colon);
            
            // Extract map name from building_map_path (e.g., "four_d")
            std::string map_name = "four_d"; // default
            size_t four_d_pos = building_map_path_.find("/four_d/");
            if (four_d_pos != std::string::npos)
            {
              size_t maps_pos = building_map_path_.find("/maps/");
              if (maps_pos != std::string::npos)
              {
                size_t map_start = maps_pos + 6; // after "/maps/"
                size_t map_end = building_map_path_.find("/", map_start);
                if (map_end != std::string::npos)
                  map_name = building_map_path_.substr(map_start, map_end - map_start);
              }
            }
            
            nav_graph_path_ = install_path + "/rmf_demos_maps/share/rmf_demos_maps/maps/" + map_name + "/nav_graphs/0.yaml";
          }
        }
      }
    }
    
    if (nav_graph_path_.empty())
    {
      RCLCPP_WARN(get_logger(), "nav_graph_path not set - will use navgraph markers or YAML positions");
      return;
    }
    
    try
    {
      YAML::Node root = YAML::LoadFile(nav_graph_path_);
      const auto levels = root["levels"];
      if (!levels || !levels.IsMap())
      {
        RCLCPP_WARN(get_logger(), "No 'levels' section in nav_graph file");
        return;
      }
      
      const auto level = levels[level_name_];
      if (!level || !level.IsMap())
      {
        RCLCPP_WARN(get_logger(), "Level [%s] not found in nav_graph", level_name_.c_str());
        return;
      }
      
      const auto nav_vertices = level["vertices"];
      if (!nav_vertices || !nav_vertices.IsSequence())
      {
        RCLCPP_WARN(get_logger(), "No 'vertices' section in nav_graph");
        return;
      }
      
      // Parse vertices: each is [x, y, {name: "..."}]
      // Store by NAME to match waypoints correctly (avoids index mismatches)
      for (std::size_t i = 0; i < nav_vertices.size(); ++i)
      {
        const auto& v = nav_vertices[i];
        if (!v.IsSequence() || v.size() < 2)
          continue;
        
        double x = v[0].as<double>();
        double y = v[1].as<double>();
        std::string name;
        
        // Get name from third element if it's a map
        if (v.size() >= 3 && v[2].IsMap() && v[2]["name"])
        {
          name = v[2]["name"].as<std::string>();
        }
        
        if (!name.empty())
        {
          geometry_msgs::msg::Point p;
          p.x = x;
          p.y = y;
          p.z = 0.0;
          nav_graph_positions_by_name_[name] = p; // Store by NAME
        }
      }
      
      RCLCPP_INFO(
        get_logger(),
        "Loaded %zu waypoint positions from nav_graph [%s] (matched by name)",
        nav_graph_positions_by_name_.size(), nav_graph_path_.c_str());
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to load nav_graph waypoints from [%s]: %s",
        nav_graph_path_.c_str(), e.what());
    }
  }

  void auto_detect_map_offset()
  {
    // Try to read image offsets from building map YAML
    try
    {
      const YAML::Node root = YAML::LoadFile(building_map_path_);
      const auto levels = root["levels"];
      if (levels && levels.IsMap())
      {
        const auto level = levels[level_name_];
        if (level && level.IsMap() && level["images"] && level["images"].IsSequence() && level["images"].size() > 0)
        {
          const auto& image = level["images"][0];
          if (image["x_offset"] && image["y_offset"])
          {
            double x_off = image["x_offset"].as<double>();
            double y_off = image["y_offset"].as<double>();
            
            // Apply negative offset to align zone with map
            offset_x_ = -x_off;
            offset_y_ = -y_off;
            
            RCLCPP_INFO(
              get_logger(),
              "Auto-detected map offset from building map: x_offset=%.2f -> offset_x=%.2f, y_offset=%.2f -> offset_y=%.2f",
              x_off, offset_x_, y_off, offset_y_);
          }
        }
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(
        get_logger(),
        "Could not auto-detect map offset from building map: %s. Using manual offsets if provided.",
        e.what());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ZoneVisualizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

