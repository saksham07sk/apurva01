/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

// Include the main Supergraph header file for class definitions and declarations
#include "Supergraph.hpp"

// Include math utilities for mathematical operations like atan2, rotation calculations
#include <rmf_utils/math.hpp>

// Include unordered_set for efficient set operations used in floor change detection
#include <unordered_set>

// Conditional debug include for Supergraph debugging - only included if debug flag is set
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__SUPERGRAPH
#include <iostream>
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__SUPERGRAPH

// Additional iostream include for general debugging output
#include <iostream>

// Begin the main namespace hierarchy for RMF traffic AGV planning
namespace rmf_traffic {
namespace agv {
namespace planning {

// Anonymous namespace for internal helper functions and structures
namespace {
//==============================================================================
// Function to find all floor changes (map transitions) in the navigation graph
// This analyzes the original graph to identify lanes that connect different maps/floors
Supergraph::FloorChangeMap find_floor_changes(
  const Graph::Implementation& original)
{
  // Initialize the map to store floor changes organized by starting map name
  Supergraph::FloorChangeMap all_floor_changes;

  // TODO(MXG): Calculate this in the regular Graph structure while lanes get
  // added to it.
  // Iterate through all waypoints to find potential floor changes
  for (std::size_t i = 0; i < original.waypoints.size(); ++i)
  {
    // Get the map name of the current waypoint (starting point)
    const auto& initial_map_name = original.waypoints[i].get_map_name();
    // Get or create the floor changes map for this starting map
    auto& floor_changes = all_floor_changes[initial_map_name];

    // Check all lanes that start from this waypoint
    for (const auto l : original.lanes_from[i])
    {
      // Get the lane information
      const auto& lane = original.lanes[l];
      // Get the exit waypoint of this lane
      const auto& exit = original.waypoints[lane.exit().waypoint_index()];
      // Get the map name of the exit waypoint (destination)
      const auto& final_map_name = exit.get_map_name();
      // If the starting and ending maps are different, this is a floor change
      if (initial_map_name != final_map_name)
        // Add this lane as a floor change to the destination map
        floor_changes[final_map_name].push_back(Supergraph::FloorChange{l});
    }
  }

  // Return the complete map of all floor changes
  return all_floor_changes;
}

//==============================================================================
// Function to compute rotation offset from a forward direction vector
// Converts a 2D direction vector into a rotation matrix using atan2
Eigen::Rotation2Dd compute_forward_offset(
  const Eigen::Vector2d& forward)
{
  // Create a 2D rotation matrix from the angle computed using atan2
  // atan2(y, x) gives the angle from positive x-axis to the vector
  return Eigen::Rotation2Dd(std::atan2(forward[1], forward[0]));
}

//==============================================================================
// Structure representing a traversal node in the navigation graph
// Contains all information needed to represent a path segment between waypoints
struct TraversalNode
{
  // Index of the initial lane in the graph's lane array
  std::size_t initial_lane_index;
  // Index of the final lane in the graph's lane array
  std::size_t finish_lane_index;
  // Index of the starting waypoint in the graph's waypoint array
  std::size_t initial_waypoint_index;
  // Index of the ending waypoint in the graph's waypoint array
  std::size_t finish_waypoint_index;

  // 2D position vector of the starting point
  Eigen::Vector2d initial_p;
  // 2D position vector of the ending point
  Eigen::Vector2d finish_p;

  // Optional event that occurs when entering this traversal (e.g., door opening)
  Graph::Lane::EventPtr entry_event;
  // Optional event that occurs when exiting this traversal (e.g., door closing)
  Graph::Lane::EventPtr exit_event;

  // Mutex group identifier for coordinating access to shared resources
  std::string mutex_group;

  // TODO(MXG): Replace this with a more intelligent way of handling speed limit
  // changes that may occur when traversing multiple consecutive lanes
  // Optional lowest speed limit encountered during traversal
  std::optional<double> lowest_speed_limit;

  // TODO(MXG): Can std::string_view be used to make this more memory efficient?
  // List of map names that this traversal passes through
  std::vector<std::string> map_names;
  // List of lane indices that are traversed in sequence
  std::vector<std::size_t> traversed_lanes;

  // Array of optional orientation values (up to 2 orientations supported)
  std::array<std::optional<double>, 2> orientations;
  // Flag indicating if this is a standstill traversal (no movement)
  bool standstill = false;
};

//==============================================================================
// Function to validate if a traversal node represents a valid path
// A traversal is valid if it's a standstill or has at least one valid orientation
bool valid_traversal(const TraversalNode& node)
{
  // Standstill traversals are always valid (no movement required)
  if (node.standstill)
    return true;

  // Check if any of the orientation values are set (not nullopt)
  for (const auto orientation : node.orientations)
  {
    // If at least one orientation is valid, the traversal is valid
    if (orientation.has_value())
      return true;
  }

  // If no orientations are valid and it's not a standstill, traversal is invalid
  return false;
}

//==============================================================================
// Converts a TraversalNode into one or more Traversal objects for path planning
// This function takes a node representing a path segment and creates the actual
// traversal data structures that can be used by the motion planner
void node_to_traversals(
  const TraversalNode& node,  // Input node containing path segment information
  const TraversalFromGenerator::Kinematics& kin,  // Kinematic constraints and limits
  std::vector<Traversal>& output)  // Output vector to store generated traversals
{

  // Ensure the input node represents a valid traversal path
  assert(valid_traversal(node));
  
  // Create a new traversal object to populate with node data
  Traversal traversal;
  
  // Copy basic lane and waypoint indices from the node
  traversal.initial_lane_index = node.initial_lane_index;  // Starting lane in the graph
  traversal.finish_lane_index = node.finish_lane_index;    // Ending lane in the graph
  traversal.initial_waypoint_index = node.initial_waypoint_index;  // Starting waypoint
  traversal.finish_waypoint_index = node.finish_waypoint_index;    // Ending waypoint
  
  // Initialize cost to zero (will be accumulated as we process events and motion)
  traversal.best_cost = 0.0;
  
  // Copy map names that this traversal passes through
  traversal.maps = std::vector<std::string>(
    node.map_names.begin(), node.map_names.end());
  
  // Copy the sequence of lane indices that are traversed
  traversal.traversed_lanes = node.traversed_lanes;

//  std::cout << "Traversal [" << traversal.initial_lane_index << "] -> ("
//            << traversal.finish_lane_index << "): entry event {"
//            << traversal.entry_event << "}" << std::endl;
  
  // Process entry event if present (e.g., door opening, lift activation)
  if (node.entry_event)
  {
    // Clone the entry event to avoid shared ownership issues
    traversal.entry_event = node.entry_event->clone();
    // Add the event duration to the total traversal cost
    traversal.best_cost += rmf_traffic::time::to_seconds(
      traversal.entry_event->duration());
  }

  // Process exit event if present (e.g., door closing, lift deactivation)
  if (node.exit_event)
  {
    // Clone the exit event to avoid shared ownership issues
    traversal.exit_event = node.exit_event->clone();
    // Add the event duration to the total traversal cost
    traversal.best_cost += rmf_traffic::time::to_seconds(
      traversal.exit_event->duration());
  }

  // Handle standstill traversals (no movement, just waiting at a waypoint)
  if (node.standstill)
  {
    // Create an alternative for standstill motion
    Traversal::Alternative alt;
    // Generate a route factory for staying at the initial position
    alt.routes = make_start_factory(
      node.initial_p, std::nullopt, kin.limits,
      kin.interpolate.rotation_thresh, traversal.maps);

    // Store the standstill alternative in the "Any" orientation slot
    traversal.alternatives[static_cast<std::size_t>(Orientation::Any)] =
      std::move(alt);

    // Verify that standstill nodes don't have orientation requirements
    // This is a safety check to ensure logical consistency
    assert(
      !node.orientations[static_cast<std::size_t>(Orientation::Forward)]
      .has_value() &&
      !node.orientations[static_cast<std::size_t>(Orientation::Backward)]
      .has_value()
    );
  }

  // Variables to track the best trajectory time and distance for cost calculation
  std::optional<double> best_trajectory_time;  // Fastest time among all orientations
  std::optional<double> traversal_distance;    // Euclidean distance of the path
  
  // Process each possible orientation (Forward and Backward)
  for (std::size_t i = 0; i < 2; ++i)
  {
    // Get the yaw angle for this orientation
    const auto yaw = node.orientations[i];

    // Skip if this orientation is not valid for this traversal
    if (!yaw.has_value())
      continue;

    // Create 3D start position (x, y, yaw)
    const Eigen::Vector3d start{
      node.initial_p.x(),  // X coordinate of starting point
      node.initial_p.y(),  // Y coordinate of starting point
      * yaw                // Yaw angle for this orientation
    };

    // Create 3D finish position (x, y, yaw)
    const Eigen::Vector3d finish{
      node.finish_p.x(),   // X coordinate of ending point
      node.finish_p.y(),   // Y coordinate of ending point
      * yaw                // Same yaw angle (no rotation during traversal)
    };

    // Calculate Euclidean distance if not already computed
    if (!traversal_distance.has_value())
    {
      // Compute 2D distance between start and finish points
      traversal_distance =
        (finish.block<2, 1>(0, 0) - start.block<2, 1>(0, 0)).norm();
    }

    // Apply speed limits if specified in the node
    auto kin_limits = kin.limits;  // Copy kinematic limits
    if (node.lowest_speed_limit.has_value())
    {
      // Use the more restrictive speed limit between kinematic limits and node limits
      kin_limits.linear.velocity =
        std::min(kin_limits.linear.velocity, *node.lowest_speed_limit);
    }

    // Create an alternative trajectory for this orientation
    Traversal::Alternative alternative;
    alternative.yaw = *yaw;  // Store the yaw angle for this alternative
    
    // Generate a differential drive trajectory factory for this path segment
    auto factory_info = make_differential_drive_translate_factory(
      start, finish, kin_limits,                    // Start/end points and limits
      kin.interpolate.translation_thresh,          // Translation interpolation threshold
      kin.interpolate.rotation_thresh,             // Rotation interpolation threshold
      kin.traversal_cost_per_meter,                // Cost per meter of travel
      traversal.maps);                             // Maps this traversal covers

    // Extract the minimum cost (time) for this trajectory
    const auto time = factory_info.minimum_cost;
    alternative.cost = time;  // Store the cost for this alternative
    alternative.routes = std::move(factory_info.factory);  // Move the route factory

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__SUPERGRAPH
    // Debug output showing traversal details and timing
    std::cout << "SUPERGRAPH [" << traversal.initial_lane_index
              << Orientation(i) << Side::Start << "] ("
              << traversal.finish_waypoint_index << "): " << time << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__SUPERGRAPH

    // Track the best (fastest) trajectory time across all orientations
    if (!best_trajectory_time.has_value() || time < *best_trajectory_time)
      best_trajectory_time = time;

    // Store this alternative in the traversal
    traversal.alternatives[i] = std::move(alternative);
  }

  // Add motion costs to the total traversal cost
  if (best_trajectory_time.has_value() && traversal_distance.has_value())
  {
    // Total cost = best trajectory time + distance-based cost
    traversal.best_cost +=
      *best_trajectory_time  // Time cost for the fastest orientation
      + kin.traversal_cost_per_meter * (*traversal_distance);  // Distance-based cost
  }

  // Add the completed traversal to the output vector
  output.emplace_back(std::move(traversal));
}

//==============================================================================
// Helper function to add a map name to a vector if it's not already present
// This prevents duplicate map names in the traversal path
void add_if_missing(
  std::vector<std::string>& all_maps,  // Vector of map names to add to
  const std::string& map)              // Map name to potentially add
{
  // Check if the map name already exists in the vector
  if (std::find(all_maps.begin(), all_maps.end(), map) == all_maps.end())
    all_maps.push_back(map);  // Add the map name if it's not found
}

//==============================================================================
// Main function to perform traversal from one waypoint to another through a lane
// This handles the core logic of expanding the graph by traversing lanes
void perform_traversal(
  const TraversalNode* parent,                    // Parent node in the traversal tree (nullptr for root)
  const std::size_t lane_index,                   // Index of the lane to traverse
  const Graph::Implementation& graph,             // The navigation graph structure
  const LaneClosure& closures,                    // Information about which lanes are closed
  const TraversalFromGenerator::Kinematics& kin,  // Kinematic constraints and parameters
  std::vector<TraversalNode>& queue,              // Queue of nodes to be processed
  std::vector<Traversal>& output,                 // Output vector of valid traversals
  std::unordered_set<std::size_t>& visited)      // Set of already visited waypoints
{
  // Check if the lane is closed - if so, we cannot traverse it
  if (closures.is_closed(lane_index))
  {
    // If the lane is closed, then we must not traverse it.
    return;
  }

  // Get references to the lane and its entry/exit points
  const auto& lane = graph.lanes[lane_index];     // The lane we're trying to traverse
  const auto& entry = lane.entry();               // Entry waypoint of the lane
  const auto& exit = lane.exit();                 // Exit waypoint of the lane
  const std::size_t wp_index_0 = entry.waypoint_index();  // Index of start waypoint
  const std::size_t wp_index_1 = exit.waypoint_index();   // Index of end waypoint

  // Validate the speed limit for this lane
  if (lane.properties().speed_limit().has_value())
  {
    const auto speed_limit = *lane.properties().speed_limit();  // Get the speed limit value
    if (speed_limit <= 0.0)  // Check for invalid speed limits
    {
      // If the lane has a nonsense speed limit, then we will warn the user and
      // avoid traversing it.
      std::cerr << "A speed limit of " << speed_limit
                << " was given for lane " << lane_index
                << ". Speed limits must be strictly greater than 0.0 to "
                << "prevent mathematical singularities or illegal time travel. "
                << "The planner will treat this lane as though it is blocked, "
                << "but you are advised to use the lane closure feature for "
                << "that instead." << std::endl;
      return;  // Exit early due to invalid speed limit
    }
  }

  // Check if we've already visited the destination waypoint
  if (!visited.insert(wp_index_1).second)
  {
    // If we have already added the finish waypoint to the queue, then there is
    // no need to add it again.
    return;  // Exit early to avoid duplicate processing
  }

  // Get waypoint information and calculate positions
  const auto& wp0 = graph.waypoints[wp_index_0];  // Start waypoint object
  const auto& wp1 = graph.waypoints[wp_index_1];  // End waypoint object
  const Eigen::Vector2d p0 = wp0.get_location();  // Start position (x, y)
  const Eigen::Vector2d p1 = wp1.get_location();  // End position (x, y)

  // Create a new traversal node for this lane
  TraversalNode node;
  node.finish_lane_index = lane_index;                    // Set the lane we're finishing on
  node.initial_waypoint_index = wp_index_0;               // Set the start waypoint index
  node.finish_waypoint_index = wp_index_1;                // Set the end waypoint index
  node.lowest_speed_limit = lane.properties().speed_limit();  // Copy speed limit from lane
  node.mutex_group = lane.properties().in_mutex_group();      // Copy mutex group from lane

  // Handle parent node inheritance and validation
  if (parent)  // If this is not the root node
  {
    // Check for entry events that would break the traversal chain
    if (entry.event())
    {
      // If this lane has an entry event, then we cannot continue the traversal.
      // A new traversal will have to begin from this waypoint.
      return;  // Exit early - entry events break continuous traversal
    }

    // Handle mutex group transitions
    const auto& wp0_mutex = wp0.in_mutex_group();  // Get mutex group of start waypoint
    if (!parent->mutex_group.empty() && wp0_mutex.empty())
    {
      // We are exiting a mutex group, so we should do a quick stop here to make
      // sure that we release the mutex.
      return;  // Exit early - need to stop when exiting mutex groups
    }

    // Check for mutex group conflicts between parent and current lane
    if (!node.mutex_group.empty() && parent->mutex_group != node.mutex_group)
    {
      // The lane belongs to a different mutex group than the parent, so we
      // cannot continuously traverse.
      return;  // Exit early - mutex group mismatch
    }

    // Check for mutex group conflicts at the end waypoint
    const auto& wp1_mutex = wp1.in_mutex_group();  // Get mutex group of end waypoint
    if (!wp1_mutex.empty() && wp1_mutex != node.mutex_group)
    {
      // The end waypoint of this lane belongs to a different mutex group than
      // the lane leading up to it. The waypoint's mutex must be locked before
      // traversing this lane, so we should stop before traversing this lane
      // instead of continuing from a parent.
      return;  // Exit early - waypoint mutex conflict
    }

    // Inherit properties from parent node
    node.initial_lane_index = parent->initial_lane_index;      // Copy initial lane from parent
    node.initial_waypoint_index = parent->initial_waypoint_index;  // Copy initial waypoint from parent
    node.initial_p = parent->initial_p;                        // Copy initial position from parent
    node.map_names = parent->map_names;                        // Copy map names from parent
    node.traversed_lanes = parent->traversed_lanes;            // Copy traversed lanes from parent

    // Handle speed limit inheritance (take the more restrictive limit)
    if (parent->lowest_speed_limit.has_value())
    {
      if (node.lowest_speed_limit.has_value())
      {
        // Take the minimum of parent and current speed limits
        node.lowest_speed_limit =
          std::min(*node.lowest_speed_limit, *parent->lowest_speed_limit);
      }
      else
      {
        // Use parent's speed limit if current lane doesn't have one
        node.lowest_speed_limit = parent->lowest_speed_limit;
      }
    }

    // Clone entry event from parent if it exists
    if (parent->entry_event)
      node.entry_event = parent->entry_event->clone();
  }
  else  // This is a root node (no parent)
  {
    // Initialize as a root traversal node
    node.initial_lane_index = lane_index;  // Set initial lane to current lane
    node.initial_p = p0;                   // Set initial position to start position

    // Clone entry event from lane if it exists
    if (const auto* entry_event = entry.event())
      node.entry_event = entry_event->clone();
  }

  // Update traversal information
  node.traversed_lanes.push_back(lane_index);  // Add current lane to traversed lanes list
  node.finish_p = p1;                          // Set finish position to end position

  // Add map names for both waypoints to the node's map list
  add_if_missing(node.map_names, wp0.get_map_name());  // Add start waypoint's map
  add_if_missing(node.map_names, wp1.get_map_name());  // Add end waypoint's map

  // Calculate distance between waypoints and handle orientation logic
  const double dist = (p1 - p0).norm();  // Euclidean distance between waypoints
  if (!kin.constraint.has_value() || dist < kin.interpolate.translation_thresh)
  {
    // Handle case where waypoints are very close together (standstill)
    if (!parent)  // This is a root node
    {
      // These waypoints are effectively on top of each other, and we haven't
      // moved anywhere to arrive here. We will call this a standstill.
      node.standstill = true;
    }
    else
    {
      // These waypoints are effectively on top of each other. We will carry
      // over the orientations of the parent.
      node.standstill = parent->standstill;
      node.orientations = parent->orientations;
    }
  }
  else
  {
    const Eigen::Vector2d course_vector = (p1 - p0)/dist;
    const auto orientations =
      kin.constraint->get_orientations(course_vector);

    const double thresh = kin.interpolate.rotation_thresh;

    for (std::size_t i = 0; i < orientations.size(); ++i)
    {
      const auto orientation = orientations[i];
      if (!orientation.has_value())
        continue;

      const auto* entry_constraint = entry.orientation_constraint();
      if (!orientation_constraint_satisfied(
          p0, *orientation, course_vector, entry_constraint, thresh))
        continue;

      const auto* exit_constraint = exit.orientation_constraint();
      if (!orientation_constraint_satisfied(
          p1, *orientation, course_vector, exit_constraint, thresh))
        continue;

      if (parent && !parent->standstill)
      {
        const auto parent_orientation = parent->orientations[i];
        if (!parent_orientation.has_value())
          continue;

        const double R_diff = rmf_utils::wrap_to_pi(
          *orientation - *parent_orientation);

        if (std::abs(R_diff) > kin.interpolate.rotation_thresh)
          continue;
      }

      node.orientations[i] = *orientation;
    }
  }

  if (!valid_traversal(node))
  {
    // If this lane has no valid orientations and also does not stand still,
    // then it's not a real traversal, and we should not output it or queue it.
    return;
  }

  const auto* exit_event = exit.event();
  if (exit_event)
    node.exit_event = exit_event->clone();

  // Convert this node into a set of traversals
  node_to_traversals(node, kin, output);

  if (exit_event)
  {
    // If this lane has an exit event, then we need to stop the traversal here,
    // so it does not get added to the queue.
    return;
  }

//  std::cout << "Pushing for further expansion" << std::endl;
  queue.push_back(std::move(node));
}

//==============================================================================
// Wrapper function to expand traversal from a parent node
// This function is called when continuing traversal from an existing node in the search tree
void expand_traversal(
  const TraversalNode& parent,                    // Parent node that we're expanding from
  const std::size_t lane_index,                   // Index of the lane to traverse next
  const Graph::Implementation& graph,             // The navigation graph structure
  const LaneClosure& closures,                    // Information about which lanes are closed
  const TraversalFromGenerator::Kinematics& kin,  // Kinematic constraints and parameters
  std::vector<TraversalNode>& queue,              // Queue of nodes to be processed
  std::vector<Traversal>& output,                 // Output vector of valid traversals
  std::unordered_set<std::size_t>& visited)      // Set of already visited waypoints
{
  // Delegate to the main traversal function, passing the parent node pointer
  perform_traversal(
    &parent, lane_index, graph, closures, kin, queue, output, visited);
}

//==============================================================================
// Wrapper function to initiate traversal from a starting waypoint
// This function is called when starting a new traversal from a waypoint (no parent node)
void initiate_traversal(
  const std::size_t lane_index,                   // Index of the lane to start traversing
  const Graph::Implementation& graph,             // The navigation graph structure
  const LaneClosure& closures,                    // Information about which lanes are closed
  const TraversalFromGenerator::Kinematics& kin,  // Kinematic constraints and parameters
  std::vector<TraversalNode>& queue,              // Queue of nodes to be processed
  std::vector<Traversal>& output,                 // Output vector of valid traversals
  std::unordered_set<std::size_t>& visited)      // Set of already visited waypoints
{
  // Delegate to the main traversal function, passing nullptr for no parent
  perform_traversal(
    nullptr, lane_index, graph, closures, kin, queue, output, visited);
}

} // anonymous namespace

//==============================================================================
// Calculate the total cost of a trajectory including both time and distance costs
// This is used to evaluate the cost of different path options during planning
double calculate_cost(
  const rmf_traffic::Trajectory& traj,            // The trajectory to calculate cost for
  const double traversal_cost_per_meter)          // Cost per meter of travel distance
{
  // If trajectory is empty, no cost
  if (traj.empty())
    return 0.0;

  // Calculate distance-based cost by summing up distances between consecutive waypoints
  double distance_cost = 0.0;
  for (std::size_t i=1; i < traj.size(); ++i)
  {
    // Add cost for the distance between this waypoint and the previous one
    // Only consider x,y coordinates (block<2,1>(0,0)) ignoring z and orientation
    distance_cost +=
      traversal_cost_per_meter *
      (traj[i].position() - traj[i-1].position()).block<2, 1>(0, 0).norm();
  }

  // Return the total time duration of the trajectory in seconds
  // Note: distance_cost is calculated but not used in the return value
  return time::to_seconds(traj.duration());
}

//==============================================================================
// Check if a given orientation satisfies the orientation constraint at a waypoint
// This ensures the robot can actually achieve the required orientation when entering/exiting
bool orientation_constraint_satisfied(
  const Eigen::Vector2d p,                        // Position of the waypoint
  const double orientation,                       // Desired orientation angle
  const Eigen::Vector2d course_vector,           // Direction vector of the path segment
  const rmf_traffic::agv::Graph::OrientationConstraint* constraint,  // Constraint to check against
  const double rotation_thresh)                   // Threshold for acceptable rotation difference
{
  // If no constraint is specified, any orientation is valid
  if (!constraint)
    return true;

  // Create a 3D position vector with the given orientation
  Eigen::Vector3d position{p.x(), p.y(), orientation};
  
  // Apply the constraint to see what orientation is actually achievable
  constraint->apply(position, course_vector);
  
  // Calculate the difference between desired and achievable orientation
  const double diff = rmf_utils::wrap_to_pi(position[2] - orientation);
  
  // Check if the difference is within acceptable threshold
  if (std::abs(diff) > rotation_thresh)
    return false;

  return true;
}

//==============================================================================
// Static constant representing a 180-degree rotation
// Used for calculating backward orientations in differential drive vehicles
const Eigen::Rotation2Dd DifferentialDriveConstraint::R_pi =
  Eigen::Rotation2Dd(M_PI);

//==============================================================================
// Constructor for differential drive constraint
// Sets up the constraint based on the vehicle's forward direction and reversibility
DifferentialDriveConstraint::DifferentialDriveConstraint(
  const Eigen::Vector2d& forward,                 // Forward direction vector of the vehicle
  const bool reversible)                          // Whether the vehicle can drive backward
: R_f_inv(compute_forward_offset(forward).inverse()),  // Store inverse of forward rotation
  reversible(reversible)                          // Store reversibility flag
{
  // Do nothing - initialization handled by member initializer list
}

//==============================================================================
// Calculate valid orientations for a differential drive vehicle given a course direction
// Returns orientations for both forward and backward (if reversible) directions
std::array<std::optional<double>, 2>
DifferentialDriveConstraint::get_orientations(
  const Eigen::Vector2d& course_vector) const    // Direction vector of the path to follow
{
  // Array to store valid orientations [Forward, Backward]
  std::array<std::optional<double>, 2> orientations;

  // Calculate rotation matrix for the course direction
  const Eigen::Rotation2Dd R_c(
    std::atan2(course_vector[1], course_vector[0]));
  
  // Calculate the heading rotation by combining course and inverse forward rotations
  const Eigen::Rotation2Dd R_h = R_c * R_f_inv;

  // Set forward orientation (always available)
  orientations[static_cast<std::size_t>(Orientation::Forward)] =
    rmf_utils::wrap_to_pi(R_h.angle());

  // If vehicle is reversible, calculate backward orientation (180° from forward)
  if (reversible)
  {
    orientations[static_cast<std::size_t>(Orientation::Backward)] =
      rmf_utils::wrap_to_pi((R_pi * R_h).angle());
  }

  return orientations;
}

//==============================================================================
// Constructor for TraversalFromGenerator::Kinematics
// Initializes kinematic parameters and constraints for path traversal calculations
TraversalFromGenerator::Kinematics::Kinematics(
  const VehicleTraits& traits,                    // Vehicle characteristics (speed limits, dimensions, etc.)
  const Interpolate::Options::Implementation& interpolate_,  // Interpolation settings for trajectory generation
  double traversal_cost_per_meter_)               // Cost per meter of travel for path planning
: limits(VehicleTraits::Implementation::get_limits(traits)),  // Extract speed/acceleration limits from vehicle traits
  interpolate(interpolate_),                      // Store interpolation configuration
  traversal_cost_per_meter(traversal_cost_per_meter_)  // Store cost per meter for path evaluation
{
  // Check if this vehicle uses differential drive kinematics
  if (const auto* diff_drive = traits.get_differential())
  {
    // Create differential drive constraint with forward direction and reversibility
    constraint = DifferentialDriveConstraint(
      diff_drive->get_forward(),                  // Forward direction vector of the vehicle
      diff_drive->is_reversible());               // Whether vehicle can drive backward
  }
}

//==============================================================================
// Constructor for TraversalFromGenerator
// Sets up the generator to calculate all possible traversals from a given waypoint
TraversalFromGenerator::TraversalFromGenerator(
  const std::shared_ptr<const Supergraph>& graph)  // Shared pointer to the navigation graph
: _graph(graph),                                   // Store weak reference to the graph
  _kinematics(                                     // Initialize kinematics with graph parameters
    graph->traits(),                              // Vehicle traits from the graph
    graph->options(),                             // Interpolation options from the graph
    graph->traversal_cost_per_meter())            // Cost per meter from the graph
{
  // Do nothing - initialization handled by member initializer list
}

//==============================================================================
// Main function to generate all possible traversals from a specific waypoint
// Uses breadth-first search to explore all reachable paths from the starting waypoint
ConstTraversalsPtr TraversalFromGenerator::generate(
  const std::size_t& key,                         // Starting waypoint index to generate traversals from
  const Storage&, // old items are irrelevant     // Previous cache items (not used in this implementation)
  Storage& new_items) const                       // Output storage for newly calculated traversals
{
  //std::cout << "[FROM_GENERATOR] 🚀 CALCULATING traversals FROM waypoint " << key << std::endl;
  
  // Get a strong reference to the supergraph (convert weak_ptr to shared_ptr)
  const auto supergraph = _graph.lock();
  if (!supergraph)
  {
    // If supergraph has been destroyed, throw an error
    throw std::runtime_error(
            "[rmf_traffic::agv::planning::TraversalGenerator::generate] "
            "Supergraph died while a TraversalCache was still being used. "
            "Please report this critical bug to the maintainers of rmf_traffic.");
  }

  // Extract waypoint index and get references to graph components
  const std::size_t waypoint_index = key;         // Starting waypoint for traversal generation
  const auto& graph = supergraph->original();     // Reference to the original navigation graph
  const auto& closures = supergraph->closures();  // Information about which lanes are closed
  const auto& initial_lanes = graph.lanes_from[waypoint_index];  // All lanes starting from this waypoint
  
  // Initialize data structures for breadth-first search
  std::vector<TraversalNode> queue;               // Queue of nodes to be processed (BFS frontier)
  std::vector<Traversal> output;                  // Vector to store completed traversals
  std::unordered_set<std::size_t> visited;       // Set of waypoints already visited to prevent cycles
  visited.insert(waypoint_index);                 // Mark starting waypoint as visited

  // Process all initial lanes from the starting waypoint
  for (const auto l : initial_lanes)
    initiate_traversal(l, graph, closures, _kinematics, queue, output, visited);

  // Continue processing until all reachable paths have been explored
  while (!queue.empty())
  {
    // Get the next node from the queue (using back() for LIFO behavior)
    auto top = std::move(queue.back());
    queue.pop_back();

    // Get all lanes that start from the current node's finish waypoint
    const auto& lanes = graph.lanes_from[top.finish_waypoint_index];
    for (const auto l : lanes)
    {
      // Expand the traversal by following each available lane
      expand_traversal(
        top, l, graph, closures, _kinematics, queue, output, visited);
    }
  }

 // std::cout << "[FROM_GENERATOR] ✅ COMPLETED: Found " << output.size() 
 //           << " traversals FROM waypoint " << waypoint_index << std::endl;

  // Create shared pointer to the completed traversals and store in cache
  auto new_traversals = std::make_shared<Traversals>(std::move(output));
  new_items.insert({waypoint_index, new_traversals});  // Cache the results for future use
  return new_traversals;                          // Return the generated traversals
}

//==============================================================================
// Constructor for TraversalIntoGenerator
// Sets up the generator to calculate all possible traversals that lead TO a given waypoint
TraversalIntoGenerator::TraversalIntoGenerator(
  std::shared_ptr<const CacheManager<TraversalFromCache>> traversals_from,  // Cache manager for "from" traversals
  const std::shared_ptr<const Supergraph>& graph)  // Shared pointer to the navigation graph
: _traversals_from(std::move(traversals_from)),    // Store cache manager for accessing "from" traversals
  _graph(graph)                                    // Store weak reference to the graph
{
  // Do nothing - initialization handled by member initializer list
}

//==============================================================================
// Generate all possible traversals that lead TO a specific waypoint
// This function performs a reverse breadth-first search to find all paths that end at the target waypoint
ConstTraversalsPtr TraversalIntoGenerator::generate(
  const std::size_t& key,                          // Target waypoint index to find traversals TO
  const Storage&, // old items are irrelevant      // Unused parameter for cache compatibility
  Storage& new_items) const                        // Output cache to store generated traversals
{
  // Debug output disabled to reduce terminal clutter
  // std::cout << "[INTO_GENERATOR] 🎯 CALCULATING traversals TO waypoint " << key << std::endl;
  
  // Get a shared pointer to the supergraph (convert weak_ptr to shared_ptr)
  const auto supergraph = _graph.lock();
  if (!supergraph)                                 // Check if the graph still exists
    return nullptr;                                // Return null if graph was destroyed

  // Get reference to the underlying navigation graph structure
  const auto& graph = supergraph->original();
  // Create a shared pointer to store all traversals that lead TO the target waypoint
  const auto traversals_into = std::make_shared<Traversals>();
  // Set to track which waypoints we've already processed (prevents infinite loops)
  std::unordered_set<std::size_t> visited;
  // Queue for breadth-first search - starts with the target waypoint
  std::vector<std::size_t> frontier;
  frontier.push_back(key);                         // Initialize BFS with the target waypoint

  // Continue breadth-first search until all reachable paths are explored
  while (!frontier.empty())
  {
    // Get the next waypoint to process from the frontier (LIFO order)
    const auto next = frontier.back();
    frontier.pop_back();
    // Skip if we've already processed this waypoint
    if (!visited.insert(next).second)
      continue;

    // Get all lanes that lead INTO the current waypoint
    const auto& lanes_into = graph.lanes_into[next];
    // Process each lane that leads to the current waypoint
    for (const auto& lane_index : lanes_into)
    {
      // Get the waypoint that this lane starts FROM
      const auto& waypoint_from =
        graph.lanes[lane_index].entry().waypoint_index();

      // Get all traversals that start FROM the source waypoint
      const auto& traversals_from = _traversals_from->get().get(waypoint_from);
      // Flag to determine if we should continue exploring from this waypoint
      bool keep_exploring = false;
      // Check each traversal from the source waypoint
      for (const auto& traversal : *traversals_from)
      {
        // If this traversal ends at our target waypoint, it's a valid path TO the target
        if (traversal.finish_waypoint_index == key)
        {
          keep_exploring = true;                   // Mark that we found a valid path
          traversals_into->push_back(traversal);   // Add this traversal to our results
        }
      }

      // If we found valid traversals, continue exploring from the source waypoint
      if (keep_exploring)
        frontier.push_back(waypoint_from);         // Add source waypoint to exploration queue
    }
  }

  // Debug output disabled to reduce terminal clutter
  // std::cout << "[INTO_GENERATOR] ✅ COMPLETED: Found " << traversals_into->size() 
  //           << " traversals TO waypoint " << key << std::endl;

  // Cache the results for future use
  new_items.insert({key, traversals_into});
  // Return the generated traversals
  return traversals_into;
}

//==============================================================================
// Factory function to create a new Supergraph instance with all necessary cache managers
// This sets up the complete navigation graph with caching for efficient path planning
std::shared_ptr<const Supergraph> Supergraph::make(
  Graph::Implementation original,                  // The underlying navigation graph structure
  VehicleTraits traits,                           // Vehicle kinematic constraints and capabilities
  LaneClosure lane_closures,                      // Information about which lanes are closed/blocked
  const Interpolate::Options::Implementation& interpolate,  // Interpolation settings for trajectory generation
  double traversal_cost_per_meter)                // Cost per meter for distance-based path evaluation
{
  // Create a new Supergraph instance with the provided parameters
  auto supergraph = std::shared_ptr<Supergraph>(
    new Supergraph(
      std::move(original), std::move(traits),      // Move the graph and traits to avoid copying
      std::move(lane_closures), interpolate, traversal_cost_per_meter));

  // Set up cache manager for traversals FROM each waypoint
  // This caches all possible paths that start from a given waypoint
  supergraph->_traversals_from =
    CacheManager<TraversalFromCache>::make(
    std::make_shared<TraversalFromGenerator>(supergraph));

  // Set up cache manager for traversals INTO each waypoint
  // This caches all possible paths that end at a given waypoint
  supergraph->_traversals_into =
    CacheManager<TraversalIntoCache>::make(
    std::make_shared<TraversalIntoGenerator>(
      supergraph->_traversals_from, supergraph));

  // Set up cache manager for entries into each waypoint
  // This caches entry information for efficient access during planning
  supergraph->_entries_into_waypoint_cache =
    CacheManager<EntriesCache>::make(
    std::make_shared<EntriesGenerator>(supergraph));

  // Calculate the number of lanes for hash table sizing
  const std::size_t N_lanes = supergraph->original().lanes.size();
  // Set up cache manager for lane yaw angles with optimized hash table
  // Uses 251 buckets (prime number) and custom hash function based on lane count
  supergraph->_lane_yaw_cache =
    CacheManager<LaneYawCache>::make(
    std::make_shared<LaneYawGenerator>(supergraph),
    [N_lanes]() { return LaneYawMap(251, EntryHash(N_lanes)); });

  // Return the fully configured Supergraph instance
  return supergraph;
}

//==============================================================================
// Accessor method to retrieve the underlying navigation graph structure
// Returns a const reference to the original graph implementation used for path planning
const Graph::Implementation& Supergraph::original() const
{
  return _original;  // Return the stored navigation graph with waypoints, lanes, and connections
}

//==============================================================================
// Accessor method to retrieve vehicle kinematic constraints and capabilities
// Returns a const reference to the vehicle traits defining motion limits and behavior
const VehicleTraits& Supergraph::traits() const
{
  return _traits;  // Return vehicle-specific constraints like max speed, acceleration, and turning radius
}

//==============================================================================
// Accessor method to retrieve information about closed or blocked lanes
// Returns a const reference to the lane closure data structure for path planning
const LaneClosure& Supergraph::closures() const
{
  return _lane_closures;  // Return information about which lanes are currently unavailable for traversal
}

//==============================================================================
// Accessor method to retrieve interpolation settings for trajectory generation
// Returns a const reference to the interpolation options used for smooth path planning
const Interpolate::Options::Implementation& Supergraph::options() const
{
  return _interpolate;  // Return settings for translation and rotation interpolation thresholds
}

//==============================================================================
// Accessor method to retrieve the cost per meter for distance-based path evaluation
// Returns the cost value used to weight distance in path planning algorithms
double Supergraph::traversal_cost_per_meter() const
{
  return _traversal_cost_per_meter;  // Return the cost factor applied to travel distance in path optimization
}

//==============================================================================
// Accessor method to retrieve floor change information for multi-level navigation
// Returns a const reference to the floor change map for elevators and level transitions
auto Supergraph::floor_change() const -> const FloorChangeMap&
{
  return _floor_changes;  // Return mapping of floor transitions and elevator connections
}

//==============================================================================
// Cache accessor method to retrieve all possible paths starting from a specific waypoint
// Returns cached traversals that begin at the given waypoint index for efficient path planning
ConstTraversalsPtr Supergraph::traversals_from(
  const std::size_t waypoint_index) const
{
 // std::cout << "[CACHE_FROM] 📥 REQUESTING traversals FROM waypoint " << waypoint_index << std::endl;
  return _traversals_from->get().get(waypoint_index);  // Access cached traversal data for paths starting from this waypoint
}

//==============================================================================
// Cache accessor method to retrieve all possible paths ending at a specific waypoint
// Returns cached traversals that terminate at the given waypoint index for efficient path planning
ConstTraversalsPtr Supergraph::traversals_into(
  const std::size_t waypoint_index) const
{
  // Debug output disabled to reduce terminal clutter
  // std::cout << "[CACHE_INTO] 📥 REQUESTING traversals TO waypoint " << waypoint_index << std::endl;
  return _traversals_into->get().get(waypoint_index);  // Access cached traversal data for paths ending at this waypoint
}

//==============================================================================
// Method to filter and return relevant entry points based on vehicle orientation
// Returns a vector of entry points that are valid for the given orientation constraint
std::vector<Supergraph::Entry> Supergraph::Entries::relevant_entries(
  std::optional<double> orientation_opt) const
{
  std::vector<Supergraph::Entry> output;  // Vector to store filtered entry points
  output.reserve(_total_entries);  // Pre-allocate memory for efficiency based on total entry count
  
  // Always include the agnostic entry if it exists (orientation-independent entry point)
  if (_agnostic_entry.has_value())
  {
    output.push_back(*_agnostic_entry);  // Add the orientation-agnostic entry to the result set
  }

  // If no specific orientation is requested, return all possible entries
  if (!orientation_opt.has_value())
  {
    // If there isn't a specific orientation being asked for, then we need to
    // consider every possible entry.
    for (const auto& [_, entry] : _angled_entries)  // Iterate through all orientation-specific entries
      output.push_back(entry);  // Add each entry to the result set

    return output;  // Return all entries since no orientation filtering is needed
  }

  // If no angled entries exist, return only the agnostic entry (if any)
  if (_angled_entries.empty())
    return output;  // Return early with only agnostic entry

  // Normalize the requested orientation to [-π, π] range for consistent comparison
  const double orientation = rmf_utils::wrap_to_pi(*orientation_opt);
  const double lower_bound = _angled_entries.begin()->first;  // Get the minimum orientation angle
  const double upper_bound = _angled_entries.rbegin()->first;  // Get the maximum orientation angle
  
  // Check if the requested orientation is outside the range of available entries
  if (orientation < lower_bound || upper_bound < orientation)
  {
    output.push_back(_angled_entries.begin()->second);  // Add the entry with minimum orientation
    if (lower_bound != upper_bound)  // If there are multiple entries with different orientations
      output.push_back(_angled_entries.rbegin()->second);  // Add the entry with maximum orientation

    return output;  // Return the boundary entries for out-of-range orientations
  }

  // Find the entry with orientation closest to the requested orientation
  const auto it = _angled_entries.lower_bound(orientation);  // Find first entry >= requested orientation
  output.push_back(it->second);  // Add the entry with orientation >= requested orientation
  
  // If the found entry's orientation is greater than requested, also include the previous entry
  if (orientation < it->first)
  {
    // it cannot be begin() because the earlier if-statement would have caught
    // it if it were. So we can safely decrement and dereference this iterator,
    // and it will certainly provide the lower bound for the requested
    // orientation.
    output.push_back((--std::map<double, Entry>::const_iterator(it))->second);  // Add the entry with orientation < requested orientation
  }

  return output;  // Return the filtered entries that are relevant for the given orientation
}

//==============================================================================
Supergraph::Entries::Entries(
  std::map<double, Entry> angled_entries,
  std::optional<Entry> agnostic_entry)
: _angled_entries(std::move(angled_entries)),
  _agnostic_entry(std::move(agnostic_entry))
{
  _total_entries = _angled_entries.size()
    + (_agnostic_entry.has_value() ? 1 : 0);
}

//==============================================================================
Supergraph::ConstEntriesPtr Supergraph::entries_into(
  const std::size_t waypoint_index) const
{
  return _entries_into_waypoint_cache->get().get(waypoint_index);
}

//==============================================================================
std::optional<double> Supergraph::yaw_of(const Entry& entry) const
{
  // TODO(MXG): Try going back to the caching system when time permits.
//  if (entry.orientation == Orientation::Any)
//    return std::nullopt;

//  return _lane_yaw_cache->get().get(entry);

  // Check if the entry has any orientation constraint - if so, no specific yaw can be determined
  if (entry.orientation == Orientation::Any)
    return std::nullopt;

  // If no differential drive constraint is available, we cannot calculate orientation-specific yaw
  if (!_constraint.has_value())
    return std::nullopt;

  // Get the lane information from the navigation graph using the entry's lane index
  const auto& lane = _original.lanes[entry.lane];
  // Extract the waypoint indices for the start and end of this lane
  const std::size_t waypoint_index_0 = lane.entry().waypoint_index();
  const std::size_t waypoint_index_1 = lane.exit().waypoint_index();
  // Get the actual waypoint objects from the graph
  const auto& wp0 = _original.waypoints[waypoint_index_0];
  const auto& wp1 = _original.waypoints[waypoint_index_1];

  // Extract the 2D positions of the start and end waypoints
  const Eigen::Vector2d p0 = wp0.get_location();
  const Eigen::Vector2d p1 = wp1.get_location();
  // Calculate the Euclidean distance between the waypoints
  const double dist = (p1 - p0).norm();
  // If the distance is too small (below interpolation threshold), orientation is not meaningful
  if (dist <= _interpolate.translation_thresh)
    return std::nullopt;

  // Calculate the normalized direction vector from start to end waypoint
  const Eigen::Vector2d course_vector = (p1 - p0)/dist;
  // Get all possible orientations for this direction vector based on vehicle constraints
  const auto orientations = _constraint->get_orientations(course_vector);
  // Return the specific yaw angle for the requested orientation (Forward, Backward, or Any)
  return orientations[static_cast<std::size_t>(entry.orientation)];
}

//==============================================================================
// Generate a set of differential drive keys for path planning between start and goal waypoints
// These keys represent all possible combinations of lanes, orientations, and entry points
DifferentialDriveKeySet Supergraph::keys_for(
  const std::size_t start_waypoint_index,        // Index of the starting waypoint in the navigation graph
  const std::size_t goal_waypoint_index,         // Index of the target waypoint in the navigation graph
  std::optional<double> goal_orientation) const  // Optional specific orientation constraint for the goal
{
  // Define hash function type for the differential drive key set
  using KeyHash = DifferentialDriveMapTypes::KeyHash;
  // Create hash set with 31 buckets and custom hash function based on total number of lanes
  DifferentialDriveKeySet keys(31, KeyHash{_original.lanes.size()});

  // Get all entry points into the goal waypoint that are relevant for the given orientation
  const auto relevant_goal_entries = entries_into(goal_waypoint_index)
    ->relevant_entries(goal_orientation);

  // Get all possible traversals that start from the specified waypoint
  const auto relevant_traversals = traversals_from(start_waypoint_index);
  // Ensure that traversals were successfully retrieved (should never be null)
  assert(relevant_traversals);

  // Iterate through each possible traversal from the start waypoint
  for (const auto& traversal : *relevant_traversals)
  {
    // Get the initial lane index for this traversal
    const std::size_t lane_index = traversal.initial_lane_index;
    // Check all three possible orientations: Forward (0), Backward (1), Any (2)
    for (std::size_t orientation = 0; orientation < 3; ++orientation)
    {
      // Get the alternative trajectory for this specific orientation
      const auto& alt = traversal.alternatives[orientation];
      // Skip if this orientation is not valid for this traversal
      if (!alt.has_value())
        continue;

      // For each valid goal entry, create a key combining start and goal information
      for (const auto& entry : relevant_goal_entries)
      {
        // Insert a key representing: start lane, start orientation, start side, goal lane, goal orientation
        keys.insert(
          {
            lane_index, Orientation(orientation), Side::Start,
            entry.lane, entry.orientation
          });
      }
    }
  }

  // Return the complete set of keys representing all possible path combinations
  return keys;
}

//==============================================================================
// Constructor for EntriesGenerator that sets up differential drive constraints
// This generator creates entry information for waypoints based on vehicle kinematics
Supergraph::EntriesGenerator::EntriesGenerator(
  const std::shared_ptr<const Supergraph>& graph)  // Shared pointer to the navigation graph
: _graph(graph)  // Store weak reference to avoid circular dependencies
{
  // Check if the vehicle has differential drive characteristics
  if (const auto* differential = graph->traits().get_differential())
  {
    // Create constraint object with forward direction and reversibility settings
    _constraint = DifferentialDriveConstraint(
      differential->get_forward(),    // Maximum forward speed
      differential->is_reversible()); // Whether vehicle can move backward
  }
}

//==============================================================================
// Generate entry information for a specific waypoint, creating all possible entry combinations
// This function is called by the cache system when entry data is needed
auto Supergraph::EntriesGenerator::generate(
  const std::size_t& key,              // The waypoint index to generate entries for
  const Storage&,                      // Old cached items (not used in this implementation)
  Storage& new_items) const -> ConstEntriesPtr  // Output storage for newly generated entries
{
  // Get a strong reference to the supergraph (may fail if graph was destroyed)
  const auto supergraph = _graph.lock();

  // TODO(MXG): When we have C++20 support, we can label this with [[unlikely]]
  // Check if the supergraph still exists (should not happen in normal operation)
  if (!supergraph)
  {
    // This means the supergraph that's being traversed has destructed while
    // this cache is still alive. That's really weird and shouldn't happen.
    // The only reason we keep the graph as a nullptr is
    // 1) to avoid a circular dependency
    // 2) we cannot technically guarantee that the cache's lifecycle will fit
    //    within the supergraph's lifecycle, and throwing an exception is
    //    preferable to Undefined Behavior.
    throw std::runtime_error(
            "[rmf_traffic::agv::planning::Supergraph::EntriesGenerator::generate]"
            " Supergraph died while a EntriesCache was still being used. "
            "Please report this critical bug to the maintainers of rmf_traffic.");
  }

  // Extract the waypoint index from the key
  const std::size_t waypoint_index = key;
  // Get references to the navigation graph and interpolation settings
  const auto& graph = supergraph->original();
  const auto& interpolate = supergraph->options();
  // Get all lanes that lead into this waypoint
  const auto& entry_lanes = graph.lanes_into[waypoint_index];

  // Storage for entries with specific orientation angles
  std::map<double, Entry> angled_entries;
  // Storage for entries that work with any orientation
  std::optional<Entry> agnostic_entry;

  // Get the 2D position of the target waypoint
  const Eigen::Vector2d p1 = graph.waypoints[waypoint_index].get_location();

  // Process each lane that leads into this waypoint
  for (const auto& lane_index : entry_lanes)
  {
    // Get the lane object and its starting waypoint
    const auto& lane = graph.lanes[lane_index];
    const auto& wp0 = graph.waypoints[lane.entry().waypoint_index()];
    // Get the 2D position of the lane's starting waypoint
    const Eigen::Vector2d p0 = wp0.get_location();

    // Calculate the distance between start and end waypoints of this lane
    const double dist = (p1 - p0).norm();
    // If no constraint exists or distance is too small, create an orientation-agnostic entry
    if (!_constraint.has_value() || dist < interpolate.translation_thresh)
    {
      // Create entry that works with any orientation for short distances
      agnostic_entry = Entry{lane_index, Orientation::Any, Side::Finish};
    }
    else
    {
      // Calculate the normalized direction vector from start to end waypoint
      const Eigen::Vector2d course_vector = (p1 - p0)/dist;
      // Get all valid orientations for this direction based on vehicle constraints
      const auto orientations =
        _constraint->get_orientations(course_vector);

      // Process each possible orientation for this lane
      for (std::size_t i = 0; i < orientations.size(); ++i)
      {
        // Get the specific orientation angle for this index
        const auto orientation = orientations[i];
        // Skip if this orientation is not valid for this direction
        if (!orientation.has_value())
          continue;

        // Store this entry with its specific orientation angle as the key
        angled_entries.insert(
          {*orientation, Entry{lane_index, Orientation(i), Side::Finish}});
      }
    }
  }

  // Create a shared pointer to store the newly generated entries with their orientation data
  auto new_entries = std::make_shared<Entries>(
    std::move(angled_entries), std::move(agnostic_entry));

  // Cache the generated entries in the storage for future lookups using the waypoint index as key
  new_items.insert({key, new_entries});
  // Return the newly created entries object for immediate use
  return new_entries;
}

//==============================================================================
// Constructor for LaneYawGenerator - initializes the generator with a reference to the supergraph
Supergraph::LaneYawGenerator::LaneYawGenerator(
  const std::shared_ptr<const Supergraph>& graph)
: _graph(graph)  // Store weak reference to the supergraph to avoid circular dependencies
{
  // Check if the vehicle has differential drive constraints
  if (const auto* diff_drive = graph->traits().get_differential())
  {
    // We pretend the vehicle is always reversible, because this isn't the place
    // where the reversibility constraint is enforced.
    // Create constraint with forward speed and assume reversibility for yaw calculation purposes
    _constraint = DifferentialDriveConstraint(diff_drive->get_forward(), true);
  }
}

//==============================================================================
// Generate yaw angle for a specific lane entry based on vehicle orientation constraints
std::optional<double> Supergraph::LaneYawGenerator::generate(
  const Entry& key,                    // Lane entry with specific orientation and side
  const Storage& /*old_items*/,        // Unused parameter for cache compatibility
  Storage& new_items) const            // Output cache to store generated yaw values
{
  // Handle case where orientation is "Any" - no specific yaw angle needed
  if (key.orientation == Orientation::Any)
  {
    // Generate entries for all possible sides (Start, Finish) with no yaw constraint
    for (std::size_t j = 0; j <= static_cast<std::size_t>(Side::Finish); ++j)
      new_items.insert({{key.lane, Orientation::Any, Side(j)}, std::nullopt});
    // Return null since no specific yaw is required for "Any" orientation
    return std::nullopt;
  }

  // Handle case where no differential drive constraint exists
  if (!_constraint.has_value())
  {
    // Get the maximum orientation index (Any = 2)
    const auto any = static_cast<std::size_t>(Orientation::Any);
    // Generate entries for all orientations and sides with no yaw constraint
    for (std::size_t i = 0; i <= any; ++i)
    {
      for (std::size_t j = 0; j <= static_cast<std::size_t>(Side::Finish); ++j)
        new_items.insert({{key.lane, Orientation(i), Side(j)}, std::nullopt});
    }

    // Return null since no constraint means no specific yaw calculation
    return std::nullopt;
  }

  // Convert weak pointer to shared pointer to access the supergraph
  const auto supergraph = _graph.lock();

  // TODO(MXG): When we have C++20 support, we can label this with [[unlikely]]
  // Check if the supergraph still exists (safety check for cache lifecycle)
  if (!supergraph)
  {
    // This means the supergraph that's being traversed has destructed while
    // this cache is still alive. That's really weird and shouldn't happen.
    // The only reason we keep the graph as a nullptr is
    // 1) to avoid a circular dependency
    // 2) we cannot technically guarantee that the cache's lifecycle will fit
    //    within the supergraph's lifecycle, and throwing an exception is
    //    preferable to Undefined Behavior.
    // Throw runtime error if supergraph was destroyed while cache is still active
    throw std::runtime_error(
            "[rmf_traffic::agv::planning::Supergraph::EntriesGenerator::generate]"
            " Supergraph died while a EntriesCache was still being used. "
            "Please report this critical bug to the maintainers of rmf_traffic.");
  }

  // Get reference to the original navigation graph structure
  const auto& original = supergraph->original();
  // Get the specific lane and its waypoint indices
  const auto& lane = original.lanes[key.lane];
  const std::size_t waypoint_index_0 = lane.entry().waypoint_index();  // Starting waypoint index
  const std::size_t waypoint_index_1 = lane.exit().waypoint_index();   // Ending waypoint index
  // Get the actual waypoint objects from the graph
  const auto& wp0 = original.waypoints[waypoint_index_0];
  const auto& wp1 = original.waypoints[waypoint_index_1];

  // Extract 2D positions of start and end waypoints
  const Eigen::Vector2d p0 = wp0.get_location();
  const Eigen::Vector2d p1 = wp1.get_location();
  // Calculate Euclidean distance between waypoints
  const double dist = (p1 - p0).norm();
  // If distance is too small, orientation calculation is not meaningful
  if (dist <= supergraph->options().translation_thresh)
  {
    // Get the maximum orientation index (Any = 2)
    const auto any = static_cast<std::size_t>(Orientation::Any);
    // Generate entries for all orientations and sides with no yaw constraint for short distances
    for (std::size_t i = 0; i <= any; ++i)
    {
      for (std::size_t j = 0; j <= static_cast<std::size_t>(Side::Finish); ++j)
        new_items.insert({{key.lane, Orientation(i), Side(j)}, std::nullopt});
    }

    // Return null since short distances don't have meaningful orientation
    return std::nullopt;
  }

  // Calculate normalized direction vector from start to end waypoint
  const Eigen::Vector2d course_vector = (p1 - p0)/dist;
  // Get all valid orientations for this direction based on vehicle constraints
  const auto orientations = _constraint->get_orientations(course_vector);
  // Process each possible orientation for this lane direction
  for (std::size_t i = 0; i < orientations.size(); ++i)
  {
    // Get the specific yaw angle for this orientation index
    const auto yaw = orientations[i];
    // Assert that the yaw value is valid (should not be nullopt)
    assert(yaw.has_value());

    // Generate entries for all sides (Start, Finish) with this specific yaw angle
    for (std::size_t j = 0; j <= static_cast<std::size_t>(Side::Finish); ++j)
      new_items.insert({{key.lane, Orientation(i), Side(j)}, yaw});
  }

  // Return the yaw angle for the specific orientation requested in the key
  return orientations[static_cast<std::size_t>(key.orientation)];
}

//==============================================================================
// Constructor for Supergraph - initializes the navigation graph with vehicle constraints and options
Supergraph::Supergraph(Graph::Implementation original,        // Original navigation graph structure
  VehicleTraits traits,                                       // Vehicle movement capabilities and constraints
  LaneClosure lane_closures,                                  // Information about closed/blocked lanes
  const Interpolate::Options::Implementation& interpolate,    // Interpolation settings for path planning
  double traversal_cost_per_meter)                            // Cost per meter for path traversal
: _original(std::move(original)),                             // Store the navigation graph
  _traits(std::move(traits)),                                 // Store vehicle traits
  _lane_closures(std::move(lane_closures)),                   // Store lane closure information
  _interpolate(interpolate),                                  // Store interpolation options
  _traversal_cost_per_meter(traversal_cost_per_meter),        // Store traversal cost
  _floor_changes(find_floor_changes(_original))               // Find and store floor change information
{
  // Check if the vehicle has differential drive constraints
  if (const auto* diff = _traits.get_differential())
  {
    // Create differential drive constraint with forward speed and reversibility settings
    _constraint = DifferentialDriveConstraint(
      diff->get_forward(), diff->is_reversible());
  }
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
