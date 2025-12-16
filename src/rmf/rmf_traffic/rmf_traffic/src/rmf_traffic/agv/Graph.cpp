/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

// Include the internal graph implementation header
#include "internal_Graph.hpp"

// Include the main graph header for public interface
#include <rmf_traffic/agv/Graph.hpp>
// Include iostream for console output/debugging
#include <iostream>

// Include utility headers for mathematical operations and optional values
#include <rmf_utils/math.hpp>
#include <rmf_utils/optional.hpp>

// Define the main namespace for RMF traffic
namespace rmf_traffic {
// Define the AGV (Autonomous Ground Vehicle) sub-namespace
namespace agv {

//==============================================================================
// Implementation class for lift properties - uses PIMPL pattern for encapsulation
class Graph::LiftProperties::Implementation
{
public:
  // Name identifier for the lift
  std::string name;
  // 2D position of the lift in the map coordinate system
  Eigen::Vector2d location;
  // Orientation angle of the lift (in radians)
  double orientation;
  // Half dimensions of the lift (width/2, height/2) for collision detection
  Eigen::Vector2d half_dimensions;
  // Inverse transformation matrix for converting world coordinates to lift-local coordinates
  Eigen::Isometry2d tf_inv;

  // Static method to update one lift properties object with another
  static void update(
    LiftProperties& original,    // The lift properties to be updated
    const LiftProperties& incoming)  // The source lift properties
  {
    // Copy all implementation data from incoming to original
    *original._pimpl = *incoming._pimpl;
  }
};

//==============================================================================
// Getter method to retrieve the lift's name
const std::string& Graph::LiftProperties::name() const
{
  // Return reference to the name stored in the implementation
  return _pimpl->name;
}

//==============================================================================
// Getter method to retrieve the lift's 2D location
Eigen::Vector2d Graph::LiftProperties::location() const
{
  // Return copy of the location vector from implementation
  return _pimpl->location;
}

//==============================================================================
// Getter method to retrieve the lift's orientation angle
double Graph::LiftProperties::orientation() const
{
  // Return the orientation value from implementation
  return _pimpl->orientation;
}

//==============================================================================
// Getter method to retrieve the full dimensions of the lift
Eigen::Vector2d Graph::LiftProperties::dimensions() const
{
  // Convert half dimensions back to full dimensions by multiplying by 2
  return 2.0 * _pimpl->half_dimensions;
}

//==============================================================================
// Method to check if a given position is inside the lift area
bool Graph::LiftProperties::is_in_lift(
  Eigen::Vector2d position,  // The position to check (in world coordinates)
  double envelope) const     // Safety envelope around the lift boundaries
{
  // Transform world position to lift-local coordinate system
  Eigen::Vector2d p_local = _pimpl->tf_inv * position;
  
  // Check both X and Y dimensions
  for (int i = 0; i < 2; ++i)
  {
    // Check if position is too far in negative direction (with envelope)
    if (p_local[i]  < -_pimpl->half_dimensions[i] - envelope)
      return false;

    // Check if position is too far in positive direction (with envelope)
    if (_pimpl->half_dimensions[i] + envelope < p_local[i])
      return false;
  }

  // Position is within lift boundaries (including envelope)
  return true;
}

//==============================================================================
// Helper function to create inverse transformation matrix for lift coordinate system
Eigen::Isometry2d make_lift_tf_inv(Eigen::Vector2d location, double orientation)
{
  // Start with identity transformation matrix
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
  // Apply translation to the lift's location
  tf.translate(location);
  // Apply rotation by the lift's orientation
  tf.rotate(orientation);
  // Return the inverse transformation (world to lift-local coordinates)
  return tf.inverse();
}

//==============================================================================
// Constructor for LiftProperties class
Graph::LiftProperties::LiftProperties(
  std::string name,              // Name of the lift
  Eigen::Vector2d location,      // 2D position of the lift
  double orientation,            // Orientation angle of the lift
  Eigen::Vector2d dimensions)    // Full dimensions (width, height) of the lift
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation {
        std::move(name),                                    // Move name to avoid copy
        location,                                           // Copy location
        orientation,                                        // Copy orientation
        dimensions / 2.0,                                   // Convert to half dimensions
        make_lift_tf_inv(location, orientation)            // Create inverse transformation
      }))
{
  // Constructor body is empty - all initialization done in member initializer list
}

//==============================================================================
// Implementation class for DoorProperties - stores door geometry and metadata
class Graph::DoorProperties::Implementation
{
public:
  std::string name;        // Unique identifier for the door
  Eigen::Vector2d start;   // Starting point of the door segment in 2D space
  Eigen::Vector2d end;     // Ending point of the door segment in 2D space
  std::string map;         // Name of the map/floor this door belongs to
};

//==============================================================================
// Accessor method to retrieve the door's unique identifier
const std::string& Graph::DoorProperties::name() const
{
  return _pimpl->name;  // Return reference to the door's name string
}

//==============================================================================
// Accessor method to retrieve the door's starting point coordinates
Eigen::Vector2d Graph::DoorProperties::start() const
{
  return _pimpl->start;  // Return copy of the door's starting position vector
}

//==============================================================================
// Accessor method to retrieve the door's ending point coordinates
Eigen::Vector2d Graph::DoorProperties::end() const
{
  return _pimpl->end;  // Return copy of the door's ending position vector
}

//==============================================================================
// Accessor method to retrieve the map name this door belongs to
const std::string& Graph::DoorProperties::map() const
{
  return _pimpl->map;  // Return reference to the door's map name string
}

//==============================================================================
// Anonymous namespace for helper functions used in door intersection calculations
namespace {
// Calculate the minimum distance from a point to a line segment
// Returns the shortest distance between point q and the line segment from p0 to p1
double distance_from_point_to_segment(
  Eigen::Vector2d q,    // The point to measure distance from
  Eigen::Vector2d p0,   // Starting point of the line segment
  Eigen::Vector2d p1)   // Ending point of the line segment
{
  // Calculate distance to both endpoints of the segment
  const double endpoint_distance = std::min((q - p0).norm(), (q - p1).norm());
  // Calculate the length of the line segment
  const auto L = (p1 - p0).norm();
  // If segment is too short (degenerate case), return distance to nearest endpoint
  if (L < 1e-3)
  {
    return endpoint_distance;
  }

  // Calculate unit direction vector along the line segment
  const Eigen::Vector2d n = (p1 - p0) / L;
  // Vector from segment start to the query point
  const Eigen::Vector2d v = q - p0;
  // Project the query point onto the line segment
  const Eigen::Vector2d q_proj = (v.dot(n)) * n;
  // Calculate perpendicular distance from point to line
  const double ortho_distance = (q - q_proj).norm();
  // Return minimum of perpendicular distance and endpoint distances
  return std::min(ortho_distance, endpoint_distance);
}
} // anonymous namespace

//==============================================================================
// Check if a door intersects with a given line segment within a specified envelope
// Returns true if the door and line segment are within 'envelope' distance of each other
bool Graph::DoorProperties::intersects(
  Eigen::Vector2d p0,    // Starting point of the test line segment
  Eigen::Vector2d p1,    // Ending point of the test line segment
  double envelope) const // Maximum distance for intersection detection
{
  // Get door's start and end points from implementation
  const auto q0 = _pimpl->start;
  const auto q1 = _pimpl->end;
  // Test all four combinations of point-to-segment distances
  for (const auto test : std::vector<std::function<double()>>{
      [&]{ return distance_from_point_to_segment(p0, q0, q1); },  // Distance from p0 to door segment
      [&]{ return distance_from_point_to_segment(p1, q0, q1); },  // Distance from p1 to door segment
      [&]{ return distance_from_point_to_segment(q0, p0, p1); },  // Distance from door start to test segment
      [&]{ return distance_from_point_to_segment(_pimpl->end, p0, p1); }  // Distance from door end to test segment
    })
  {
    // Calculate the distance for this test case
    const double distance = test();
    // If any distance is within envelope, segments intersect
    if (distance <= envelope)
      return true;
  }

  // If none of the endpoints are within range of the other lines, then the only
  // way for an intersection to exist is if the lines truly cross each other.
  // Calculate determinant for line intersection test (cross product of direction vectors)
  const double det = (p0.x() - p1.x()) * (q0.y() - q1.y())
    - (p0.y() - p1.y()) * (q0.x() - q1.x());

  // Check if lines are parallel (determinant close to zero)
  if (std::abs(det) < 1e-8)
  {
    // The lines are essentially parallel and their endpoints aren't close
    // enough, so there is no intersection.
    return false;
  }

  // Calculate parameter t for intersection point on first line segment (p0 to p1)
  const double t = ( (p0.x() - q0.x()) * (q0.y() - q1.y())
    - (p0.y() - q0.y()) * (q0.x() - q1.x()) )
    / det;

  // Check if intersection point lies within first line segment bounds
  if (t < 0.0 || 1.0 < t)
    return false;

  // Calculate parameter u for intersection point on second line segment (q0 to q1)
  const double u = ( (p0.x() - q0.x()) * (p0.y() - p1.y())
    - (p0.y() - q0.y()) * (p0.x() - p1.x()) )
    / det;

  // Check if intersection point lies within second line segment bounds
  if (u < 0.0 || 1.0 < u)
    return false;

  // Both parameters are within bounds, so line segments truly intersect
  return true;
}

//==============================================================================
// Constructor for DoorProperties - creates a door with specified geometry and map location
Graph::DoorProperties::DoorProperties(
  std::string name,        // Human-readable name identifier for the door
  Eigen::Vector2d start,   // 2D coordinates of the door's starting point
  Eigen::Vector2d end,     // 2D coordinates of the door's ending point  
  std::string map)         // Name of the map where this door is located
: _pimpl(rmf_utils::make_impl<Implementation>(  // Initialize private implementation using pimpl pattern
    Implementation {       // Create implementation struct with provided parameters
      std::move(name),     // Move door name to avoid copying
      start,               // Copy start coordinates (Eigen::Vector2d is small)
      end,                 // Copy end coordinates (Eigen::Vector2d is small)
      std::move(map)       // Move map name to avoid copying
    }))
{
  // Do nothing - all initialization handled in member initializer list
}

//==============================================================================
// Private implementation class for Waypoint - contains all waypoint data and behavior
class Graph::Waypoint::Implementation
{
public:

  std::size_t index;       // Unique numerical identifier for this waypoint in the graph

  std::string map_name;    // Name of the map/floor where this waypoint is located

  Eigen::Vector2d location;  // 2D coordinates (x, y) of the waypoint's position

  rmf_utils::optional<std::string> name = rmf_utils::nullopt;  // Optional human-readable name for the waypoint

  bool holding_point = false;      // Flag indicating if vehicles must stop and wait at this waypoint

  bool passthrough_point = false;  // Flag indicating if this is just a path waypoint (no stopping required)

  bool parking_spot = false;       // Flag indicating if this waypoint can be used for vehicle parking

  bool charger = false;            // Flag indicating if this waypoint has charging capabilities

  LiftPropertiesPtr in_lift = nullptr;  // Pointer to lift properties if this waypoint is inside an elevator

  std::string mutex_group = "";    // Name of mutex group for coordinating access to this waypoint

  std::optional<double> merge_radius = std::nullopt;  // Optional radius for merging nearby waypoints

  // Factory method to create a Waypoint with variadic arguments
  template<typename... Args>
  static Waypoint make(Args&& ... args)
  {
    Waypoint result;  // Create empty waypoint object
    result._pimpl = rmf_utils::make_impl<Implementation>(  // Initialize implementation with forwarded arguments
      Implementation{std::forward<Args>(args)...});

    return result;  // Return the constructed waypoint
  }

  // Helper method to get reference to implementation from waypoint object
  static Waypoint::Implementation& get(Waypoint& wp)
  {
    return *wp._pimpl;  // Dereference the shared pointer to get implementation reference
  }
};

//==============================================================================
// Getter method to retrieve the map name where this waypoint is located
const std::string& Graph::Waypoint::get_map_name() const
{
  return _pimpl->map_name;  // Return const reference to the map name string
}

//==============================================================================
// Setter method to update the map name for this waypoint
auto Graph::Waypoint::set_map_name(std::string map) -> Waypoint&
{
  _pimpl->map_name = std::move(map);  // Move the new map name to avoid copying
  return *this;  // Return reference to this waypoint for method chaining
}

//==============================================================================
// Getter method to retrieve the 2D location coordinates of this waypoint
const Eigen::Vector2d& Graph::Waypoint::get_location() const
{
  return _pimpl->location;  // Return const reference to the location vector
}

//==============================================================================
// Setter method to update the 2D location coordinates of this waypoint
auto Graph::Waypoint::set_location(Eigen::Vector2d location) -> Waypoint&
{
  _pimpl->location = std::move(location);  // Move the new location to avoid copying
  return *this;  // Return reference to this waypoint for method chaining
}

//==============================================================================
// Getter method to check if this waypoint is designated as a holding point
bool Graph::Waypoint::is_holding_point() const
{
  return _pimpl->holding_point;  // Return the boolean flag indicating holding point status
}

//==============================================================================
// Setter method to mark this waypoint as a holding point or not
auto Graph::Waypoint::set_holding_point(bool _is_holding_point) -> Waypoint&
{
  _pimpl->holding_point = _is_holding_point;  // Set the holding point flag
  return *this;  // Return reference to this waypoint for method chaining
}

//==============================================================================
// Getter method to check if this waypoint is designated as a passthrough point
bool Graph::Waypoint::is_passthrough_point() const
{
  return _pimpl->passthrough_point;  // Return the boolean flag indicating passthrough point status
}

//==============================================================================
// Setter method to mark this waypoint as a passthrough point or not
auto Graph::Waypoint::set_passthrough_point(bool _is_passthrough) -> Waypoint&
{
  _pimpl->passthrough_point = _is_passthrough;  // Set the passthrough point flag
  return *this;  // Return reference to this waypoint for method chaining
}

//==============================================================================
// Getter method to check if this waypoint is designated as a parking spot
bool Graph::Waypoint::is_parking_spot() const
{
  return _pimpl->parking_spot;  // Return the boolean flag indicating parking spot status
}

//==============================================================================
// Setter method to mark this waypoint as a parking spot or not
auto Graph::Waypoint::set_parking_spot(bool _is_parking_spot) -> Waypoint&
{
  _pimpl->parking_spot = _is_parking_spot;  // Set the parking spot flag
  return *this;  // Return reference to this waypoint for method chaining
}

//==============================================================================
// Getter method to check if this waypoint has charging capabilities
bool Graph::Waypoint::is_charger() const
{
  return _pimpl->charger;  // Return the boolean flag indicating charger status
}

//==============================================================================
// Setter method to mark this waypoint as having charging capabilities or not
auto Graph::Waypoint::set_charger(bool _is_charger) -> Waypoint&
{
  _pimpl->charger = _is_charger;  // Set the charger flag
  return *this;  // Return reference to this waypoint for method chaining
}

//==============================================================================
// Getter method to retrieve the lift properties associated with this waypoint
// Returns a shared pointer to the lift properties if this waypoint is inside a lift
auto Graph::Waypoint::in_lift() const -> LiftPropertiesPtr
{
  return _pimpl->in_lift;  // Return the lift properties pointer from the implementation
}

//==============================================================================
// Setter method to associate this waypoint with a specific lift
// Sets the lift properties and returns a reference to this waypoint for method chaining
auto Graph::Waypoint::set_in_lift(LiftPropertiesPtr lift) -> Waypoint&
{
  _pimpl->in_lift = lift;  // Store the lift properties pointer in the implementation
  return *this;  // Return reference to this waypoint for method chaining
}

//==============================================================================
// Getter method to retrieve the unique index of this waypoint within the navigation graph
// Returns the waypoint's position in the graph's waypoint array
std::size_t Graph::Waypoint::index() const
{
  return _pimpl->index;  // Return the waypoint index from the implementation
}

//==============================================================================
// Getter method to retrieve the optional name of this waypoint
// Returns a pointer to the name string if it exists, otherwise returns nullptr
const std::string* Graph::Waypoint::name() const
{
  if (_pimpl->name)  // Check if the waypoint has a name assigned
    return &_pimpl->name.value();  // Return pointer to the name string

  return nullptr;  // Return null if no name is assigned to this waypoint
}

//==============================================================================
// Method to get either the waypoint name or index as a formatted string
// Uses name_format if waypoint has a name, otherwise uses index_format with the waypoint index
std::string Graph::Waypoint::name_or_index(
  const std::string& name_format,    // Format string for waypoint name (use %s as placeholder)
  const std::string& index_format) const  // Format string for waypoint index (use %d as placeholder)
{
  if (_pimpl->name)  // Check if waypoint has a name assigned
  {
    const auto it = name_format.find_first_of("%s");  // Find the %s placeholder in name format
    if (it == std::string::npos)  // If no placeholder found, return format as-is
      return name_format;

    return name_format.substr(0, it)  // Return format before placeholder
      + _pimpl->name.value()          // Plus the actual waypoint name
      + name_format.substr(it+2);     // Plus format after placeholder
  }

  const auto it = index_format.find_first_of("%d");  // Find the %d placeholder in index format
  if (it == std::string::npos)  // If no placeholder found, return format as-is
    return index_format;

  return index_format.substr(0, it)  // Return format before placeholder
    + std::to_string(_pimpl->index)  // Plus the waypoint index as string
    + index_format.substr(it+2);     // Plus format after placeholder
}

//==============================================================================
// Getter method to retrieve the mutex group name that this waypoint belongs to
// Returns the name of the mutex group for coordination between multiple robots
const std::string& Graph::Waypoint::in_mutex_group() const
{
  return _pimpl->mutex_group;  // Return the mutex group name from the implementation
}

//==============================================================================
// Setter method to assign this waypoint to a specific mutex group
// Sets the mutex group name and returns a reference to this waypoint for method chaining
auto Graph::Waypoint::set_in_mutex_group(std::string group_name) -> Waypoint&
{
  _pimpl->mutex_group = std::move(group_name);  // Move the group name to the implementation
  return *this;  // Return reference to this waypoint for method chaining
}

//==============================================================================
// Getter method to retrieve the optional merge radius for this waypoint
// Returns the merge radius if set, which defines how close robots can get before merging paths
std::optional<double> Graph::Waypoint::merge_radius() const
{
  return _pimpl->merge_radius;  // Return the merge radius from the implementation
}

//==============================================================================
// Setter method to set the merge radius for this waypoint
// Sets the merge radius value and returns a reference to this waypoint for method chaining
auto Graph::Waypoint::set_merge_radius(std::optional<double> value) -> Waypoint&
{
  _pimpl->merge_radius = value;  // Store the merge radius in the implementation
  return *this;  // Return reference to this waypoint for method chaining
}

//==============================================================================
// Default constructor for Graph::Waypoint
// Initializes a new waypoint with default values
Graph::Waypoint::Waypoint()
{
  // Do nothing - default initialization handled by member initializer list
}

namespace {
//==============================================================================
// Constraint class that restricts robot orientation to a predefined set of acceptable angles
// This is used when a waypoint or lane has specific orientation requirements
class AcceptableOrientationConstraint : public Graph::OrientationConstraint
{
public:

  // Constructor that takes a vector of acceptable orientation angles in radians
  AcceptableOrientationConstraint(std::vector<double> acceptable)
  : orientations(std::move(acceptable))  // Store the acceptable orientations
  {
    // Do nothing - initialization handled by member initializer list
  }

  std::vector<double> orientations;  // Vector storing all acceptable orientation angles

  // Apply the orientation constraint to a robot's position
  // Finds the closest acceptable orientation to the current position and updates it
  bool apply(Eigen::Vector3d& position,
    const Eigen::Vector2d& /*course_vector*/) const final
  {
    assert(!orientations.empty());  // Ensure we have at least one acceptable orientation
    // This constraint can never be satisfied if there are no acceptable
    // orientations.
    if (orientations.empty())  // Double-check for empty orientations vector
      return false;

    const double p = position[2];  // Get current yaw angle from position vector
    double closest = p;  // Initialize closest orientation to current position
    double best_diff = std::numeric_limits<double>::infinity();  // Initialize best difference to infinity
    for (const double theta : orientations)  // Iterate through all acceptable orientations
    {
      const double diff = std::abs(rmf_utils::wrap_to_pi(theta - p));  // Calculate wrapped angular difference
      if (diff < best_diff)  // If this orientation is closer than previous best
      {
        closest = theta;  // Update closest orientation
        best_diff = diff;  // Update best difference
      }
    }

    position[2] = closest;  // Set the position's yaw to the closest acceptable orientation
    return true;  // Return true indicating constraint was successfully applied
  }

  // Create a deep copy of this orientation constraint
  rmf_utils::clone_ptr<OrientationConstraint> clone() const final
  {
    return rmf_utils::make_clone<AcceptableOrientationConstraint>(*this);  // Return cloned instance
  }

};

//==============================================================================
// TODO(MXG): Think about how to refactor this constraint so that it can share
// an implementation with DifferentialOrientationConstraint. Maybe instead of
// a single direction it could have a std::vector of acceptable directions.
// Or it can have `bool forward_okay` and `bool backward_okay` fields.
// DirectionConstraint class - implements orientation constraint based on robot's movement direction
class DirectionConstraint : public Graph::OrientationConstraint
{
public:

  // Static method to compute rotation matrix from forward direction vector
  // Converts a 2D direction vector into a rotation matrix representing that orientation
  static Eigen::Rotation2Dd compute_forward_offset(
    const Eigen::Vector2d& forward)
  {
    return Eigen::Rotation2Dd(std::atan2(forward[1], forward[0]));
  }

  // Static constant representing 180-degree rotation (π radians)
  // Used for backward direction transformations
  static const Eigen::Rotation2Dd R_pi;

  // Constructor for DirectionConstraint - initializes rotation matrices and direction
  // Takes the movement direction (forward/backward) and the forward vector reference
  DirectionConstraint(
    Direction _direction,                    // Movement direction (Forward or Backward)
    const Eigen::Vector2d& _forward_vector) // Reference direction vector for "forward"
  : R_f(compute_forward_offset(_forward_vector)),  // Compute rotation from forward vector
    R_f_inv(R_f.inverse()),                        // Store inverse rotation for efficiency
    direction(_direction)                          // Store the movement direction
  {
    // Do nothing - all initialization handled in member initializer list
  }

  Eigen::Rotation2Dd R_f;        // Rotation matrix representing the forward direction
  Eigen::Rotation2Dd R_f_inv;    // Inverse of R_f for computational efficiency
  Direction direction;           // Movement direction (Forward or Backward)

  // Compute the final rotation matrix based on course vector and direction
  // Combines course direction with forward/backward constraints
  Eigen::Rotation2Dd compute_R_final(
    const Eigen::Vector2d& course_vector) const
  {
    // Create rotation matrix from the course vector (desired movement direction)
    const Eigen::Rotation2Dd R_c(
      std::atan2(course_vector[1], course_vector[0]));

    // If moving backward, apply 180-degree rotation to reverse the direction
    if (Direction::Backward == direction)
      return R_pi * R_c * R_f_inv;

    // For forward movement, just apply the course rotation relative to forward direction
    return R_c * R_f_inv;
  }

  // Apply the direction constraint to a robot's position
  // Updates the yaw angle (position[2]) based on course vector and direction constraint
  bool apply(
    Eigen::Vector3d& position,              // Robot position [x, y, yaw] - yaw will be modified
    const Eigen::Vector2d& course_vector) const final // Desired movement direction
  {
    // Set yaw angle to the computed final rotation, wrapped to [-π, π] range
    position[2] = rmf_utils::wrap_to_pi(compute_R_final(course_vector).angle());
    return true;  // Always returns true as direction constraint can always be applied
  }

  // Create a deep copy of this direction constraint
  rmf_utils::clone_ptr<OrientationConstraint> clone() const final
  {
    return rmf_utils::make_clone<DirectionConstraint>(*this);
  }
};

//==============================================================================
// Static member definition - 180-degree rotation matrix for backward movement
const Eigen::Rotation2Dd DirectionConstraint::R_pi = Eigen::Rotation2Dd(M_PI);

} // anonymous namespace

//==============================================================================
// Factory method to create orientation constraint with acceptable orientations
// Returns a clone_ptr to AcceptableOrientationConstraint with given orientation angles
rmf_utils::clone_ptr<Graph::OrientationConstraint>
Graph::OrientationConstraint::make(std::vector<double> acceptable_orientations)
{
  return rmf_utils::make_clone<AcceptableOrientationConstraint>(
    std::move(acceptable_orientations));
}

//==============================================================================
// Factory method to create direction-based orientation constraint
// Returns a clone_ptr to DirectionConstraint with specified direction and forward vector
rmf_utils::clone_ptr<Graph::OrientationConstraint>
Graph::OrientationConstraint::make(
  Direction direction,              // Movement direction (Forward or Backward)
  const Eigen::Vector2d& forward)  // Reference direction vector for "forward"
{
  return rmf_utils::make_clone<DirectionConstraint>(direction, forward);
}

//==============================================================================
// Private implementation class for Lane::Door - stores door metadata and timing
class Graph::Lane::Door::Implementation
{
public:

  std::string name;     // Unique identifier for the door
  Duration duration;    // Time required to traverse this door

};

//==============================================================================
// Constructor for Lane::Door - creates a door with name and traversal duration
Graph::Lane::Door::Door(
  std::string name,     // Unique identifier for the door
  Duration duration)    // Time required to traverse this door
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(name),    // Move door name to avoid copying
        duration           // Copy duration value
      }))
{
  // Do nothing - all initialization handled in member initializer list
}

//==============================================================================
// Getter method to retrieve the door's unique identifier
const std::string& Graph::Lane::Door::name() const
{
  return _pimpl->name;  // Return const reference to door name
}

//==============================================================================
// Setter method to update the door's unique identifier
auto Graph::Lane::Door::name(std::string name) -> Door&
{
  _pimpl->name = std::move(name);  // Move new name to avoid copying
  return *this;  // Return reference to this door for method chaining
}

//==============================================================================
// Getter method to retrieve the door's traversal duration
Duration Graph::Lane::Door::duration() const
{
  return _pimpl->duration;  // Return copy of duration value
}

//==============================================================================
// Setter method to update the door's traversal duration
auto Graph::Lane::Door::duration(Duration duration_) -> Door&
{
  _pimpl->duration = duration_;  // Set new duration value
  return *this;  // Return reference to this door for method chaining
}

//==============================================================================
// Private implementation class for Lane::LiftSession - stores lift session metadata
class Graph::Lane::LiftSession::Implementation
{
public:

  std::string lift_name;   // Name of the lift/elevator
  std::string floor_name;  // Name of the destination floor
  Duration duration;       // Time required for the lift session

};

//==============================================================================
// Constructor for Lane::LiftSession - creates a lift session with metadata and duration
Graph::Lane::LiftSession::LiftSession(
  std::string lift_name,   // Name of the lift/elevator
  std::string floor_name,  // Name of the destination floor
  Duration duration)       // Time required for the lift session
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(lift_name),   // Move lift name to avoid copying
        std::move(floor_name),  // Move floor name to avoid copying
        duration               // Copy duration value
      }))
{
  // Do nothing - all initialization handled in member initializer list
}

//==============================================================================
// Getter method to retrieve the lift's name
const std::string& Graph::Lane::LiftSession::lift_name() const
{
  return _pimpl->lift_name;  // Return const reference to lift name
}

//==============================================================================
// Setter method to update the lift's name
auto Graph::Lane::LiftSession::lift_name(std::string name) -> LiftSession&
{
  _pimpl->lift_name = std::move(name);  // Move new name to avoid copying
  return *this;  // Return reference to this lift session for method chaining
}

//==============================================================================
// Getter method to retrieve the floor name
const std::string& Graph::Lane::LiftSession::floor_name() const
{
  return _pimpl->floor_name;  // Return const reference to floor name
}

//==============================================================================
// Setter method to update the floor name
auto Graph::Lane::LiftSession::floor_name(std::string name) -> LiftSession&
{
  _pimpl->floor_name = std::move(name);  // Move new name to avoid copying
  return *this;  // Return reference to this lift session for method chaining
}

//==============================================================================
// Getter method to retrieve the lift session duration
Duration Graph::Lane::LiftSession::duration() const
{
  return _pimpl->duration;  // Return copy of duration value
}

//==============================================================================
// Setter method to update the lift session duration
auto Graph::Lane::LiftSession::duration(Duration duration_) -> LiftSession&
{
  _pimpl->duration = duration_;  // Set new duration value
  return *this;  // Return reference to this lift session for method chaining
}

//==============================================================================
// Private implementation class for Lane::Dock - stores docking station metadata
class Graph::Lane::Dock::Implementation
{
public:

  std::string dock_name;  // Name of the docking station
  Duration duration;      // Time required for docking operation

};

//==============================================================================
// Constructor for Dock - creates a docking station event with specified name and duration
Graph::Lane::Dock::Dock(
  std::string dock_name,  // Name of the docking station
  Duration duration)      // Time required for docking operation
: _pimpl(rmf_utils::make_impl<Implementation>(  // Initialize private implementation using pimpl pattern
      Implementation{
        std::move(dock_name),  // Move dock name to avoid copying
        duration               // Copy duration value
      }))
{
  // Do nothing - all initialization handled in member initializer list
}

//==============================================================================
// Getter method to retrieve the docking station name
const std::string& Graph::Lane::Dock::dock_name() const
{
  return _pimpl->dock_name;  // Return const reference to dock name
}

//==============================================================================
// Setter method to update the docking station name
auto Graph::Lane::Dock::dock_name(std::string name) -> Dock&
{
  _pimpl->dock_name = name;  // Set new dock name
  return *this;  // Return reference to this dock for method chaining
}

//==============================================================================
// Getter method to retrieve the docking operation duration
Duration Graph::Lane::Dock::duration() const
{
  return _pimpl->duration;  // Return copy of duration value
}

//==============================================================================
// Setter method to update the docking operation duration
auto Graph::Lane::Dock::duration(Duration d) -> Dock&
{
  _pimpl->duration = d;  // Set new duration value
  return *this;  // Return reference to this dock for method chaining
}

//==============================================================================
// Private implementation class for Wait - stores wait event duration
class Graph::Lane::Wait::Implementation
{
public:

  Duration duration;  // Time duration for the wait event

};

//==============================================================================
// Constructor for Wait - creates a wait event with specified duration
Graph::Lane::Wait::Wait(Duration value)  // Duration to wait
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{value}))  // Initialize implementation with duration
{
  // Do nothing - all initialization handled in member initializer list
}

//==============================================================================
// Getter method to retrieve the wait duration
Duration Graph::Lane::Wait::duration() const
{
  return _pimpl->duration;  // Return copy of duration value
}

//==============================================================================
// Setter method to update the wait duration
auto Graph::Lane::Wait::duration(Duration value) -> Wait&
{
  _pimpl->duration = value;  // Set new duration value
  return *this;  // Return reference to this wait event for method chaining
}

//==============================================================================
// Execute method for Wait event - performs no operation during execution
void Graph::Lane::Executor::execute(const Wait&)
{
  // Do nothing - wait events require no execution logic
}

namespace {
//==============================================================================
// Template class for wrapping lane events - provides common event interface
template<typename EventT>
class TemplateEvent : public Graph::Lane::Event
{
public:

  using This = TemplateEvent<EventT>;  // Type alias for this template instantiation

  // Constructor for TemplateEvent - wraps the provided event
  TemplateEvent(EventT event)  // Event to wrap
  : _event(std::move(event))   // Move event to avoid copying
  {
    // Do nothing - initialization handled in member initializer list
  }

  // Get the duration of the wrapped event
  Duration duration() const final
  {
    return _event.duration();  // Delegate to wrapped event's duration method
  }

  // Execute the wrapped event using the provided executor
  Graph::Lane::Executor& execute(Graph::Lane::Executor& executor) const final
  {
    executor.execute(_event);  // Delegate execution to the wrapped event
    return executor;  // Return executor for method chaining
  }

  // Static factory method to create a new TemplateEvent instance
  static Graph::Lane::EventPtr make(EventT _event)  // Event to wrap
  {
    return rmf_utils::make_clone<This>(std::move(_event));  // Create cloned instance
  }

  // Create a deep copy of this template event
  Graph::Lane::EventPtr clone() const final
  {
    return make(_event);  // Create new instance with same wrapped event
  }

private:

  EventT _event;  // The wrapped event instance

};  // End of TemplateEvent class definition
} // anonymous namespace  // End of anonymous namespace containing TemplateEvent

//==============================================================================
// Factory method to create a DoorOpen event wrapped in TemplateEvent
auto Graph::Lane::Event::make(DoorOpen open) -> EventPtr
{
  return TemplateEvent<DoorOpen>::make(std::move(open));  // Create TemplateEvent wrapper for DoorOpen event
}

//==============================================================================
// Factory method to create a DoorClose event wrapped in TemplateEvent
auto Graph::Lane::Event::make(DoorClose close) -> EventPtr
{
  return TemplateEvent<DoorClose>::make(std::move(close));  // Create TemplateEvent wrapper for DoorClose event
}

//==============================================================================
// Factory method to create a LiftSessionBegin event wrapped in TemplateEvent
auto Graph::Lane::Event::make(LiftSessionBegin open) -> EventPtr
{
  return TemplateEvent<LiftSessionBegin>::make(std::move(open));  // Create TemplateEvent wrapper for LiftSessionBegin event
}

//==============================================================================
// Factory method to create a LiftSessionEnd event wrapped in TemplateEvent
auto Graph::Lane::Event::make(LiftSessionEnd close) -> EventPtr
{
  return TemplateEvent<LiftSessionEnd>::make(std::move(close));  // Create TemplateEvent wrapper for LiftSessionEnd event
}

//==============================================================================
// Factory method to create a LiftMove event wrapped in TemplateEvent
auto Graph::Lane::Event::make(LiftMove move) -> EventPtr
{
  return TemplateEvent<LiftMove>::make(std::move(move));  // Create TemplateEvent wrapper for LiftMove event
}

//==============================================================================
// Factory method to create a LiftDoorOpen event wrapped in TemplateEvent
auto Graph::Lane::Event::make(LiftDoorOpen open) -> EventPtr
{
  return TemplateEvent<LiftDoorOpen>::make(std::move(open));  // Create TemplateEvent wrapper for LiftDoorOpen event
}

//==============================================================================
// Factory method to create a Dock event wrapped in TemplateEvent
auto Graph::Lane::Event::make(Dock dock) -> EventPtr
{
  return TemplateEvent<Dock>::make(std::move(dock));  // Create TemplateEvent wrapper for Dock event
}

//==============================================================================
// Factory method to create a Wait event wrapped in TemplateEvent
auto Graph::Lane::Event::make(Wait wait) -> EventPtr
{
  return TemplateEvent<Wait>::make(std::move(wait));  // Create TemplateEvent wrapper for Wait event
}

//==============================================================================
// Private implementation class for Graph::Lane::Node - stores node data and behavior
class Graph::Lane::Node::Implementation
{
public:

  std::size_t waypoint;  // Index of the waypoint this node represents in the graph

  rmf_utils::clone_ptr<Event> _event;  // Optional event to execute when reaching this node

  rmf_utils::clone_ptr<OrientationConstraint> _orientation;  // Optional orientation constraint for this node

};

//==============================================================================
// Constructor for Graph::Lane::Node with waypoint, event, and orientation constraint
Graph::Lane::Node::Node(
  std::size_t waypoint_index,  // Index of the waypoint in the graph
  rmf_utils::clone_ptr<Event> event,  // Event to execute at this node (can be null)
  rmf_utils::clone_ptr<OrientationConstraint> orientation)  // Orientation constraint for this node
: _pimpl(rmf_utils::make_impl<Implementation>(  // Initialize private implementation using pimpl pattern
      Implementation{  // Create implementation struct with provided parameters
        waypoint_index,  // Store waypoint index
        std::move(event),  // Move event to avoid copying
        std::move(orientation)  // Move orientation constraint to avoid copying
      }))
{
  // Do nothing  // All initialization handled in member initializer list
}

//==============================================================================
// Constructor for Graph::Lane::Node with waypoint and orientation constraint only (no event)
Graph::Lane::Node::Node(
  std::size_t waypoint_index,  // Index of the waypoint in the graph
  rmf_utils::clone_ptr<OrientationConstraint> orientation)  // Orientation constraint for this node
: _pimpl(rmf_utils::make_impl<Implementation>(  // Initialize private implementation using pimpl pattern
      Implementation{  // Create implementation struct with provided parameters
        waypoint_index,  // Store waypoint index
        nullptr,  // No event associated with this node
        std::move(orientation)  // Move orientation constraint to avoid copying
      }))
{
  // Do nothing  // All initialization handled in member initializer list
}

//==============================================================================
// Getter method to retrieve the waypoint index associated with this node
std::size_t Graph::Lane::Node::waypoint_index() const
{
  return _pimpl->waypoint;  // Return the waypoint index from private implementation
}

//==============================================================================
// Getter method to retrieve the event associated with this node (can be null)
auto Graph::Lane::Node::event() const -> const Event*
{
  return _pimpl->_event.get();  // Return raw pointer to the event (nullptr if no event)
}

//==============================================================================
// Setter method to update the event associated with this node
Graph::Lane::Node& Graph::Lane::Node::event(
  rmf_utils::clone_ptr<Event> new_event)  // New event to associate with this node
{
  _pimpl->_event = std::move(new_event);  // Move the new event to avoid copying
  return *this;  // Return reference to this node for method chaining
}

//==============================================================================
// Getter method to retrieve the orientation constraint associated with this node (can be null)
auto Graph::Lane::Node::orientation_constraint() const
-> const OrientationConstraint*
{
  return _pimpl->_orientation.get();  // Return raw pointer to the orientation constraint (nullptr if no constraint)
}

//==============================================================================
// Private implementation class for Graph::Lane::Properties - stores lane properties and behavior
class Graph::Lane::Properties::Implementation
{
public:

  std::optional<double> speed_limit;  // Optional speed limit for this lane (nullopt means no limit)

  std::string mutex_group;  // Name of the mutex group this lane belongs to (empty string means no group)

};

//==============================================================================
// Constructor for Lane::Properties - initializes the properties object with default values
Graph::Lane::Properties::Properties()
: _pimpl(rmf_utils::make_impl<Implementation>())  // Create private implementation using pimpl pattern
{
  // Do nothing - all initialization handled in member initializer list
}

//==============================================================================
// Getter method to retrieve the speed limit for this lane
// Returns optional double - nullopt means no speed limit is set
std::optional<double> Graph::Lane::Properties::speed_limit() const
{
  return _pimpl->speed_limit;  // Return copy of speed limit from private implementation
}

//==============================================================================
// Setter method to update the speed limit for this lane
// Takes optional double value and returns reference to this properties object for method chaining
auto Graph::Lane::Properties::speed_limit(std::optional<double> value)
-> Properties&
{
  _pimpl->speed_limit = value;  // Set new speed limit value in private implementation
  return *this;  // Return reference to this properties object for method chaining
}

//==============================================================================
// Getter method to retrieve the mutex group name for this lane
// Returns const reference to the mutex group string (empty string means no group)
const std::string& Graph::Lane::Properties::in_mutex_group() const
{
  return _pimpl->mutex_group;  // Return const reference to mutex group name from private implementation
}

//==============================================================================
// Setter method to update the mutex group name for this lane
// Takes group name string and returns reference to this properties object for method chaining
auto Graph::Lane::Properties::set_in_mutex_group(std::string group_name)
-> Properties&
{
  _pimpl->mutex_group = std::move(group_name);  // Move new group name to avoid copying
  return *this;  // Return reference to this properties object for method chaining
}

//==============================================================================
// Private implementation class for Graph::Lane - contains all lane data and behavior
class Graph::Lane::Implementation
{
public:

  std::size_t index;  // Unique numerical identifier for this lane in the graph

  Node entry;  // Entry node representing the starting point of this lane

  Node exit;  // Exit node representing the ending point of this lane

  Properties properties;  // Lane properties including speed limits and mutex groups

  // Factory method to create a Lane with variadic arguments
  // Uses perfect forwarding to construct the lane with any number of arguments
  template<typename... Args>
  static Lane make(Args&& ... args)
  {
    Lane lane;  // Create empty lane object
    lane._pimpl = rmf_utils::make_impl<Implementation>(  // Initialize implementation with forwarded arguments
      Implementation{std::forward<Args>(args)...});

    return lane;  // Return the constructed lane
  }
};

//==============================================================================
// Getter method to retrieve the entry node of this lane (non-const version)
// Returns reference to the entry node for modification
auto Graph::Lane::entry() -> Node&
{
  return _pimpl->entry;  // Return reference to entry node from private implementation
}

//==============================================================================
// Getter method to retrieve the entry node of this lane (const version)
// Returns const reference to the entry node for read-only access
auto Graph::Lane::entry() const -> const Node&
{
  return _pimpl->entry;  // Return const reference to entry node from private implementation
}

//==============================================================================
// Getter method to retrieve the exit node of this lane (non-const version)
// Returns reference to the exit node for modification
auto Graph::Lane::exit() -> Node&
{
  return _pimpl->exit;  // Return reference to exit node from private implementation
}

//==============================================================================
// Getter method to retrieve the exit node of this lane (const version)
// Returns const reference to the exit node for read-only access
auto Graph::Lane::exit() const -> const Node&
{
  return _pimpl->exit;  // Return const reference to exit node from private implementation
}

//==============================================================================
// Getter method to retrieve the properties of this lane (non-const version)
// Returns reference to the properties object for modification
auto Graph::Lane::properties() -> Properties&
{
  return _pimpl->properties;  // Return reference to properties from private implementation
}

//==============================================================================
// Getter method to retrieve the properties of this lane (const version)
// Returns const reference to the properties object for read-only access
auto Graph::Lane::properties() const -> const Properties&
{
  return _pimpl->properties;  // Return const reference to properties from private implementation
}

//==============================================================================
std::size_t Graph::Lane::index() const
{
  return _pimpl->index;
}

//==============================================================================
// Default constructor for Lane class - creates an empty lane with no implementation
Graph::Lane::Lane()
{
  // Do nothing - default constructor creates empty lane
}

//==============================================================================
// Constructor for Graph class - initializes the private implementation
Graph::Graph()
: _pimpl(rmf_utils::make_impl<Implementation>())  // Create new implementation using rmf_utils factory
{
}

//==============================================================================
// Add a new waypoint to the graph with specified map name and 2D location
auto Graph::add_waypoint(
  std::string map_name,        // Name of the map this waypoint belongs to
  Eigen::Vector2d location) -> Waypoint&  // 2D coordinates of the waypoint
{
  // Create new waypoint using factory method and add to waypoints vector
  _pimpl->waypoints.emplace_back(
    Waypoint::Implementation::make(
      _pimpl->waypoints.size(),  // Use current size as index
      std::move(map_name),       // Move map name to avoid copying
      std::move(location)));     // Move location to avoid copying

  // Initialize adjacency data structures for the new waypoint
  _pimpl->lanes_from.push_back({});    // Empty vector for outgoing lanes
  _pimpl->lanes_into.push_back({});    // Empty vector for incoming lanes
  _pimpl->lane_between.push_back({});  // Empty map for direct connections

  // Return reference to the newly created waypoint
  return _pimpl->waypoints.back();
}

//==============================================================================
// Get a waypoint by its index (non-const version for modification)
auto Graph::get_waypoint(const std::size_t index) -> Waypoint&
{
  // Debug output commented out for performance
 // std::cout << "[Graph.cpp]  GETTING WAYPOINT: index " << index << std::endl;
  // Return reference to waypoint at specified index (throws if out of bounds)
  return _pimpl->waypoints.at(index);
}

//==============================================================================
// Get a waypoint by its index (const version for read-only access)
auto Graph::get_waypoint(const std::size_t index) const -> const Waypoint&
{
  // Return const reference to waypoint at specified index (throws if out of bounds)
  return _pimpl->waypoints.at(index);
}

//==============================================================================
// Find a waypoint by its string key/name
auto Graph::find_waypoint(const std::string& key) -> Waypoint*
{
  // Search for the key in the keys map
  const auto it = _pimpl->keys.find(key);
  // If key not found, return null pointer
  if (it == _pimpl->keys.end())
    return nullptr;

  // Return pointer to the waypoint associated with the key
  return &get_waypoint(it->second);
}

//==============================================================================
// Find a waypoint by its string key/name (const version)
auto Graph::find_waypoint(const std::string& key) const -> const Waypoint*
{
  // Use const_cast to call non-const version and return const pointer
  return const_cast<Graph&>(*this).find_waypoint(key);
}

//==============================================================================
// Add a string key/name to associate with a waypoint
bool Graph::add_key(const std::string& key, std::size_t wp_index)
{
  // Check if waypoint index is valid (not beyond current waypoints)
  if (wp_index > _pimpl->waypoints.size())
    return false;

  // Try to insert the key-index pair into the keys map
  const auto inserted = _pimpl->keys.insert({key, wp_index}).second;
  // If key already exists, insertion fails
  if (!inserted)
    return false;

  // Set the name field of the waypoint to the key
  Waypoint::Implementation::get(_pimpl->waypoints.at(wp_index)).name = key;
  return true;
}

//==============================================================================
// Remove a string key/name from a waypoint
bool Graph::remove_key(const std::string& key)
{
  // Find the key in the keys map
  const auto it = _pimpl->keys.find(key);
  // If key not found, return false
  if (it == _pimpl->keys.end())
    return false;

  // Clear the name field of the associated waypoint
  Waypoint::Implementation::get(_pimpl->waypoints.at(it->second))
  .name = rmf_utils::nullopt;

  // Remove the key from the keys map
  _pimpl->keys.erase(it);
  return true;
}

//==============================================================================
// Set or update a string key/name for a waypoint (overwrites existing)
bool Graph::set_key(const std::string& key, std::size_t wp_index)
{
  // Check if waypoint index is valid
  if (_pimpl->waypoints.size() <= wp_index)
    return false;

  // Insert or update the key-index mapping
  _pimpl->keys[key] = wp_index;
  // Try to insert the key-index pair
  const auto insertion = _pimpl->keys.insert({key, wp_index});
  // If key already existed, update the old waypoint's name
  if (!insertion.second)
  {
    // Clear name of waypoint that previously had this key
    Waypoint::Implementation::get(
      _pimpl->waypoints.at(insertion.first->second)).name = rmf_utils::nullopt;
    // Update the mapping to point to new waypoint
    insertion.first->second = wp_index;
  }

  // Set the name field of the target waypoint
  Waypoint::Implementation::get(_pimpl->waypoints.at(wp_index)).name = key;
  return true;
}

//==============================================================================
// Get all string keys and their associated waypoint indices
const std::unordered_map<std::string, std::size_t>& Graph::keys() const
{
  // Return const reference to the keys map
  return _pimpl->keys;
}

//==============================================================================
// Get the total number of waypoints in the graph
std::size_t Graph::num_waypoints() const
{
  // Return the size of the waypoints vector
  return _pimpl->waypoints.size();
}

//==============================================================================
// Add a new lane connecting two waypoints with specified properties
auto Graph::add_lane(
  const Lane::Node& entry,           // Entry node (source waypoint)
  const Lane::Node& exit,            // Exit node (destination waypoint)
  Lane::Properties properties) -> Lane&  // Lane properties (speed limits, etc.)
{
  // Assert that both waypoint indices are valid
  assert(entry.waypoint_index() < _pimpl->waypoints.size());
  assert(exit.waypoint_index() < _pimpl->waypoints.size());

  // Get the lane ID (current number of lanes)
  const std::size_t lane_id = _pimpl->lanes.size();
  // Get the entry waypoint index
  const std::size_t entry_index = entry.waypoint_index();
  // Add this lane to the outgoing lanes of the entry waypoint
  _pimpl->lanes_from.at(entry_index).push_back(lane_id);
  // Add this lane to the incoming lanes of the exit waypoint
  _pimpl->lanes_into.at(exit.waypoint_index()).push_back(lane_id);
  // Record direct connection between entry and exit waypoints
  _pimpl->lane_between.at(entry_index)[exit.waypoint_index()] = lane_id;

  // Create new lane using factory method and add to lanes vector
  _pimpl->lanes.emplace_back(
    Lane::Implementation::make(
      _pimpl->lanes.size(),    // Use current size as lane index
      std::move(entry),        // Move entry node to avoid copying
      std::move(exit),         // Move exit node to avoid copying
      std::move(properties))); // Move properties to avoid copying

  // Return reference to the newly created lane
  return _pimpl->lanes.back();
}

//==============================================================================
// Get a lane by its index (non-const version for modification)
auto Graph::get_lane(const std::size_t index) -> Lane&
{
  // Debug output commented out for performance
 // std::cout << "[Graph.cpp] 🛣️ GETTING LANE: index " << index << std::endl;
  return _pimpl->lanes.at(index);
}

//==============================================================================
auto Graph::get_lane(const std::size_t index) const -> const Lane&
{
  return _pimpl->lanes.at(index);
}

//==============================================================================
std::size_t Graph::num_lanes() const
{
  return _pimpl->lanes.size();
}

//==============================================================================
const std::vector<std::size_t>& Graph::lanes_from(std::size_t wp_index) const
{
  return _pimpl->lanes_from.at(wp_index);
}

//==============================================================================
const std::vector<std::size_t>& Graph::lanes_into(std::size_t wp_index) const
{
  return _pimpl->lanes_into.at(wp_index);
}

//==============================================================================
auto Graph::lane_from(std::size_t from_wp, std::size_t to_wp) -> Lane*
{
  const auto& lanes = _pimpl->lane_between.at(from_wp);
  const auto it = lanes.find(to_wp);
  if (it == lanes.end())
    return nullptr;

  return &_pimpl->lanes.at(it->second);
}

//==============================================================================
auto Graph::lane_from(std::size_t from_wp, std::size_t to_wp) const
-> const Lane*
{
  return const_cast<Graph&>(*this).lane_from(from_wp, to_wp);
}

//==============================================================================
auto Graph::set_known_lift(LiftProperties lift) -> LiftPropertiesPtr
{
  const auto [l_it, inserted] = _pimpl->lifts.insert({lift.name(), nullptr});
  if (inserted)
  {
    l_it->second = std::make_shared<LiftProperties>(std::move(lift));
  }
  else
  {
    *l_it->second = std::move(lift);
  }

  return l_it->second;
}

//==============================================================================
auto Graph::all_known_lifts() const -> std::vector<LiftPropertiesPtr>
{
  std::vector<LiftPropertiesPtr> lifts;
  lifts.reserve(_pimpl->lifts.size());
  for (const auto& [_, lift] : _pimpl->lifts)
  {
    lifts.push_back(lift);
  }

  return lifts;
}

//==============================================================================
auto Graph::find_known_lift(const std::string& name) const -> LiftPropertiesPtr
{
  const auto l_it = _pimpl->lifts.find(name);
  if (l_it == _pimpl->lifts.end())
    return nullptr;

  return l_it->second;
}

//==============================================================================
auto Graph::set_known_door(DoorProperties door) -> DoorPropertiesPtr
{
  const auto [d_it, inserted] = _pimpl->doors.insert({door.name(), nullptr});
  if (inserted)
  {
    d_it->second = std::make_shared<DoorProperties>(std::move(door));
  }
  else
  {
    *d_it->second = std::move(door);
  }

  return d_it->second;
}

//==============================================================================
auto Graph::all_known_doors() const -> std::vector<DoorPropertiesPtr>
{
  std::vector<DoorPropertiesPtr> doors;
  doors.reserve(_pimpl->doors.size());
  for (const auto& [_, door] : _pimpl->doors)
  {
    doors.push_back(door);
  }

  return doors;
}

//==============================================================================
auto Graph::find_known_door(const std::string& name) const -> DoorPropertiesPtr
{
  const auto d_it = _pimpl->doors.find(name);
  if (d_it == _pimpl->doors.end())
    return nullptr;

  return d_it->second;
}

} // namespace avg
} // namespace rmf_traffic
