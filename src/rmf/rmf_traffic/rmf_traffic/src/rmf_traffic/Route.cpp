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

// Include the internal implementation header that contains the actual data structures
// This header defines the Route::Implementation class with all the private members
#include "internal_Route.hpp"

// Include utility for modular arithmetic operations (used for plan ID comparisons)
// Plan IDs are sequential numbers that wrap around, so we need modular arithmetic
#include <rmf_utils/Modular.hpp>
  //  //// Include iostream for logging functionality (std::cout and std::endl)
#include <iostream>

// All code in this file belongs to the rmf_traffic namespace
// This namespace contains all traffic management related classes and functions
namespace rmf_traffic {

//==============================================================================
// DEPENDENCY EQUALITY COMPARISON OPERATOR
// This function compares two Dependency objects to see if they represent the same dependency
// Used by the traffic system to check if two routes have identical dependencies
bool Dependency::operator==(const Dependency& other) const
{
  //  //  //std::cout << "[Dependency::operator==] ⚖️ Comparing dependencies" << std::endl;
  //  //  //std::cout << "[Dependency::operator==] 📋 This: participant=" << on_participant 
  //            << " plan=" << on_plan << " route=" << on_route 
  //            << " checkpoint=" << on_checkpoint << std::endl;
  //  //  //std::cout << "[Dependency::operator==] 📋 Other: participant=" << other.on_participant 
  //            << " plan=" << other.on_plan << " route=" << other.on_route 
  //            << " checkpoint=" << other.on_checkpoint << std::endl;
  
  // Two dependencies are considered equal if ALL these fields match exactly:
  bool equal = on_participant == other.on_participant  // Same robot/participant ID
      && on_plan == other.on_plan                // Same plan ID (plans are numbered sequentially)
      && on_route == other.on_route              // Same route ID within the plan
      && on_checkpoint == other.on_checkpoint;   // Same checkpoint ID within the route
  
  //  //  //std::cout << "[Dependency::operator==] ⚖️ Comparing dependencies" << std::endl;
  return equal;
}

//==============================================================================
// DEPENDS_ON_PLAN IMPLEMENTATION CLASS
// This is the internal data structure for DependsOnPlan using PIMPL pattern
// PIMPL (Pointer to Implementation) hides implementation details from the header
class DependsOnPlan::Implementation
{
public:
  // The plan ID that this dependency refers to (optional - might not be set initially)
  // Plan IDs are sequential numbers that identify different planning attempts
  std::optional<PlanId> plan;
  
  // Map of route dependencies: RouteId -> CheckpointId -> DependentCheckpointId
  // This stores which checkpoints in our route depend on which checkpoints in other routes
  DependsOnRoute routes;
};

//==============================================================================
// DEPENDS_ON_PLAN DEFAULT CONSTRUCTOR
// Creates an empty DependsOnPlan with no dependencies set
// Used when initializing a new dependency structure
DependsOnPlan::DependsOnPlan()
: _pimpl(rmf_utils::make_impl<Implementation>())  // Create empty implementation object
{
  //  //  //std::cout << "[DependsOnPlan::DependsOnPlan] 🏗️ Default constructor called - creating empty DependsOnPlan" << std::endl;
  // Do nothing - all fields are already initialized to default values
  // plan will be std::nullopt, routes will be empty map
}

//==============================================================================
// DEPENDS_ON_PLAN PARAMETERIZED CONSTRUCTOR
// Creates a DependsOnPlan with specific plan ID and route dependencies
// Used when we already know what this dependency should contain
DependsOnPlan::DependsOnPlan(PlanId plan, DependsOnRoute routes)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{plan, std::move(routes)}))  // Move routes to avoid expensive copying
{
  //  //  //std::cout << "[DependsOnPlan::DependsOnPlan] ️ Parameterized constructor called" << std::endl;
  //  //  //std::cout << "[DependsOnPlan::DependsOnPlan] 📋 Plan ID: " << plan << std::endl;
  //  //  //std::cout << "[DependsOnPlan::DependsOnPlan] 📋 Routes count: " << routes.size() << std::endl;
  // Do nothing - implementation is initialized with provided values
}

//==============================================================================
// SET PLAN ID
// Sets the plan ID that this dependency refers to
// Plan IDs are sequential numbers - higher numbers are newer plans
DependsOnPlan& DependsOnPlan::plan(std::optional<PlanId> plan)
{
  //  //  //std::cout << "[DependsOnPlan::plan] 🔧 Setting plan ID" << std::endl;
  //  //  //std::cout << "[DependsOnPlan::plan]  Old plan: " << (_pimpl->plan.has_value() ? std::to_string(*_pimpl->plan) : "nullopt") << std::endl;
  //  //  //std::cout << "[DependsOnPlan::plan]  New plan: " << (plan.has_value() ? std::to_string(*plan) : "nullopt") << std::endl;
  
  _pimpl->plan = plan;  // Store the plan ID in the implementation
  return *this;         // Return reference for method chaining (fluent interface)
}

//==============================================================================
// GET PLAN ID
// Returns the plan ID that this dependency refers to
// Returns std::nullopt if no plan ID has been set yet
std::optional<PlanId> DependsOnPlan::plan() const
{
  //  //  //std::cout << "[DependsOnPlan::plan] 📖 Getting plan ID" << std::endl;
  //  //  //std::cout << "[DependsOnPlan::plan]  Plan ID: " << (_pimpl->plan.has_value() ? std::to_string(*_pimpl->plan) : "nullopt") << std::endl;
  return _pimpl->plan;  // Return the stored plan ID (might be empty)
}

//==============================================================================
// SET ROUTE DEPENDENCIES
// Sets the route dependencies for this plan
// This replaces any existing route dependencies
DependsOnPlan& DependsOnPlan::routes(DependsOnRoute routes)
{
  //  //  //std::cout << "[DependsOnPlan::routes] 🔧 Setting route dependencies" << std::endl;
  //  //  //std::cout << "[DependsOnPlan::routes]  Old routes count: " << _pimpl->routes.size() << std::endl;
  //  //  //std::cout << "[DependsOnPlan::routes] 📋 New routes count: " << routes.size() << std::endl;
  
  _pimpl->routes = std::move(routes);  // Move routes to avoid expensive copying
  return *this;                        // Return reference for method chaining
}

//==============================================================================
// GET ROUTE DEPENDENCIES (MUTABLE VERSION)
// Returns a reference to the route dependencies (allows modification)
// Used when we need to add or modify dependencies
DependsOnRoute& DependsOnPlan::routes()
{
  //  //  //std::cout << "[DependsOnPlan::routes] 📖 Getting mutable route dependencies" << std::endl;
  //  //  //std::cout << "[DependsOnPlan::routes] 📋 Routes count: " << _pimpl->routes.size() << std::endl;
  return _pimpl->routes;  // Return reference to route dependencies map
}

//==============================================================================
// GET ROUTE DEPENDENCIES (CONST VERSION)
// Returns a const reference to the route dependencies (read-only)
// Used when we only need to read the dependencies
const DependsOnRoute& DependsOnPlan::routes() const
{
  //  //  //std::cout << "[DependsOnPlan::routes] 📖 Getting const route dependencies" << std::endl;
  //  //  //std::cout << "[DependsOnPlan::routes] 📋 Routes count: " << _pimpl->routes.size() << std::endl;
  return _pimpl->routes;  // Return const reference to route dependencies map
}

//==============================================================================
// ADD DEPENDENCY TO PLAN
// Adds a new dependency between checkpoints of different routes
// This is the core function for establishing robot coordination
DependsOnPlan& DependsOnPlan::add_dependency(
  const CheckpointId dependent_checkpoint,  // Checkpoint in our route that must wait
  const Dependency dep)                     // Details of what we're waiting for
{
  //  //  //std::cout << "[DependsOnPlan::add_dependency] 🔗 Adding dependency to plan" << std::endl;
  //  //  //std::cout << "[DependsOnPlan::add_dependency]  Dependent checkpoint: " << dependent_checkpoint << std::endl;
  //  //  //std::cout << "[DependsOnPlan::add_dependency] 📋 Dependency: route=" << dep.on_route 
  //            << " checkpoint=" << dep.on_checkpoint << std::endl;
  
  // Try to insert the dependency into the route's checkpoint map
  // The map structure is: RouteId -> (CheckpointId -> DependentCheckpointId)
  const auto insertion = _pimpl->routes[dep.on_route]
    .insert({dep.on_checkpoint, dependent_checkpoint});

  //  //  //std::cout << "[DependsOnPlan::add_dependency]  Insertion result: " << (insertion.second ? "NEW" : "EXISTING") << std::endl;
  
  // Check if insertion was successful (new dependency) or if it already existed
  if (!insertion.second)
  {
  //  //  //std::cout << "[DependsOnPlan::add_dependency] ⚠️ Dependency already exists, checking for earlier checkpoint" << std::endl;
    // If the dependent checkpoint already has a dependency on this route, then
    // we should check if the new other_checkpoint is larger than the one that
    // already there.
    auto& prior_checkpoint = insertion.first->second;
  //  //  //std::cout << "[DependsOnPlan::add_dependency] 📋 Prior checkpoint: " << prior_checkpoint << std::endl;
    
    // Keep the earlier checkpoint (smaller ID) as the dependency
    // This ensures we wait for the earliest possible checkpoint
    if (dependent_checkpoint < prior_checkpoint)
    {
  //  //  //std::cout << "[DependsOnPlan::add_dependency]  Updating to earlier checkpoint: " << dependent_checkpoint << std::endl;
      prior_checkpoint = dependent_checkpoint;  // Update to earlier checkpoint
    }
    else
    {
  //  //  //std::cout << "[DependsOnPlan::add_dependency] ✅ Keeping existing checkpoint: " << prior_checkpoint << std::endl;
    }
  }
  else
  {
  //  //  //std::cout << "[DependsOnPlan::add_dependency] ✅ New dependency added successfully" << std::endl;
  }

  return *this;  // Return reference for method chaining
}

//==============================================================================
// ROUTE CONSTRUCTOR
// Creates a new Route object with map name and trajectory
// This is the main constructor that creates a route for robot navigation
Route::Route(
  std::string map,        // Name of the map/level (e.g., "L1", "L2", "warehouse_floor_1")
  Trajectory trajectory)  // The actual path with timing information (x, y, time)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(map),           // Move map name to avoid copying
        std::move(trajectory),    // Move trajectory to avoid copying
        {},                       // Empty checkpoints set (will be set later)
        {}                        // Empty dependencies map (will be set later)
      }))
{
  //  //  //std::cout << "[Route::Route] 🏗️ Constructor called - creating new Route" << std::endl;
  //  //  //std::cout << "[Route::Route] 🗺️ Map: " << _pimpl->map << std::endl;
  //  //  //std::cout << "[Route::Route]  Trajectory size: " << _pimpl->trajectory.size() << std::endl;
  //  //  //std::cout << "[Route::Route]  Checkpoints: " << _pimpl->checkpoints.size() << std::endl;
  //  //  //std::cout << "[Route::Route] 🔗 Dependencies: " << _pimpl->dependencies.size() << std::endl;
}

//==============================================================================
// SET MAP NAME
// Changes the map name for this route
// Used when a route needs to be moved to a different map/level
Route& Route::map(std::string value)
{
  //  //  //std::cout << "[Route::map]  Setting map name" << std::endl;
  //  //  //std::cout << "[Route::map] 🗺️ Old map: " << _pimpl->map << std::endl;
  //  //  //std::cout << "[Route::map] 🗺️ New map: " << value << std::endl;
  
  _pimpl->map = std::move(value);  // Move new map name to avoid copying
  return *this;                    // Return reference for method chaining
}

//==============================================================================
// GET MAP NAME
// Returns the map name for this route
// Used to identify which building level/area this route is on
const std::string& Route::map() const
{
  //  //  //std::cout << "[Route::map]  Getting map name" << std::endl;
  //  //  //std::cout << "[Route::map] 🗺️ Map: " << _pimpl->map << std::endl;
  return _pimpl->map;  // Return reference to map name
}

//==============================================================================
// SET TRAJECTORY
// Changes the trajectory for this route
// Used when a route needs to be updated with a new path
Route& Route::trajectory(Trajectory value)
{
  //  //  //std::cout << "[Route::trajectory] 🔧 Setting trajectory" << std::endl;
  //  //  //std::cout << "[Route::trajectory]  Old trajectory size: " << _pimpl->trajectory.size() << std::endl;
  //  //  //std::cout << "[Route::trajectory]  New trajectory size: " << value.size() << std::endl;
  
  _pimpl->trajectory = std::move(value);  // Move new trajectory to avoid copying
  return *this;                           // Return reference for method chaining
}

//==============================================================================
// GET TRAJECTORY (MUTABLE VERSION)
// Returns a reference to the trajectory (allows modification)
// Used when we need to modify the path points or timing
Trajectory& Route::trajectory()
{
  //  //  //std::cout << "[Route::trajectory] 📖 Getting mutable trajectory" << std::endl;
  //  //  //std::cout << "[Route::trajectory]  Trajectory size: " << _pimpl->trajectory.size() << std::endl;
  return _pimpl->trajectory;  // Return reference to trajectory
}

//==============================================================================
// GET TRAJECTORY (CONST VERSION)
// Returns a const reference to the trajectory (read-only)
// Used when we only need to read the path information
const Trajectory& Route::trajectory() const
{
  //  //  //std::cout << "[Route::trajectory] 📖 Getting const trajectory" << std::endl;
  //  //  //std::cout << "[Route::trajectory]  Trajectory size: " << _pimpl->trajectory.size() << std::endl;
  return _pimpl->trajectory;  // Return const reference to trajectory
}

//==============================================================================
// SET CHECKPOINTS
// Sets the checkpoints for this route (important waypoints for coordination)
// Checkpoints are specific points along the route where robots need to coordinate
Route& Route::checkpoints(std::set<uint64_t> value)
{
  //  //  //std::cout << "[Route::checkpoints] 🔧 Setting checkpoints" << std::endl;
  //  //  //std::cout << "[Route::checkpoints] 📍 Old checkpoints count: " << _pimpl->checkpoints.size() << std::endl;
  //  //  //std::cout << "[Route::checkpoints] 📍 New checkpoints count: " << value.size() << std::endl;
  
  if (!value.empty()) {
  //  //  //std::cout << "[Route::checkpoints]  New checkpoints: ";
    for (const auto& cp : value) {
  //  //      std::cout << cp << " ";
    }
  //  //    std::cout << std::endl;
  }
  
  _pimpl->checkpoints = std::move(value);  // Move checkpoints to avoid copying
  return *this;                            // Return reference for method chaining
}

//==============================================================================
// GET CHECKPOINTS (MUTABLE VERSION)
// Returns a reference to the checkpoints set (allows modification)
// Used when we need to add or remove checkpoints
std::set<uint64_t>& Route::checkpoints()
{
  //  //  //std::cout << "[Route::checkpoints]  Getting mutable checkpoints" << std::endl;
  //  //  //std::cout << "[Route::checkpoints] 📍 Checkpoints count: " << _pimpl->checkpoints.size() << std::endl;
  return _pimpl->checkpoints;  // Return reference to checkpoints set
}

//==============================================================================
// GET CHECKPOINTS (CONST VERSION)
// Returns a const reference to the checkpoints set (read-only)
// Used when we only need to read the checkpoint information
const std::set<uint64_t>& Route::checkpoints() const
{
  //  //  //std::cout << "[Route::checkpoints] 📖 Getting const checkpoints" << std::endl;
  //  //  //std::cout << "[Route::checkpoints] 📍 Checkpoints count: " << _pimpl->checkpoints.size() << std::endl;
  return _pimpl->checkpoints;  // Return const reference to checkpoints set
}

//==============================================================================
// SET DEPENDENCIES
// Sets the dependencies for this route (which other robots this route depends on)
// Dependencies define coordination relationships between robots
Route& Route::dependencies(DependsOnParticipant value)
{
  //  //  //std::cout << "[Route::dependencies]  Setting dependencies" << std::endl;
  //  //  //std::cout << "[Route::dependencies] 🔗 Old dependencies count: " << _pimpl->dependencies.size() << std::endl;
  //  //  //std::cout << "[Route::dependencies]  New dependencies count: " << value.size() << std::endl;
  
  _pimpl->dependencies = std::move(value);  // Move dependencies to avoid copying
  return *this;                             // Return reference for method chaining
}

//==============================================================================
// GET DEPENDENCIES (MUTABLE VERSION)
// Returns a reference to the dependencies (allows modification)
// Used when we need to add or modify dependencies
DependsOnParticipant& Route::dependencies()
{
  //  //  //std::cout << "[Route::dependencies] 📖 Getting mutable dependencies" << std::endl;
  //  //  //std::cout << "[Route::dependencies] 🔗 Dependencies count: " << _pimpl->dependencies.size() << std::endl;
  return _pimpl->dependencies;  // Return reference to dependencies map
}

//==============================================================================
// GET DEPENDENCIES (CONST VERSION)
// Returns a const reference to the dependencies (read-only)
// Used when we only need to read the dependency information
const DependsOnParticipant& Route::dependencies() const
{
  //  //  //std::cout << "[Route::dependencies] 📖 Getting const dependencies" << std::endl;
  //  //  //std::cout << "[Route::dependencies] 🔗 Dependencies count: " << _pimpl->dependencies.size() << std::endl;
  return _pimpl->dependencies;  // Return const reference to dependencies map
}

//==============================================================================
// ADD DEPENDENCY
// Adds a dependency on another robot's route/checkpoint
// This is the core function for establishing robot coordination and collision avoidance
Route& Route::add_dependency(
  const CheckpointId dependent_checkpoint,  // Our checkpoint that must wait
  const Dependency dep)                     // Details of what we're waiting for
{
  //  //  //std::cout << "[Route::add_dependency]  Adding dependency" << std::endl;
  //  //  //std::cout << "[Route::add_dependency]  Dependent checkpoint: " << dependent_checkpoint << std::endl;
  //  //  //std::cout << "[Route::add_dependency] 📋 Dependency details:" << std::endl;
  //  //  //std::cout << "[Route::add_dependency]   - Participant: " << dep.on_participant << std::endl;
  //  //  //std::cout << "[Route::add_dependency]   - Plan: " << dep.on_plan << std::endl;
  //  //  //std::cout << "[Route::add_dependency]   - Route: " << dep.on_route << std::endl;
  //  //  //std::cout << "[Route::add_dependency]   - Checkpoint: " << dep.on_checkpoint << std::endl;
  
  // Get or create the dependency entry for the specified participant
  // This creates a new entry if the participant doesn't exist in our dependencies
  auto& depends_on_plan = _pimpl->dependencies[dep.on_participant];
  
  // Check if we already have a plan dependency for this participant
  if (depends_on_plan.plan().has_value())
  {
  //  //  //std::cout << "[Route::add_dependency] 📋 Existing plan for participant " << dep.on_participant 
  //              << ": " << *depends_on_plan.plan() << std::endl;
    
    // If the new dependency is for an earlier plan than the current one, we
    // will ignore it (plans are numbered sequentially, higher numbers are newer)
    if (rmf_utils::modular(dep.on_plan).less_than(*depends_on_plan.plan()))
    {
  //  //  //std::cout << "[Route::add_dependency] ⚠️ Ignoring older plan " << dep.on_plan 
  //                << " (current: " << *depends_on_plan.plan() << ")" << std::endl;
      return *this;  // Ignore older plan, return without adding dependency
    }
    else if (dep.on_plan != *depends_on_plan.plan())
    {
  //  //  //std::cout << "[Route::add_dependency]  Newer plan detected, clearing old dependencies" << std::endl;
      // A newer plan exists for this other participant, so we will clear out
      // the old list of dependencies (newer plan supersedes older one)
      depends_on_plan.routes().clear();
    }
  }
  else
  {
  //  //  //std::cout << "[Route::add_dependency] ✨ First plan for participant " << dep.on_participant << std::endl;
  }

  // Set the plan ID for this dependency
  depends_on_plan.plan(dep.on_plan);
  //  //  //std::cout << "[Route::add_dependency] 🔧 Set plan ID: " << dep.on_plan << std::endl;
  
  // Add the specific route/checkpoint dependency
  depends_on_plan.add_dependency(
    dependent_checkpoint, {dep.on_route, dep.on_checkpoint});
  
  //  //  //std::cout << "[Route::add_dependency] ✅ Dependency added successfully" << std::endl;
  return *this;  // Return reference for method chaining
}

//==============================================================================
// SHOULD IGNORE PARTICIPANT
// Checks if this route should ignore information from a specific participant/plan
// Used during traffic negotiation to determine if we should consider a participant's plan
bool Route::should_ignore(
  const ParticipantId participant,  // ID of the participant to check
  const PlanId plan) const          // Plan ID to check
{
  //  //  //std::cout << "[Route::should_ignore]  Checking if should ignore participant" << std::endl;
  //  //  //std::cout << "[Route::should_ignore] 📋 Participant: " << participant << std::endl;
  //  //  //std::cout << "[Route::should_ignore] 📋 Plan: " << plan << std::endl;
  
  // Look for dependencies on this participant
  const auto p_it = _pimpl->dependencies.find(participant);
  if (p_it == _pimpl->dependencies.end())
  {
  //  //  //std::cout << "[Route::should_ignore] ✅ No dependencies on participant, don't ignore" << std::endl;
    return false;  // No dependencies on this participant, don't ignore
  }

  //  //  //std::cout << "[Route::should_ignore] 📋 Found dependencies on participant" << std::endl;
  
  // Check if we have a plan dependency for this participant
  if (!p_it->second.plan().has_value())
  {
  //  //  //std::cout << "[Route::should_ignore] ✅ No plan dependency, don't ignore" << std::endl;
    return false;  // No plan dependency, don't ignore
  }

  //  //  //std::cout << "[Route::should_ignore] 📋 Our plan dependency: " << *p_it->second.plan() << std::endl;
  
  // Return true if the given plan is older than our dependency
  // We should ignore older plans from this participant (they're outdated)
  bool should_ignore = rmf_utils::modular(plan).less_than(*p_it->second.plan());
  //  //  //std::cout << "[Route::should_ignore] " << (should_ignore ? "❌ IGNORE" : "✅ DON'T IGNORE") 
  //            << " Plan " << plan << std::endl;
  return should_ignore;
}

//==============================================================================
// CHECK DEPENDENCIES
// Checks if this route has dependencies on a specific participant's route
// Used to find specific coordination requirements between robots
const DependsOnCheckpoint* Route::check_dependencies(
  const ParticipantId on_participant,  // Participant to check
  const PlanId on_plan,                // Plan ID to check
  const RouteId on_route) const        // Route ID to check
{
  //  //  //std::cout << "[Route::check_dependencies] 🔍 Checking dependencies" << std::endl;
  //  //  //std::cout << "[Route::check_dependencies] 📋 Participant: " << on_participant << std::endl;
  //  //  //std::cout << "[Route::check_dependencies] 📋 Plan: " << on_plan << std::endl;
  //  //  //std::cout << "[Route::check_dependencies] 📋 Route: " << on_route << std::endl;
  
  // Look for dependencies on this participant
  const auto p_it = _pimpl->dependencies.find(on_participant);
  if (p_it == _pimpl->dependencies.end())
  {
  //  //  //std::cout << "[Route::check_dependencies] ❌ No dependencies on participant" << std::endl;
    return nullptr;  // No dependencies on this participant
  }

  //  //  //std::cout << "[Route::check_dependencies] ✅ Found dependencies on participant" << std::endl;
  
  // Get the plan dependencies for this participant
  const auto& plan_deps = p_it->second;
  const auto plan_deps_id = plan_deps.plan();
  if (!plan_deps_id.has_value())
  {
  //  //  //std::cout << "[Route::check_dependencies] ❌ No plan dependency" << std::endl;
    return nullptr;  // No plan dependency
  }

  //  //  //std::cout << "[Route::check_dependencies] 📋 Our plan dependency: " << *plan_deps_id << std::endl;
  
  // Check if the plan ID matches
  if (*plan_deps_id != on_plan)
  {
  //  //  //std::cout << "[Route::check_dependencies] ❌ Different plan, no dependency" << std::endl;
    return nullptr;  // Different plan, no dependency
  }

  //  //  //std::cout << "[Route::check_dependencies] ✅ Plan ID matches" << std::endl;
  
  // Look for the specific route in the dependencies
  const auto& routes = plan_deps.routes();
  const auto r_it = routes.find(on_route);
  if (r_it == routes.end())
  {
  //  //  //std::cout << "[Route::check_dependencies] ❌ No dependency on this specific route" << std::endl;
    return nullptr;  // No dependency on this specific route
  }

  //  //  //std::cout << "[Route::check_dependencies] ✅ Found dependency on route" << std::endl;
  //  //  //std::cout << "[Route::check_dependencies] 📋 Checkpoint dependencies count: " << r_it->second.size() << std::endl;
  
  // Return pointer to the checkpoint dependencies for this route
  return &r_it->second;
}

} // namespace rmf_traffic
