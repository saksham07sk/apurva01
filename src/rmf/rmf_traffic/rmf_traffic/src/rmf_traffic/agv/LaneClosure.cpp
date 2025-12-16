/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

// Include the header file that declares the LaneClosure class interface
#include <rmf_traffic/agv/LaneClosure.hpp>
// Include unordered_map for efficient hash-based storage of bitfields
#include <unordered_map>
#include <iostream>
// Enter the RMF traffic namespace
namespace rmf_traffic {
// Enter the AGV (Automated Guided Vehicle) sub-namespace
namespace agv {

//==============================================================================
// Private implementation class that handles the actual data storage and operations
// This uses the PIMPL (Pointer to Implementation) pattern for encapsulation
class LaneClosure::Implementation
{
public:

  // Check if a specific lane (by index) is marked as closed
  // This is the core method that route planners call to avoid closed lanes
  bool contains(std::size_t value) const
  {
    // Assert that we're on a 64-bit system (each bitfield stores 64 lane states)
    assert(field_size == 64);
    
    // Calculate which bitfield bucket this lane index belongs to
    // For lane 100: key = 100/64 = 1 (second bucket)
    const std::size_t key = _get_key(value);
    
    // Look up the bitfield bucket in our hash map
    const auto bucket_it = _bitfields.find(key);
    
    // If bucket doesn't exist, the lane is definitely open (not closed)
    if (bucket_it == _bitfields.end())
    {
      return false;
    }

    // Calculate which specific bit within the bucket represents this lane
    // For lane 100: bit = 1 << (100 % 64) = 1 << 36
    const std::size_t bit = _get_bit(value);
    
    // Get the actual bitfield value from the bucket
    const std::size_t bitfield = bucket_it->second;
    
    // Use bitwise AND to check if the specific bit is set
    // If bit is set, lane is closed; if not set, lane is open
    bool is_closed = static_cast<bool>(bitfield & bit);
    return is_closed;
  }

  // Mark a lane as closed by setting its corresponding bit
  // This is called when obstacles are detected or lanes need to be blocked
  void insert(std::size_t value)
  {
    // Calculate which bitfield bucket and bit position for this lane
    const std::size_t key = _get_key(value);
    const std::size_t bit = _get_bit(value);

    // Try to insert a new bitfield bucket with just this bit set
    const auto insertion = _bitfields.insert({key, bit});
    
    // Check if this was a new bucket (didn't exist before)
    if (insertion.second)
    {
      // The bitfield did not exist before, so now it has been inserted with
      // the desired bit. We will need to recalculate the hash. After that, we
      // can return.
      _recalculate_hash();
      return;
    }

    // Bucket already exists, get reference to the existing bitfield
    std::size_t& bitfield = insertion.first->second;
    
    // Check if this specific bit is already set (lane already closed)
    if (static_cast<bool>(bitfield & bit))
    {
      // The map already contained this value, so we do not need to insert it
      // or recalculate the hash.
      return;
    }

    // Set the bit using bitwise OR (lane is now closed)
    bitfield |= bit;
    
    // Recalculate hash since we modified the bitfield
    _recalculate_hash();
  }

  // Mark a lane as open by clearing its corresponding bit
  // This is called when obstacles are cleared or lanes are reopened
  void erase(std::size_t value)
  {
    // Calculate which bitfield bucket this lane belongs to
    const std::size_t key = _get_key(value);
    
    // Look up the bitfield bucket
    const auto bucket_it = _bitfields.find(key);
    
    // If bucket doesn't exist, lane is already open (nothing to do)
    if (bucket_it == _bitfields.end())
    {
      // There was no bitfield for this value, so it is not in the map anyway.
      // We can simply return here.
      return;
    }

    // Calculate which specific bit represents this lane
    const std::size_t bit = _get_bit(value);
    
    // Get reference to the bitfield
    std::size_t& bitfield = bucket_it->second;
    
    // Check if the bit is currently set (lane is closed)
    if (!static_cast<bool>(bitfield & bit))
    {
      // This bit was not active in the bitfield, so we do not need to do
      // anything.
      return;
    }

    // Clear the bit using bitwise AND with NOT (lane is now open)
    bitfield &= ~bit;
    
    // Recalculate hash since we modified the bitfield
    _recalculate_hash();
  }

  // Return the current hash value for this LaneClosure state
  // Used for efficient comparison and caching in route planning
  std::size_t hash() const
  {
    return _hash;
  }

  // Compare two LaneClosure objects for equality
  // Used to check if lane closure states are identical
  bool operator==(const Implementation& other) const
  {
    return _bitfields == other._bitfields;
  }

private:

  // Hash map storing bitfields: key = bucket index, value = 64-bit bitfield
  // Each bitfield can track 64 lanes (on 64-bit systems)
  std::unordered_map<std::size_t, std::size_t> _bitfields;
  
  // Cached hash value for fast comparison and lookup
  std::size_t _hash = 0;

  // Number of bits per bitfield (64 on 64-bit systems, 32 on 32-bit systems)
  static constexpr std::size_t field_size = sizeof(std::size_t)*8;

  // Recalculate the hash value by combining all bitfields
  // This is called whenever the bitfields are modified
  void _recalculate_hash()
  {
    _hash = 0;
    // Iterate through all bitfield buckets
    for (const auto& [_, bitfield] : _bitfields)
      // Use bitwise OR to combine all bitfields into a single hash
      _hash |= bitfield;
  }

  // Calculate which bitfield bucket a lane index belongs to
  // For lane 100: returns 100/64 = 1 (second bucket)
  std::size_t _get_key(const std::size_t value) const
  {
    return value/field_size;
  }

  // Calculate which specific bit within a bucket represents a lane
  // For lane 100: returns 1 << (100 % 64) = 1 << 36
  std::size_t _get_bit(const std::size_t value) const
  {
    // Calculate the bit position within the 64-bit field
    const std::size_t shift = value % field_size;
    // Create a bitmask with only this bit set
    return std::size_t(1) << shift;
  }
};

//==============================================================================
// Default constructor - creates an empty LaneClosure with all lanes open
// This is called when a new LaneClosure object is created
LaneClosure::LaneClosure()
: _pimpl(rmf_utils::make_impl<Implementation>())  // Create the private implementation
{
  // Do nothing - all lanes are open by default
}

//==============================================================================
// Check if a specific lane is open (available for use)
// This is the method that route planners call to check lane availability
bool LaneClosure::is_open(const std::size_t lane) const
{
  // A lane is open if it's NOT closed
  return !is_closed(lane);
}

//==============================================================================
// Check if a specific lane is closed (blocked/unavailable)
// This is the core method used by route planning algorithms to avoid closed lanes
bool LaneClosure::is_closed(const std::size_t lane) const
{
  // Delegate to the implementation's contains method
  return _pimpl->contains(lane);
}

//==============================================================================
// Mark a lane as open (available for use)
// This is called when obstacles are cleared or lanes are reopened
LaneClosure& LaneClosure::open(const std::size_t lane)
{
  // Remove the lane from the closed lanes set
  _pimpl->erase(lane);
  // Return reference to self for method chaining
  return *this;
}

//==============================================================================
// Mark a lane as closed (blocked/unavailable)
// This is called when obstacles are detected or lanes need to be blocked
LaneClosure& LaneClosure::close(std::size_t lane)
{
  // Add the lane to the closed lanes set
  _pimpl->insert(lane);
  // Return reference to self for method chaining
  return *this;
}

//==============================================================================
// Get the hash value for this LaneClosure state
// Used for efficient comparison and caching in route planning systems
std::size_t LaneClosure::hash() const
{
  // Delegate to the implementation's hash method
  return _pimpl->hash();
}

//==============================================================================
// Compare two LaneClosure objects for equality
// Used to check if two lane closure states are identical
bool LaneClosure::operator==(const LaneClosure& other) const
{
  // Compare the underlying implementations
  return *_pimpl == *other._pimpl;
}

// End of AGV namespace
} // namespace agv
// End of RMF traffic namespace
} // namespace rmf_traffic
