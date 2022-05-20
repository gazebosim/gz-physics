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

#ifndef GZ_PHYSICS_TPE_LIB_SRC_AABBTREE_HH_
#define GZ_PHYSICS_TPE_LIB_SRC_AABBTREE_HH_

#include <memory>
#include <set>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/utils/SuppressWarning.hh>

#include "gz/physics/tpelib/Export.hh"

namespace gz {
namespace physics {
namespace tpelib {

// forward declaration
class AABBTreePrivate;

class IGNITION_PHYSICS_TPELIB_VISIBLE AABBTree
{
  /// \brief Constructor
  public: AABBTree();

  /// \brief Destructor
  public: ~AABBTree();

  /// \brief Add a node to the tree
  /// \param[in] _aabb Axis aligned bounding box of the node
  /// \param[in] _id Unique id of this node
  public: void AddNode(std::size_t _id, const math::AxisAlignedBox &_aabb);

  /// \brief Remove a node from the tree
  /// \param[in] _id Node id
  /// \return True if the node was successfully removed, false otherwise
  public: bool RemoveNode(std::size_t _id);

  /// \brief Update a node's axis aligned bounding box
  /// \param[in] _id Node id
  /// \param[in] _aabb New axis aligned bounding box
  /// \return True if the update was successful, false otherwise
  public: bool UpdateNode(std::size_t _id, const math::AxisAlignedBox &_aabb);

  /// \brief Get the number of nodes in the tree
  /// \return Number of nodes
  public: unsigned int NodeCount() const;

  /// \brief Get all the nodes that collide / intersect with input node
  /// \param[in] _id Input node id
  /// \return A set of node ids that collide with the input node
  public: std::set<std::size_t> Collisions(std::size_t _id) const;

  /// \brief Get the AABB for a node
  /// \param[in] _id Node id
  /// \return Node's AABB
  public: math::AxisAlignedBox AABB(std::size_t _id) const;

  /// \brief Get whether the tree has a node with specified id
  /// \param[in] _id Node id
  /// \return True if tree has node, false otherwise
  public: bool HasNode(std::size_t _id) const;

  /// \brief Pointer to the private data
  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  private: std::unique_ptr<AABBTreePrivate> dataPtr;
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};
}
}
}

#endif
