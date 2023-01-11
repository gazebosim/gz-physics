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

#include <set>

#include <gz/common/Console.hh>

#include "aabb_tree/AABB.h"

#include "AABBTree.hh"

namespace ignition {
namespace physics {
namespace tpelib {

/// \brief Private data class for AABBTree
class AABBTreePrivate
{
  /// \brief Pointer to the AABB tree
  public: std::unique_ptr<aabb::Tree> aabbTree;

  /// \brief A map of node id and its AABB object in the tree
  // public: std::unordered_map<std::size_t, unsigned int> nodeIds;
  public: std::set<std::size_t> nodeIds;
};
}
}
}

using namespace gz;
using namespace physics;
using namespace tpelib;

//////////////////////////////////////////////////
AABBTree::AABBTree()
  : dataPtr(new ::tpelib::AABBTreePrivate)
{
  this->dataPtr->aabbTree = std::make_unique<aabb::Tree>(3, 0.0, 100000);
}

//////////////////////////////////////////////////
AABBTree::~AABBTree() = default;

//////////////////////////////////////////////////
void AABBTree::AddNode(std::size_t _id, const math::AxisAlignedBox &_aabb)
{
  std::vector<double> lowerBound(3);
  lowerBound[0] = _aabb.Min().X();
  lowerBound[1] = _aabb.Min().Y();
  lowerBound[2] = _aabb.Min().Z();

  std::vector<double> upperBound(3);
  upperBound[0] = _aabb.Max().X();
  upperBound[1] = _aabb.Max().Y();
  upperBound[2] = _aabb.Max().Z();

  this->dataPtr->aabbTree->insertParticle(_id, lowerBound, upperBound);
  this->dataPtr->nodeIds.insert(_id);
}

//////////////////////////////////////////////////
bool AABBTree::RemoveNode(std::size_t _id)
{
  auto it = this->dataPtr->nodeIds.find(_id);
  if (it == this->dataPtr->nodeIds.end())
  {
    ignerr << "Unable to remove node '" << _id << "'. "
           << "Node not found." << std::endl;
    return false;
  }

  this->dataPtr->aabbTree->removeParticle(_id);
  this->dataPtr->nodeIds.erase(it);
  return true;
}

//////////////////////////////////////////////////
bool AABBTree::UpdateNode(std::size_t _id,
    const math::AxisAlignedBox &_aabb)
{
  auto it = this->dataPtr->nodeIds.find(_id);
  if (it == this->dataPtr->nodeIds.end())
  {
    ignerr << "Unable to update node '" << _id << "'. "
           << "Node not found." << std::endl;
    return false;
  }

  std::vector<double> lowerBound(3);
  lowerBound[0] = _aabb.Min().X();
  lowerBound[1] = _aabb.Min().Y();
  lowerBound[2] = _aabb.Min().Z();

  std::vector<double> upperBound(3);
  upperBound[0] = _aabb.Max().X();
  upperBound[1] = _aabb.Max().Y();
  upperBound[2] = _aabb.Max().Z();


  this->dataPtr->aabbTree->updateParticle(_id, lowerBound, upperBound);
  return true;
}

//////////////////////////////////////////////////
unsigned int AABBTree::NodeCount() const
{
  return this->dataPtr->nodeIds.size();
}

//////////////////////////////////////////////////
std::set<std::size_t> AABBTree::Collisions(std::size_t _id) const
{
  std::set<std::size_t> result;
  auto it = this->dataPtr->nodeIds.find(_id);
  if (it == this->dataPtr->nodeIds.end())
  {
    ignerr << "Unable to compute collisions for node '" << _id << "'. "
           << "Node not found." << std::endl;
    return result;
  }

  auto collisions  = this->dataPtr->aabbTree->query(_id);
  result = std::set<std::size_t>(collisions.begin(), collisions.end());
  return result;
}

//////////////////////////////////////////////////
math::AxisAlignedBox AABBTree::AABB(std::size_t _id) const
{
  auto it = this->dataPtr->nodeIds.find(_id);
  if (it == this->dataPtr->nodeIds.end())
  {
    ignerr << "Unable to get AABB for node '" << _id << "'. "
           << "Node not found." << std::endl;
    return math::AxisAlignedBox();
  }

  auto aabb = this->dataPtr->aabbTree->getAABB(_id);

  return math::AxisAlignedBox(
      math::Vector3d(
      aabb.lowerBound[0], aabb.lowerBound[1], aabb.lowerBound[2]),
      math::Vector3d(
      aabb.upperBound[0], aabb.upperBound[1], aabb.upperBound[2]));
}

//////////////////////////////////////////////////
bool AABBTree::HasNode(std::size_t _id) const
{
  auto it = this->dataPtr->nodeIds.find(_id);
  return it != this->dataPtr->nodeIds.end();
}
