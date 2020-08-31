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
#include <unordered_map>

#include <ignition/common/Profiler.hh>

#include "CollisionDetector.hh"
#include "Utils.hh"

#include "AABBTree.hh"

/// \brief Private data class for CollisionDetector
class ignition::physics::tpelib::CollisionDetectorPrivate
{
  /// \brief Helper function to check if collisions for a pair of nodes have
  /// already been recorded or not
  /// \param[in] _a Node A Id
  /// \param[in] _b Node B Id
  /// \return True if this is a duplicate collision
  public: bool CheckDuplicateCollisionPair(std::size_t _a, std::size_t _b);

  /// \brief AABB tree
  public: AABBTree aabbTree;

  /// \brief Set of entity id
  public: std::set<std::size_t> nodeIds;

  /// \brief Keep track of pairs of node ids that collided. The map is cleared
  /// after each collision detection iteration. The key and value are:
  ///   std::unorderd_map<node_a_id, std::unordered_map<node_b_id, collided>
  public: std::unordered_map<std::size_t, std::unordered_map<std::size_t, bool>>
    collisionStateMap;
};

using namespace ignition;
using namespace physics;
using namespace tpelib;

//////////////////////////////////////////////////
CollisionDetector::CollisionDetector()
  : dataPtr(new CollisionDetectorPrivate)
{
}

//////////////////////////////////////////////////
CollisionDetector::~CollisionDetector()
{
}

//////////////////////////////////////////////////
std::vector<Contact> CollisionDetector::CheckCollisions(
    const std::map<std::size_t, std::shared_ptr<Entity>> &_entities,
    bool _singleContact)
{
  IGN_PROFILE("tpelib::CollisionDetector::CheckCollisions");

  // contacts to be filled and returned
  std::vector<Contact> contacts;

  // update AABB tree
  // remove nodes that no longer exist
  auto nodesToCheckForRemoval = this->dataPtr->nodeIds;
  for (auto id : nodesToCheckForRemoval)
  {
    if (_entities.find(id) == _entities.end())
    {
      this->dataPtr->aabbTree.RemoveNode(id);
      this->dataPtr->nodeIds.erase(id);
    }
  }

  // add and update nodes in the tree
  for (auto it = _entities.begin(); it != _entities.end(); ++it)
  {
    std::shared_ptr<Entity> e = it->second;
    // add new nodes
    if (!this->dataPtr->aabbTree.HasNode(it->first))
    {
      math::AxisAlignedBox b = e->GetBoundingBox();

      if (b == math::AxisAlignedBox())
        continue;

      // convert to world aabb
      math::AxisAlignedBox aabb;
      math::Pose3d p = e->GetPose();
      aabb = transformAxisAlignedBox(b, p);
      this->dataPtr->aabbTree.AddNode(e->GetId(), aabb);

      this->dataPtr->nodeIds.insert(it->first);
    }
    // update existing nodes
    else if (e->PoseDirty())
    {
      math::AxisAlignedBox b = e->GetBoundingBox();

      if (b == math::AxisAlignedBox())
        continue;

      // convert to world aabb
      math::AxisAlignedBox aabb;
      math::Pose3d p = e->GetPose();
      aabb = transformAxisAlignedBox(b, p);
      this->dataPtr->aabbTree.UpdateNode(e->GetId(), aabb);
    }
  }

  // query AABB tree for collisions
  for (auto it = _entities.begin(); it != _entities.end(); ++it)
  {
    std::shared_ptr<Entity> e = it->second;

    math::AxisAlignedBox b = e->GetBoundingBox();
    if (b == math::AxisAlignedBox())
        continue;

    // check collisions
    auto result = this->dataPtr->aabbTree.Collisions(e->GetId());
    if (result.empty())
      continue;

    math::AxisAlignedBox wb1 = this->dataPtr->aabbTree.AABB(e->GetId());

    // Get collide bitmask for entity 1
    uint16_t cb1 = e->GetCollideBitmask();

    // Check intersection
    for (const auto &nId : result)
    {
      // skip if we have already checked collision for this pair of nodes
      if (this->dataPtr->CheckDuplicateCollisionPair(e->GetId(), nId))
        continue;

      // Get collide bitmask for entity 2
      uint16_t cb2 = _entities.at(nId)->GetCollideBitmask();

      // collision filtering using collide bitmask
      if ((cb1 & cb2) == 0)
        continue;

      std::vector<math::Vector3d> points;
      math::AxisAlignedBox wb2 = this->dataPtr->aabbTree.AABB(nId);
      if (this->GetIntersectionPoints(wb1, wb2, points, _singleContact))
      {
        Contact c;
        // TPE checks collisions in the model level so contacts are associated
        // with models and not collisions!
        c.entity1 = e->GetId();
        c.entity2 = nId;
        for (const auto &p : points)
        {
          c.point = p;
          contacts.push_back(c);
        }
      }
    }
  }

  this->dataPtr->collisionStateMap.clear();
  return contacts;
}

//////////////////////////////////////////////////
bool CollisionDetector::GetIntersectionPoints(const math::AxisAlignedBox &_b1,
    const math::AxisAlignedBox &_b2,
    std::vector<math::Vector3d> &_points, bool _singleContact)
{
  IGN_PROFILE("CollisionDetector::GetIntersectionPoints");
  // fast intersection check
  if (_b1.Intersects(_b2))
  {
    // when two boxes intersect, the overlapping region is a small box
    // get all corners of this intersection box
    math::Vector3d min;
    math::Vector3d max;
    min.X() = std::max(_b1.Min().X(), _b2.Min().X());
    min.Y() = std::max(_b1.Min().Y(), _b2.Min().Y());
    min.Z() = std::max(_b1.Min().Z(), _b2.Min().Z());

    max.X() = std::min(_b1.Max().X(), _b2.Max().X());
    max.Y() = std::min(_b1.Max().Y(), _b2.Max().Y());
    max.Z() = std::min(_b1.Max().Z(), _b2.Max().Z());

    if (_singleContact)
    {
      // return center of intersecting region
      _points.push_back(min + 0.5*(max-min));
      return true;
    }

    // min min min
    math::Vector3d corner = min;
    _points.push_back(corner);

    // min min max
    corner.Z() = max.Z();
    _points.push_back(corner);

    // min max max
    corner.Y() = max.Y();
    _points.push_back(corner);

    // min max min
    corner.Z() = min.Z();
    _points.push_back(corner);

    // max max min
    corner.X() = max.X();
    _points.push_back(corner);

    // max max max
    corner.Z() = max.Z();
    _points.push_back(corner);

    // max min max
    corner.Y() = min.Y();
    _points.push_back(corner);

    // max min min
    corner.Z() = min.Z();
    _points.push_back(corner);

    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool CollisionDetectorPrivate::CheckDuplicateCollisionPair(
    std::size_t _a, std::size_t _b)
{
  // use a 2d map to keep track of pairs of collisions
  // mark the corresponding elements in the 2d map to true to indicate
  // the check is done
  bool duplicate = true;
  auto aIt = this->collisionStateMap.find(_a);
  if (aIt == this->collisionStateMap.end())
  {
    duplicate = false;
  }
  else
  {
    auto bIt = aIt->second.find(_b);
    if (bIt == aIt->second.end())
    {
      duplicate = false;
    }
  }

  if (!duplicate)
  {
    this->collisionStateMap[_a][_b] = true;
    this->collisionStateMap[_b][_a] = true;
  }
  return duplicate;
}
