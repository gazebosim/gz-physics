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

#include <ignition/common/Profiler.hh>

#include "CollisionDetector.hh"
#include "Utils.hh"

#include "aabb_tree/AABBTree.h"
#include "aabb_tree/IAABB.h"

/// \brief Private data class for CollisionDetector
class ignition::physics::tpelib::CollisionDetectorPrivate
{
  /// \brief AABB tree
  public: AABBTreeIface aabbTree;

  /// \brief Set of entity id
  public: std::set<std::size_t> nodeIds;
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

  for (auto it = _entities.begin(); it != _entities.end(); ++it)
  {
    std::shared_ptr<Entity> e = it->second;
    // add new nodes
    if (!this->dataPtr->aabbTree.HasNode(it->first))
    {
      math::AxisAlignedBox b = e->GetBoundingBox();
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

    auto result = this->dataPtr->aabbTree.Collisions(e->GetId());
    if (result.empty())
      continue;

    math::AxisAlignedBox wb1 = this->dataPtr->aabbTree.AABB(e->GetId());

    // Get collide bitmask for entity 1
    uint16_t cb1 = e->GetCollideBitmask();

    // Check intersection
    for (const auto &nId : result)
    {
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

  std::cerr << "contacts ===== " << contacts.size() << std::endl;
  return contacts;
}


// //////////////////////////////////////////////////
// std::vector<Contact> CollisionDetector::CheckCollisions(
//     const std::map<std::size_t, std::shared_ptr<Entity>> &_entities,
//     bool _singleContact)
// {
//   IGN_PROFILE("CollisionDetector::CheckCollisions");
//
//   // contacts to be filled and returned
//   std::vector<Contact> contacts;
//
//   // cache of axis aligned box in world frame
//   std::unordered_map<std::size_t, math::AxisAlignedBox> worldAabb;
//
//   for (auto it = _entities.begin(); it != _entities.end(); ++it)
//   {
//     std::shared_ptr<Entity> e1 = it->second;
//
//     // Get collide bitmask for enitty 1
//     uint16_t cb1 = e1->GetCollideBitmask();
//
//     // Get world axis aligned box for entity 1
//     math::AxisAlignedBox wb1;
//     auto wb1It = worldAabb.find(e1->GetId());
//     if (wb1It == worldAabb.end())
//     {
//       // get bbox in local frame
//       math::AxisAlignedBox b1 = e1->GetBoundingBox();
//       // convert to world aabb
//       math::Pose3d p1 = e1->GetPose();
//       wb1 = transformAxisAlignedBox(b1, p1);
//       worldAabb[e1->GetId()] = wb1;
//     }
//     else
//     {
//       wb1 = wb1It->second;
//     }
//
//     for (auto it2 = std::next(it, 1); it2 != _entities.end(); ++it2)
//     {
//       std::shared_ptr<Entity> e2 = it2->second;
//
// //      IGN_PROFILE_BEGIN("CollisionDetector GetCollideBitMask");
//       // Get collide bitmask for entity 2
//       uint16_t cb2 = e2->GetCollideBitmask();
// //      IGN_PROFILE_END();
//
//       // collision filtering using collide bitmask
//       if ((cb1 & cb2) == 0)
//         continue;
//
//       IGN_PROFILE_BEGIN("CollisionDetector find bbox");
//       // Get world axis aligned box for entity 2
//       math::AxisAlignedBox wb2;
//       auto wb2It = worldAabb.find(e2->GetId());
//       IGN_PROFILE_END();
//
//       if (wb2It == worldAabb.end())
//       {
//         // get bbox in local frame
//         math::AxisAlignedBox b2 = e2->GetBoundingBox();
//         // convert to world aabb
//         math::Pose3d p2 = e2->GetPose();
//         wb2 = transformAxisAlignedBox(b2, p2);
//         worldAabb[e2->GetId()] = wb2;
//       }
//       else
//       {
//         wb2 = wb2It->second;
//       }
//
//       // Check intersection
//       std::vector<math::Vector3d> points;
//       if (this->GetIntersectionPoints(wb1, wb2, points, _singleContact))
//       {
//         std::cerr << "got intersection " << std::endl;
//         Contact c;
//         // TPE checks collisions in the model level so contacts are associated
//         // with models and not collisions!
//         c.entity1 = e1->GetId();
//         c.entity2 = e2->GetId();
//         for (const auto &p : points)
//         {
//           c.point = p;
//           contacts.push_back(c);
//         }
//       }
//     }
//   }
//
//   return contacts;
// }

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
