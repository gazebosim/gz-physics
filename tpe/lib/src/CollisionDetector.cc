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

#include "CollisionDetector.hh"
#include "Utils.hh"

using namespace ignition;
using namespace physics;
using namespace tpelib;


//////////////////////////////////////////////////
std::vector<Contact> CollisionDetector::CheckCollisions(
    const std::map<std::size_t, std::shared_ptr<Entity>> &_entities,
    bool _singleContact)
{
  // contacts to be filled and returned
  std::vector<Contact> contacts;

  // cache of collide bitmasks
  std::map<std::size_t, uint16_t> collideBitmasks;

  // cache of axis aligned box in world frame
  std::map<std::size_t, math::AxisAlignedBox> worldAabb;

  for (auto it = _entities.begin(); it != _entities.end(); ++it)
  {
    std::shared_ptr<Entity> e1 = it->second;

    // Get collide bitmask for enitty 1
    uint16_t cb1 = 0xFF;
    auto cb1It = collideBitmasks.find(e1->GetId());
    if (cb1It == collideBitmasks.end())
      collideBitmasks[e1->GetId()] = e1->GetCollideBitmask();
    else
      cb1 = cb1It->second;

    // Get world axis aligned box for entity 1
    math::AxisAlignedBox wb1;
    auto wb1It = worldAabb.find(e1->GetId());
    if (wb1It == worldAabb.end())
    {
      // get bbox in local frame
      math::AxisAlignedBox b1 = e1->GetBoundingBox();
      // convert to world aabb
      math::Pose3d p1 = e1->GetPose();
      wb1 = transformAxisAlignedBox(b1, p1);
      worldAabb[e1->GetId()] = wb1;
    }
    else
    {
      wb1 = wb1It->second;
    }

    for (auto it2 = std::next(it, 1); it2 != _entities.end(); ++it2)
    {
      std::shared_ptr<Entity> e2 = it2->second;

      // Get collid bitmask for entity 2
      uint16_t cb2 = 0xFF;
      auto cb2It = collideBitmasks.find(e2->GetId());
      if (cb2It == collideBitmasks.end())
        collideBitmasks[e2->GetId()] = e2->GetCollideBitmask();
      else
        cb2 = cb2It->second;

      // collision filtering using collide bitmask
      if ((cb1 & cb2) == 0)
        continue;

      // Get world axis aligned box for entity 2
      math::AxisAlignedBox wb2;
      auto wb2It = worldAabb.find(e2->GetId());
      if (wb2It == worldAabb.end())
      {
        // get bbox in local frame
        math::AxisAlignedBox b2 = e2->GetBoundingBox();
        // convert to world aabb
        math::Pose3d p2 = e2->GetPose();
        wb2 = transformAxisAlignedBox(b2, p2);
        worldAabb[e2->GetId()] = wb2;
      }
      else
      {
        wb2 = wb2It->second;
      }

      // Check intersection
      std::vector<math::Vector3d> points;
      if (this->GetIntersectionPoints(wb1, wb2, points, _singleContact))
      {
        Contact c;
        // TPE checks collisions in the model level so contacts are associated
        // with models and not collisions!
        c.entity1 = e1->GetId();
        c.entity2 = e2->GetId();
        for (const auto &p : points)
        {
          c.point = p;
          contacts.push_back(c);
        }
      }
    }
  }
  return contacts;
}

//////////////////////////////////////////////////
bool CollisionDetector::GetIntersectionPoints(const math::AxisAlignedBox &_b1,
    const math::AxisAlignedBox &_b2,
    std::vector<math::Vector3d> &_points, bool _singleContact)
{
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
