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

#ifndef GZ_PHYSICS_TPE_LIB_SRC_COLLISIONDETECTOR_HH_
#define GZ_PHYSICS_TPE_LIB_SRC_COLLISIONDETECTOR_HH_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/utils/SuppressWarning.hh>

#include "gz/physics/tpelib/Export.hh"

#include "Entity.hh"

#include "AABBTree.hh"

namespace ignition {
namespace physics {
namespace tpelib {

// forward declaration
class CollisionDetectorPrivate;

/// \brief A data structure to store contact properties
class IGNITION_PHYSICS_TPELIB_VISIBLE Contact
{
  /// \brief Id of frst collision entity
  public: std::size_t entity1 = kNullEntityId;

  /// \brief Id of second collision entity
  public: std::size_t entity2 = kNullEntityId;

  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief Point of contact in world frame;
  public: math::Vector3d point;
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

/// \brief Collision Detector that checks collisions between a list of entities
class IGNITION_PHYSICS_TPELIB_VISIBLE CollisionDetector
{
  /// \brief Constructor
  public: CollisionDetector();

  /// \brief Destructor
  public: ~CollisionDetector();

  /// \brief Check collisions between a list entities and get all contact points
  /// \param[in] _entities List of entities
  /// \param[in] _singleContact Get only 1 contact point for each pair of
  /// collisions.
  /// The contact point will be at the center of all points
  /// \return A list of contact points
  public: std::vector<Contact> CheckCollisions(
      const std::map<std::size_t, std::shared_ptr<Entity>> &_entities,
      bool _singleContact = false);

  /// \brief Get a vector of intersection points between two axis aligned boxes
  /// \param[in] _b1 Axis aligned box 1
  /// \param[in] _b2 Axis aligned box 2
  /// \param[out] _points Intersection points to be filled
  /// \param[in] _singleContact Get only 1 intersection point at center of
  /// all points
  public: bool GetIntersectionPoints(const math::AxisAlignedBox &_b1,
      const math::AxisAlignedBox &_b2,
      std::vector<math::Vector3d> &_points,
      bool _singleContact = false);

  /// \brief Pointer to private data
  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  private: std::unique_ptr<CollisionDetectorPrivate> dataPtr;
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

}
}
}

#endif
