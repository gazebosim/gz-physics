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

#ifndef GZ_PHYSICS_TPE_LIB_SRC_LINK_HH_
#define GZ_PHYSICS_TPE_LIB_SRC_LINK_HH_

#include <gz/utilities/SuppressWarning.hh>

#include "gz/physics/tpelib/Export.hh"

#include "Entity.hh"

namespace gz {
namespace physics {
namespace tpelib {

/// \brief Link class
class IGNITION_PHYSICS_TPELIB_VISIBLE Link : public Entity
{
  /// \brief Constructor
  public: Link();

  /// \brief Constructor
  /// \param[in] _id Link id
  public: explicit Link(std::size_t _id);

  /// \brief Destructor
  public: ~Link() = default;

  /// \brief Add a collision
  /// \return Newly created Collision
  public: Entity &AddCollision();

  /// \brief Set the linear velocity of link relative to parent
  /// \param[in] _velocity linear velocity in meters per second
  public: void SetLinearVelocity(const math::Vector3d &_velocity);

  /// \brief Get the linear velocity of link relative to parent
  /// \return linear velocity of link in meters per second
  public: math::Vector3d GetLinearVelocity() const;

  /// \brief Set the angular velocity of link relative to parent
  /// \param[in] _velocity angular velocity in radians per second
  public: void SetAngularVelocity(const math::Vector3d &_velocity);

  /// \brief Get the angular velocity of link relative to parent
  /// \return angular velocity in radians per second
  public: math::Vector3d GetAngularVelocity() const;

  /// \brief Update the pose of the entity
  /// \param[in] _timeStep current world timestep in seconds
  public: virtual void UpdatePose(double _timeStep);

  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief linear velocity of link
  protected: math::Vector3d linearVelocity;

  /// \brief angular velocity of link
  protected: math::Vector3d angularVelocity;
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

}
}
}

#endif
