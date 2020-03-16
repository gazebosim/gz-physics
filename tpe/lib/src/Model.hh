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

#ifndef IGNITION_PHYSICS_TPE_LIB_SRC_MODEL_HH_
#define IGNITION_PHYSICS_TPE_LIB_SRC_MODEL_HH_

#include <ignition/math/Pose3.hh>

#include "Entity.hh"

namespace ignition {
namespace physics {
namespace tpe {
namespace lib{

// class Link;

/// \brief Model class
class Model : public Entity
{
  /// \brief Constructor
  public: Model();

  /// \brief Constructor
  /// \param _id Model id
  public: Model(uint64_t _id);

  /// \brief Destructor
  public: ~Model() = default;

  /// \brief Add a link
  /// \return Newly created Link
  public: Entity &AddLink();

  /// \brief Get a link
  /// \param _linkName link name
  /// \return Link
  public: Entity &GetLinkByName(const std::string &_name);

  /// \brief Set the linear velocity of model
  /// \param _velocity linear velocity
  public: void SetLinearVelocity(const math::Vector3d _velocity);

  /// \brief Get the linear velocity of model
  /// \return _velocity linear velocity of model
  public: math::Vector3d GetLinearVelocity() const;

  /// \brief Set the angular velocity of model
  /// \param _velocity angular velocity from world
  public: void SetAngularVelocity(const math::Vector3d _velocity);

  /// \brief Get the angular velocity of model
  /// \return _velocity angular velocity
  public: math::Vector3d GetAngularVelocity() const;

  /// \brief Update the pose of the entity
  /// \param _timeStep current world timestep
  /// \param _linearVelocity linear velocity
  /// \param _angularVelocity angular velocity
  public: virtual void UpdatePose(
    const double _timeStep,
    const math::Vector3d _linearVelocity,
    const math::Vector3d _angularVelocity);

  /// \brief linear velocity of model
  protected: math::Vector3d linearVelocity;

  /// \brief angular velocity of model
  protected: math::Vector3d angularVelocity;
};

}
}
}
}

#endif
