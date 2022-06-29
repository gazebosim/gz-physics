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

#ifndef GZ_PHYSICS_TPE_LIB_SRC_MODEL_HH_
#define GZ_PHYSICS_TPE_LIB_SRC_MODEL_HH_

#include <gz/utils/SuppressWarning.hh>

#include "gz/physics/tpelib/Export.hh"

#include "Entity.hh"

namespace gz {
namespace physics {
namespace tpelib {

// forward declaration
class ModelPrivate;

/// \brief Model class
class GZ_PHYSICS_TPELIB_VISIBLE Model : public Entity
{
  /// \brief Constructor
  public: Model();

  /// \brief Constructor
  /// \param[in] _id Model id
  public: explicit Model(std::size_t _id);

  /// \brief Destructor
  public: virtual ~Model();

  /// \brief Add a link
  /// \return Newly created Link
  public: Entity &AddLink();

  /// \brief Get the number of links in the model
  /// \return Number of links in the model
  public: std::size_t GetLinkCount() const;

  /// \brief Add a nested model
  /// \return Newly created nested model
  public: Entity &AddModel();

  /// \brief Get the number of nested models in the model
  /// \return Number of nested models in the model
  public: std::size_t GetModelCount() const;

  /// \brief Set the canonical link of model
  public: void SetCanonicalLink(
    std::size_t linkId = kNullEntityId);

  /// \brief Get the canonical link of model
  /// \return Entity the canonical (first) link
  public: Entity &GetCanonicalLink();

  /// \brief Set the linear velocity of model relative to parent
  /// \param[in] _velocity linear velocity in meters per second
  public: void SetLinearVelocity(const math::Vector3d &_velocity);

  /// \brief Get the linear velocity of model relative to parent
  /// \return linear velocity of model in meters per second
  public: math::Vector3d GetLinearVelocity() const;

  /// \brief Set the angular velocity of model relative to parent
  /// \param[in] _velocity angular velocity from world in radians per second
  public: void SetAngularVelocity(const math::Vector3d &_velocity);

  /// \brief Get the angular velocity of model relative to parent
  /// \return angular velocity in radians per second
  public: math::Vector3d GetAngularVelocity() const;

  /// \brief Update the pose of the entity
  /// \param[in] _timeStep current world timestep in seconds
  public: virtual void UpdatePose(double _timeStep);

  /// \brief Removes a child entity (either a link or model) from the
  /// appropriate child entity containers
  /// \param[in] _ent Pointer to entity
  /// \return True if the entity was found and removed
  public: bool RemoveChildEntityBasedOnType(const Entity *_ent);
  /// \brief Remove a child entity by id
  /// \param[in] _id Id of child entity to remove
  /// \return True if the entity was found and removed
  public: bool RemoveChildById(std::size_t _id) override;

  /// \brief Remove a child entity by name
  /// \param[in] _name Name of child entity to remove
  /// \return True if child entity was removed, false otherwise
  public: bool RemoveChildByName(const std::string &_name) override;

  GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief linear velocity of model
  protected: math::Vector3d linearVelocity;

  /// \brief angular velocity of model
  protected: math::Vector3d angularVelocity;
  GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING

  /// \brief Remove a model entity by id
  /// \param[in] _id Id of model entity to remove
  private: bool RemoveModelById(std::size_t _id);

  /// \brief Remove a link entity by id
  /// \param[in] _id Id of link entity to remove
  private: bool RemoveLinkById(std::size_t _id);

  /// \brief Pointer to private data class
  private: ModelPrivate *dataPtr = nullptr;
};

}
}
}

#endif
