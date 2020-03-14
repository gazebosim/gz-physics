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

#ifndef IGNITION_PHYSICS_TPESIM_TPE_ENTITY_HH_
#define IGNITION_PHYSICS_TPESIM_TPE_ENTITY_HH_

#include <string>
#include <map>
#include <memory>

#include <ignition/math/Pose3.hh>

namespace ignition {
namespace physics {
namespace tpe {

// forward declaration
class EntityPrivate;

/// \brief Represents an invalid Id.
static const uint64_t kNullEntityId = math::MAX_UI64;

class Entity
{
  /// \brief Constructor
  public: Entity();

  /// \brief Copy Constructor
  /// \param _other Other entity to copy from
  public: Entity(const Entity &_other);

  /// \brief Move constructor
  /// \param _other Other entity to move from
  public: Entity(Entity &&_entity) noexcept;

  /// \brief Constructor with id
  /// \param _id Id to set the entity to
  protected: explicit Entity(uint64_t _id);

  /// \brief Destructor
  public: ~Entity();

  /// \brief Assignment operator
  /// \param _other Other entity to copy from
  public: Entity &operator=(const Entity &_other);

  /// \brief Set the name of the entity
  /// \param _name Name of entity
  public: virtual void SetName(const std::string &_name);

  /// \brief Get the name of the entity
  /// \return Name of entity
  public: virtual std::string GetName() const;

  /// \brief Set the id of the entity
  /// \param _unique Id
  public: virtual void SetId(uint64_t _id);

  /// \brief Get the id of the entity
  /// \return Entity id
  public: virtual uint64_t GetId() const;

  /// \brief Set the pose of the entity
  /// \param _pose Pose of entity to set to
  public: virtual void SetPose(const math::Pose3d &_pose);

  /// \brief Get the pose of the entity
  /// \return Pose of entity to set to
  public: virtual math::Pose3d GetPose() const;

  /// \brief Get a child entity by id
  /// \param _id Id of child entity
  /// \return Child entity
  public: virtual Entity &GetChildById(uint64_t _id);

  /// \brief Get a child entity by name
  /// \param _name Name of child entity
  /// \return Child entity
  public: virtual Entity &GetChildByName(const std::string &_name);

  /// \brief Remove a child entity by id
  /// \param _id Id of child entity to remove
  public: virtual bool RemoveChildById(uint64_t _id);

  /// \brief Remove a child entity by name
  /// \param _name Name of child entity to remove
  /// \return True if child entity was removed, false otherwise
  public: virtual bool RemoveChildByName(const std::string &_name);

  /// \brief Remove a child entity by index
  /// \param _index Index of child entity to remove
  // public: virtual void RemoveChildByIndex(size_t _index);

  /// \brief Get number of children
  public: virtual size_t GetChildCount() const;

  /// \brief Get number of children
  /// \return Map of child id's to child entities
  protected: std::map<uint64_t, std::shared_ptr<Entity>> &GetChildren();

  /// \brief An invalid vertex.
  public: static Entity kNullEntity;

  /// \brief
  protected: static uint64_t GetNextId();

  /// \brief Entity id counter
  private: static uint64_t nextId;

  /// \brief Pointer to private data class
  private: EntityPrivate *dataPtr = nullptr;
};


}
}
}

#endif
