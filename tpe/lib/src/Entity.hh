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

#ifndef IGNITION_PHYSICS_TPE_LIB_SRC_ENTITY_HH_
#define IGNITION_PHYSICS_TPE_LIB_SRC_ENTITY_HH_

#include <cstddef>
#include <map>
#include <memory>
#include <string>

#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Pose3.hh>
#include "ignition/physics/tpelib/Export.hh"

namespace ignition {
namespace physics {
namespace tpelib {

// forward declaration
class EntityPrivate;

/// \brief Represents an invalid Id.
static const std::size_t kNullEntityId = math::MAX_UI64;

/// \brief Entity class
class IGNITION_PHYSICS_TPELIB_VISIBLE Entity
{
  /// \brief Constructor
  public: Entity();

  /// \brief Copy Constructor
  /// \param[in] _other Other entity to copy from
  public: Entity(const Entity &_other);

  /// \brief Move constructor
  /// \param[in] _entity Other entity to move from
  public: Entity(Entity &&_entity) noexcept;

  /// \brief Move assignment operator
  /// \param[in] _entity Entity to move
  public: Entity &operator=(Entity &&_entity) noexcept;

  /// \brief Constructor with id
  /// \param[in] _id Id to set the entity to
  protected: explicit Entity(std::size_t _id);

  /// \brief Destructor
  public: ~Entity();

  /// \brief Assignment operator
  /// \param[in] _other Other entity to copy from
  public: Entity &operator=(const Entity &_other);

  /// \brief Set the name of the entity
  /// \param[in] _name Name of entity
  public: virtual void SetName(const std::string &_name);

  /// \brief Get the name of the entity
  /// \return Name of entity
  public: virtual std::string GetName() const;

  /// \brief Get a const reference to the name of the entity
  /// \return Name of entity
  public: virtual const std::string &GetNameRef() const;

  /// \brief Set the static property
  /// \param[in] _static static
  public: virtual void SetStatic(bool _static);

  /// \brief Get the static property
  /// \return Static property
  public: virtual bool GetStatic() const;

  /// \brief Set the id of the entity
  /// \param[in] _unique Id
  public: virtual void SetId(std::size_t _id);

  /// \brief Get the id of the entity
  /// \return Entity id
  public: virtual std::size_t GetId() const;

  /// \brief Set the pose of the entity
  /// \param[in] _pose Pose of entity to set to
  public: virtual void SetPose(const math::Pose3d &_pose);

  /// \brief Get the pose of the entity
  /// \return Pose of entity
  public: virtual math::Pose3d GetPose() const;

  /// \brief Get the world pose of the entity
  /// \return World pose of entity
  public: virtual math::Pose3d GetWorldPose() const;

  /// \brief Get a child entity by id
  /// \param[in] _id Id of child entity
  /// \return Child entity
  public: virtual Entity &GetChildById(std::size_t _id) const;

  /// \brief Get a child entity by name
  /// \param[in] _name Name of child entity
  /// \return Child entity
  public: virtual Entity &GetChildByName(const std::string &_name) const;

  /// \brief Get a child entity by index
  /// \param[in] _index Index of child entity
  /// \return Child entity
  public: virtual Entity &GetChildByIndex(unsigned int _index) const;

  /// \brief Remove a child entity by id
  /// \param[in] _id Id of child entity to remove
  public: virtual bool RemoveChildById(std::size_t _id);

  /// \brief Remove a child entity by name
  /// \param[in] _name Name of child entity to remove
  /// \return True if child entity was removed, false otherwise
  public: virtual bool RemoveChildByName(const std::string &_name);

  /// \brief Remove a child entity by index
  /// \param[in] _index Index of child entity to remove
  // public: virtual void RemoveChildByIndex(size_t _index);

  /// \brief Get number of children
  /// \return Number of children
  public: virtual size_t GetChildCount() const;

  /// \brief Get bounding box of entity
  /// \param[in] _force True to force update bounding box
  /// \return Entity bounding box
  public: virtual math::AxisAlignedBox GetBoundingBox(bool _force = false);

  /// \brief Get collide bitmask
  /// \return Collision's collide bitmask
  public: virtual uint16_t GetCollideBitmask() const;

  /// \internal
  /// \brief Set the parent of this entity.
  /// \param[in] _parent Parent to set to
  public: void SetParent(Entity *_parent);

  /// \internal
  /// \brief Get the parent of this entity.
  /// \return Parent of this entity
  public: Entity *GetParent() const;

  /// \internal
  /// \brief Get whether the pose has changed
  /// \return True if pose has changed, false otherwise
  public: bool PoseDirty() const;

  /// \internal
  /// \brief Reset the pose dirty flag
  public: void ResetPoseDirty();

  /// \internal
  /// \brief Mark that the children of the entity has changed, e.g. a child
  /// entity is added or removed, or child entity properties changed.
  public: void ChildrenChanged();

  /// \brief Get number of children
  /// \return Map of child id's to child entities
  public: std::map<std::size_t, std::shared_ptr<Entity>> &GetChildren()
      const;

  /// \brief Update the entity bounding box
  /// \param[in] _force True to force update children's bounding box
  private: virtual void UpdateBoundingBox(bool _force = false);

  /// \brief An invalid vertex.
  public: static Entity kNullEntity;

  /// \brief Get the id of next entity
  /// \return size_t id of next entity
  protected: static std::size_t GetNextId();

  /// \brief Entity id counter
  private: static std::size_t nextId;

  /// \brief Pointer to private data class
  private: EntityPrivate *dataPtr = nullptr;
};

}
}
}

#endif
