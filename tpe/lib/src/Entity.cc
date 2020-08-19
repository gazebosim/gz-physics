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

#include "Entity.hh"
#include "Utils.hh"

/// \brief Private data class for entity
class ignition::physics::tpelib::EntityPrivate
{
  /// \brief Name of entity
  public: std::string name;

  /// \brief Entity pose
  public: math::Pose3d pose;

  /// \brief Entity Id
  public: std::size_t id = 0u;

  /// \brief Child entities
  public: std::map<std::size_t, std::shared_ptr<Entity>> children;

  /// \brief Bounding Box
  public: math::AxisAlignedBox bbox;

  /// \brief Collide bitmask
  public: uint16_t collideBitmask = 0xFF;

  /// \brief Flag to indicate if bounding box changed
  public: bool bboxDirty = true;

  /// \brief Flag to indicate if pose changed
  public: bool poseDirty = false;

  /// \brief Flag to indicate if collide bitmask changed
  public: bool collideBitmaskDirty = true;

  /// \brief Parent of this entity
  public: Entity *parent = nullptr;
};

using namespace ignition;
using namespace physics;
using namespace tpelib;

std::size_t Entity::nextId = 0;
Entity Entity::kNullEntity = Entity(kNullEntityId);

//////////////////////////////////////////////////
Entity::Entity()
  : dataPtr(new EntityPrivate)
{
  this->dataPtr->id = Entity::GetNextId();
}

//////////////////////////////////////////////////
Entity::Entity(const Entity &_other)
  : dataPtr(new EntityPrivate)
{
  this->dataPtr->id = _other.dataPtr->id;
  this->dataPtr->name = _other.dataPtr->name;
  this->dataPtr->pose = _other.dataPtr->pose;
  this->dataPtr->children = _other.dataPtr->children;
  this->dataPtr->bbox = _other.dataPtr->bbox;
  this->dataPtr->collideBitmask = _other.dataPtr->collideBitmask;
}

//////////////////////////////////////////////////
Entity::Entity(Entity &&_other) noexcept
  : dataPtr(std::exchange(_other.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Entity &Entity::operator=(Entity &&_entity) noexcept = default;

//////////////////////////////////////////////////
Entity::Entity(std::size_t _id)
  : dataPtr(new EntityPrivate)
{
  this->dataPtr->id = _id;
}

//////////////////////////////////////////////////
Entity::~Entity()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Entity &Entity::operator=(const Entity &_other)
{
  this->dataPtr->children = _other.dataPtr->children;
  return *this;
}

//////////////////////////////////////////////////
void Entity::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
std::string Entity::GetName() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
const std::string &Entity::GetNameRef() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Entity::SetPose(const math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
  this->dataPtr->poseDirty = true;
}

//////////////////////////////////////////////////
math::Pose3d Entity::GetPose() const
{
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
math::Pose3d Entity::GetWorldPose() const
{
  if (this->dataPtr->parent)
    return this->dataPtr->parent->GetWorldPose() * this->dataPtr->pose;

  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
void Entity::SetId(std::size_t _id)
{
  this->dataPtr->id = _id;
}

//////////////////////////////////////////////////
std::size_t Entity::GetId() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
Entity &Entity::GetChildById(std::size_t _id) const
{
  auto it = this->dataPtr->children.find(_id);
  if (it != this->dataPtr->children.end())
  {
    return *it->second.get();
  }

  return kNullEntity;
}

//////////////////////////////////////////////////
Entity &Entity::GetChildByName(const std::string &_name) const
{
  for (auto it = this->dataPtr->children.begin();
      it != this->dataPtr->children.end(); ++it)
  {
    if (it->second->GetName() == _name)
    {
      return *it->second.get();
    }
  }

  return kNullEntity;
}

//////////////////////////////////////////////////
Entity &Entity::GetChildByIndex(unsigned int _index) const
{
  if (_index >= this->dataPtr->children.size())
    return kNullEntity;

  auto it = this->dataPtr->children.begin();
  std::advance(it, _index);
  if (it != this->dataPtr->children.end())
  {
    return *it->second.get();
  }

  return kNullEntity;
}

//////////////////////////////////////////////////
bool Entity::RemoveChildById(std::size_t _id)
{
  auto it = this->dataPtr->children.find(_id);
  if (it != this->dataPtr->children.end())
  {
    this->dataPtr->children.erase(it);
    this->ChildrenChanged();
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
bool Entity::RemoveChildByName(const std::string &_name)
{
  for (auto it = this->dataPtr->children.begin();
      it != this->dataPtr->children.end(); ++it)
  {
    if (it->second->GetName() == _name)
    {
      this->dataPtr->children.erase(it);
      this->ChildrenChanged();
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////
size_t Entity::GetChildCount() const
{
  return this->dataPtr->children.size();
}

//////////////////////////////////////////////////
math::AxisAlignedBox Entity::GetBoundingBox(bool _force)
{
  if (_force || this->dataPtr->bboxDirty)
  {
    this->UpdateBoundingBox(_force);
    this->dataPtr->bboxDirty = false;
  }
  return this->dataPtr->bbox;
}

//////////////////////////////////////////////////
void Entity::UpdateBoundingBox(bool _force)
{
  math::AxisAlignedBox box;
  for (auto &it : this->dataPtr->children)
  {
    auto transformedBox =
        transformAxisAlignedBox(it.second->GetBoundingBox(_force),
        it.second->GetPose());
    box.Merge(transformedBox);
  }

  this->dataPtr->bbox = box;
}

//////////////////////////////////////////////////
uint16_t Entity::GetCollideBitmask() const
{
  if (this->dataPtr->collideBitmaskDirty)
  {
    uint16_t mask = 0u;
    for (auto &it : this->dataPtr->children)
    {
      mask |= it.second->GetCollideBitmask();
    }
    this->dataPtr->collideBitmask = mask;
    this->dataPtr->collideBitmaskDirty = false;
  }

  return this->dataPtr->collideBitmask;
}

//////////////////////////////////////////////////
std::map<std::size_t, std::shared_ptr<Entity>> &Entity::GetChildren() const
{
  return this->dataPtr->children;
}

//////////////////////////////////////////////////
std::size_t Entity::GetNextId()
{
  return nextId++;
}

//////////////////////////////////////////////////
void Entity::ChildrenChanged()
{
  this->dataPtr->bboxDirty = true;
  this->dataPtr->collideBitmaskDirty = true;

  if (this->dataPtr->parent)
    this->dataPtr->parent->ChildrenChanged();
}

//////////////////////////////////////////////////
void Entity::SetParent(Entity *_parent)
{
  this->dataPtr->parent = _parent;
}

//////////////////////////////////////////////////
Entity *Entity::GetParent() const
{
  return this->dataPtr->parent;
}

//////////////////////////////////////////////////
bool Entity::PoseDirty() const
{
  return this->dataPtr->poseDirty = true;
}

//////////////////////////////////////////////////
void Entity::ClearPoseDirty()
{
  this->dataPtr->poseDirty = false;
}
