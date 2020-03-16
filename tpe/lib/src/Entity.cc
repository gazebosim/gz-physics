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


#include <ignition/common/Console.hh>

#include "Entity.hh"

/// \brief Private data class for entity
class ignition::physics::tpe::lib::EntityPrivate
{
  /// \brief Name of entity
  public: std::string name;

  /// \brief Entity pose
  public: math::Pose3d pose;

  /// \brief Entity Id
  public: uint64_t id = 0u;

  /// \brief Child entities
  public: std::map<uint64_t, std::shared_ptr<Entity>> children;
};

using namespace ignition;
using namespace physics;
using namespace tpe;
using namespace lib;

uint64_t Entity::nextId = 0;
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
}

//////////////////////////////////////////////////
Entity::Entity(Entity &&_other) noexcept
  : dataPtr(std::exchange(_other.dataPtr, nullptr))
{
}

//////////////////////////////////////////////////
Entity::Entity(uint64_t _id)
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
  return *this = Entity(_other);
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
void Entity::SetPose(const math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

//////////////////////////////////////////////////
math::Pose3d Entity::GetPose() const
{
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
void Entity::SetId(uint64_t _id)
{
  this->dataPtr->id = _id;
}

//////////////////////////////////////////////////
uint64_t Entity::GetId() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
Entity &Entity::GetChildById(uint64_t _id)
{
  auto it = this->dataPtr->children.find(_id);
  if (it != this->dataPtr->children.end())
  {
    return *it->second.get();
  }

  return kNullEntity;
}

//////////////////////////////////////////////////
Entity &Entity::GetChildByName(const std::string &_name)
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
bool Entity::RemoveChildById(uint64_t _id)
{
  auto it = this->dataPtr->children.find(_id);
  if (it != this->dataPtr->children.end())
  {
    this->dataPtr->children.erase(it);
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
std::map<uint64_t, std::shared_ptr<Entity>> &Entity::GetChildren()
{
  return this->dataPtr->children;
}

//////////////////////////////////////////////////
uint64_t Entity::GetNextId()
{
  return nextId++;
}
