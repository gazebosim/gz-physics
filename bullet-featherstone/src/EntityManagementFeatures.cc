/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <btBulletDynamicsCommon.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "EntityManagementFeatures.hh"
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->linkEntityIds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
  const Identity &_modelID, std::size_t _linkIndex) const
{
  const auto *model = this->ReferenceInterface<ModelInfo>(_modelID);
  if (_linkIndex >= model->linkEntityIds.size())
    return this->GenerateInvalidId();

  const auto linkID = model->linkEntityIds[_linkIndex];
  return this->GenerateIdentity(linkID, this->links.at(linkID));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, const std::string &_linkName) const
{
  // TODO(MXG): Consider using a hashmap with the link names as a key to speed
  // this up
  const auto *model = this->ReferenceInterface<ModelInfo>(_modelID);
  const auto it = model->linkNameToEntityId.find(_linkName);
  if (it == model->linkNameToEntityId.end())
    return this->GenerateInvalidId();

  return this->GenerateIdentity(it->second, this->links.at(it->second));
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetLinkName(
    const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkIndex(
    const Identity &_linkID) const
{
  // The root link does not have an index, so we give it an index of 0 and bump
  // the rest up by one when providing an index to gazebo
  const auto index = this->ReferenceInterface<LinkInfo>(_linkID)->indexInModel;
  if (index.has_value())
    return *index+1;

  return 0;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfLink(const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->model;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetJointCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->jointEntityIds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetJoint(
  const Identity &_modelID,
  std::size_t _jointIndex) const
{
  const auto *model = this->ReferenceInterface<ModelInfo>(_modelID);
  if (_jointIndex >= model->jointEntityIds.size())
    return this->GenerateInvalidId();

  const auto jointID = model->jointEntityIds[_jointIndex];
  return this->GenerateIdentity(jointID, this->joints.at(jointID));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetJoint(
  const Identity &_modelID,
  const std::string &_jointName) const
{
  const auto *model = this->ReferenceInterface<ModelInfo>(_modelID);
  const auto it = model->jointNameToEntityId.find(_jointName);
  if (it == model->jointNameToEntityId.end())
    return this->GenerateInvalidId();

  return this->GenerateIdentity(it->second, this->joints.at(it->second));
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetJointName(
  const Identity &_jointID) const
{
  return this->ReferenceInterface<JointInfo>(_jointID)->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetJointIndex(
  const Identity &_jointID) const
{
  return this->ReferenceInterface<JointInfo>(_jointID)->indexInGzModel;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfJoint(
  const Identity &_jointID) const
{
  return this->ReferenceInterface<JointInfo>(_jointID)->model;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeCount(
  const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->collisionEntityIds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, std::size_t _shapeIndex) const
{
  const auto *link = this->ReferenceInterface<LinkInfo>(_linkID);
  if (_shapeIndex >= link->collisionEntityIds.size())
    return this->GenerateInvalidId();

  const auto shapeID = link->collisionEntityIds[_shapeIndex];
  return this->GenerateIdentity(shapeID, this->collisions.at(shapeID));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::string &_shapeName) const
{
  const auto *link = this->ReferenceInterface<LinkInfo>(_linkID);
  const auto it = link->collisionNameToEntityId.find(_shapeName);
  if (it == link->collisionNameToEntityId.end())
    return this->GenerateInvalidId();

  return this->GenerateIdentity(it->second, this->collisions.at(it->second));
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetShapeName(
  const Identity &_shapeID) const
{
  return this->ReferenceInterface<CollisionInfo>(_shapeID)->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeIndex(
  const Identity &_shapeID) const
{
  return this->ReferenceInterface<CollisionInfo>(_shapeID)->indexInLink;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLinkOfShape(
  const Identity &_shapeID) const
{
  return this->ReferenceInterface<CollisionInfo>(_shapeID)->link;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  // Create bullet empty multibody dynamics world
  return this->AddWorld(WorldInfo(_name));
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  auto *model = this->ReferenceInterface<ModelInfo>(_modelID);
  auto *world = this->ReferenceInterface<WorldInfo>(model->world);
  if (world->modelIndexToEntityId.erase(model->indexInWorld) == 0)
  {
    // The model has already been removed at some point.
    return false;
  }

  world->modelNameToEntityId.erase(model->name);

  // Remove all constraints related to this model
  for (auto constraint_index : model->external_constraints)
  {
    const auto joint = this->joints.at(constraint_index);
    const auto &constraint =
      std::get<std::unique_ptr<btMultiBodyConstraint>>(joint->identifier);
    world->world->removeMultiBodyConstraint(constraint.get());
    this->joints.erase(constraint_index);
  }

  world->world->removeMultiBody(model->body.get());
  for (const auto linkID : model->linkEntityIds)
  {
    const auto &link = this->links.at(linkID);
    if (link->collider)
    {
      world->world->removeCollisionObject(link->collider.get());
      for (const auto shapeID : link->collisionEntityIds)
        this->collisions.erase(shapeID);
    }

    this->links.erase(linkID);
  }

  for (const auto jointID : model->jointEntityIds)
    this->joints.erase(jointID);

  this->models.erase(_modelID);
  return true;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::ModelRemoved(
    const Identity &_modelID) const
{
  auto *model = this->ReferenceInterface<ModelInfo>(_modelID);
  auto *world = this->ReferenceInterface<WorldInfo>(model->world);
  return world->modelIndexToEntityId.count(model->indexInWorld) == 0;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(
    const Identity & _worldID, std::size_t _modelIndex)
{
  auto *world = this->ReferenceInterface<WorldInfo>(_worldID);
  const auto it = world->modelIndexToEntityId.find(_modelIndex);
  if (it == world->modelIndexToEntityId.end())
    return false;

  return this->RemoveModel(
    this->GenerateIdentity(it->second, this->models.at(it->second)));
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(
    const Identity & _worldID, const std::string & _modelName )
{
  // Check if there is a model with the requested name
  auto *world = this->ReferenceInterface<WorldInfo>(_worldID);
  const auto it = world->modelNameToEntityId.find(_modelName);
  if (it == world->modelNameToEntityId.end())
    return false;

  return this->RemoveModel(
    this->GenerateIdentity(it->second, this->models.at(it->second)));
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetEngineName(
  const Identity &) const
{
  static const std::string engineName = "bullet-featherstone";
  return engineName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetEngineIndex(const Identity &) const
{
  return 0;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldCount(
    const Identity &) const
{
  return worlds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
    const Identity &, const std::size_t _requestedWorldIndex) const
{
  // _worldIndex is not the same as a WorldID. The value of _worldIndex should
  // range from 0 to GetWorldCount()-1. The most efficient implementation
  // would be to maintain a std::vector of WorldIDs, but then we'd have to
  // manage that data when worlds are added and removed.
  std::size_t currentWorldIndex = 0;
  for (const auto &[worldID, world] : this->worlds)
  {
    if (currentWorldIndex == _requestedWorldIndex)
      return this->GenerateIdentity(worldID, world);
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
    const Identity &, const std::string &_requestedWorldName) const
{
  // We could speed this up by maintaining a hashmap from world name to world ID
  for (const auto &[worldID, world] : this->worlds)
  {
    if (world->name == _requestedWorldName)
      return this->GenerateIdentity(worldID, world);
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetWorldName(
    const Identity &_worldID) const
{
  return this->ReferenceInterface<WorldInfo>(_worldID)->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldIndex(
    const Identity &) const
{
  return 0;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetEngineOfWorld(
    const Identity &) const
{
  return this->GenerateIdentity(0);
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelCount(
    const Identity &) const
{
  return this->models.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
    const Identity &_worldID, std::size_t _modelIndex) const
{
  const auto *world = this->ReferenceInterface<WorldInfo>(_worldID);
  const auto it = world->modelIndexToEntityId.find(_modelIndex);
  if (it == world->modelIndexToEntityId.end())
    return this->GenerateInvalidId();

  return this->GenerateIdentity(it->second, this->models.at(it->second));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
    const Identity &_worldID, const std::string &_modelName) const
{
  const auto *world = this->ReferenceInterface<WorldInfo>(_worldID);
  const auto it = world->modelNameToEntityId.find(_modelName);
  if (it == world->modelNameToEntityId.end())
    return this->GenerateInvalidId();

  return this->GenerateIdentity(it->second, this->models.at(it->second));
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetModelName(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelIndex(
    const Identity &_modelID) const
{
  // The root link does not have an index, so we give it an index of 0 and bump
  // the rest up by one when providing an index to gazebo
  const auto index = this->ReferenceInterface<ModelInfo>(
    _modelID)->indexInWorld;
  return index+1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorldOfModel(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->world;
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
