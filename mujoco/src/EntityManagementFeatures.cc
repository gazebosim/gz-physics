/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "EntityManagementFeatures.hh"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <gz/physics/Entity.hh>
#include <memory>
#include <sdf/Types.hh>
#include <string>

#include "Base.hh"

namespace gz
{
namespace physics
{
namespace mujoco
{

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldCount(
    const Identity & /*_engineID*/) const
{
  return this->worlds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(const Identity &,
                                            std::size_t _worldIndex) const
{
  if (this->worlds.indexInContainerToID.empty())
  {
    return this->GenerateInvalidId();
  }
  if (this->worlds.indexInContainerToID.begin()->second.size() <= _worldIndex)
  {
    return this->GenerateInvalidId();
  }
  const std::size_t id =
      this->worlds.indexInContainerToID.begin()->second[_worldIndex];
  return this->GenerateIdentity(id, this->worlds.idToObject.at(id));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(const Identity &,
                                            const std::string &_worldName) const
{
  const std::size_t id = this->worlds.IdentityOf(_worldName);
  return this->GenerateIdentity(id, this->worlds.idToObject.at(id));
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetWorldName(
    const Identity &_worldID) const
{
  return this->ReferenceInterface<WorldInfo>(_worldID)->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldIndex(
    const Identity &_worldID) const
{
  return this->worlds.idToIndexInContainer.at(_worldID);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetEngineOfWorld(
    const Identity & /*_worldID*/) const
{
  return this->GenerateIdentity(0);
}
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity & /*_engineID*/, const std::string &_name)
{
  auto worldInfo = std::make_shared<WorldInfo>();
  worldInfo->entityId = this->GetNextEntity();
  this->worlds.AddEntity(worldInfo->entityId, worldInfo, _name, 0);

  mjSpec *spec = mj_makeSpec();
  worldInfo->mjSpecObj = spec;
  worldInfo->mjSpecObj->option.timestep = 0.001;
  worldInfo->mjModelObj = mj_compile(spec, nullptr);
  worldInfo->mjDataObj = mj_makeData(worldInfo->mjModelObj);
  worldInfo->body = mjs_findBody(spec, "world");
  // We record the name of the world, but we don't change the name in the
  // worldbody so that it is easy to find it with mjs_findBody(s, "world")
  // elsewhere.
  worldInfo->name = _name;
  return this->GenerateIdentity(worldInfo->entityId, worldInfo);
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelCount(
    const Identity &_worldID) const
{
  const auto *worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  return worldInfo->models.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(const Identity &_worldID,
                                            std::size_t _modelIndex) const
{
  const auto *worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  const auto &models = worldInfo->models;
  if (models.indexInContainerToID.begin()->second.size() <= _modelIndex)
  {
    return this->GenerateInvalidId();
  }
  const std::size_t id =
      models.indexInContainerToID.begin()->second[_modelIndex];
  return this->GenerateIdentity(id, models.at(id));
}
/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(const Identity &_worldID,
                                            const std::string &_modelName) const
{
  const auto *worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  const std::string scopedName = this->JoinNames(worldInfo->name, _modelName);
  const std::size_t id = worldInfo->models.IdentityOf(scopedName);
  return this->GenerateIdentity(id, worldInfo->models.at(id));
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
  const auto *worldInfo =
      this->ReferenceInterface<ModelInfo>(_modelID)->worldInfo;
  return worldInfo->models.idToIndexInContainer.at(_modelID);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorldOfModel(
    const Identity &_modelID) const
{
  const auto *worldInfo =
      this->ReferenceInterface<ModelInfo>(_modelID)->worldInfo;
  auto worldInfoSharedPtr = this->worlds.at(worldInfo->entityId);
  return GenerateIdentity(worldInfoSharedPtr->entityId, worldInfoSharedPtr);
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->links.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(const Identity &_modelID,
                                           std::size_t _linkIndex) const
{
  const auto *model = this->ReferenceInterface<ModelInfo>(_modelID);
  if (_linkIndex >= model->links.indexInContainerToID.size())
  {
    return this->GenerateInvalidId();
  }

  const std::size_t id =
      model->links.indexInContainerToID.begin()->second[_linkIndex];
  return this->GenerateIdentity(id, model->links.at(id));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(const Identity &_modelID,
                                           const std::string &_linkName) const
{
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  const auto *child =
      mjs_findChild(modelInfo->parentBody,
                    ::sdf::JoinName(modelInfo->name, _linkName).c_str());

  if (!child)
  {
    return this->GenerateInvalidId();
  }

  auto linkInfo = modelInfo->links.at(child);
  if (!linkInfo)
  {
    return this->GenerateInvalidId();
  }

  return this->GenerateIdentity(linkInfo->entityId, linkInfo);
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
  const auto modelInfo =
      this->ReferenceInterface<LinkInfo>(_linkID)->modelInfo.lock();
  return modelInfo->links.idToIndexInContainer.at(_linkID);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfLink(const Identity &_linkID) const
{
  const auto modelInfo =
      this->ReferenceInterface<LinkInfo>(_linkID)->modelInfo.lock();

  return this->GenerateIdentity(modelInfo->entityId, modelInfo);
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &/* _modelID */)
{
  // TODO(azeey) Implement RemoveModel
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::ModelRemoved(
    const Identity &/* _modelID */) const
{
  // TODO(azeey) Implement RemoveModel
  return false;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeCount(
  const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->shapes.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, std::size_t _shapeIndex) const
{
  const auto linkInfo = this->ReferenceInterface<LinkInfo>(_linkID);
  if (_shapeIndex >= linkInfo->shapes.indexInContainerToID.size())
  {
    return this->GenerateInvalidId();
  }
  const std::size_t id =
      linkInfo->shapes.indexInContainerToID.begin()->second[_shapeIndex];
  return this->GenerateIdentity(id, linkInfo->shapes.at(id));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::string &_shapeName) const
{
  // TODO(azeey) Return an invalid ID here otherwise, gz-sim will incorrectly
  // assume the ConstructSdfCollision feature is implemented (bug).
  return this->GenerateInvalidId();
  const auto *linkInfo = this->ReferenceInterface<LinkInfo>(_linkID);
  auto bodyName = mjs_getName(linkInfo->body->element);
  if (!bodyName)
  {
    return this->GenerateInvalidId();
  }
  const auto *child = mjs_asGeom(mjs_findElement(linkInfo->worldInfo->mjSpecObj,
      mjOBJ_GEOM, JoinNames(*bodyName, _shapeName).c_str()));

  if (!child)
  {
    return this->GenerateInvalidId();
  }

  auto shapeInfo = linkInfo->shapes.at(child);
  if (!shapeInfo)
  {
    return this->GenerateInvalidId();
  }

  return this->GenerateIdentity(shapeInfo->entityId, shapeInfo);
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetShapeName(
  const Identity &_shapeID) const
{
  return this->ReferenceInterface<ShapeInfo>(_shapeID)->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeIndex(
  const Identity &_shapeID) const
{
  const auto linkInfo =
      this->ReferenceInterface<ShapeInfo>(_shapeID)->linkInfo.lock();
  return linkInfo->shapes.idToIndexInContainer.at(_shapeID);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLinkOfShape(
  const Identity &_shapeID) const
{
  auto linkInfo =
      this->ReferenceInterface<ShapeInfo>(_shapeID)->linkInfo.lock();
  if (!linkInfo)
    return this->GenerateInvalidId();
  return this->GenerateIdentity(linkInfo->entityId, linkInfo);
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(
    const Identity & /* _worldID */, std::size_t /* _modelIndex */)
{
  // TODO(azeey) Implement RemoveModelByIndex
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(
    const Identity & /* _worldID */, const std::string & /* _modelName  */)
{
  // TODO(azeey) Implement RemoveModelByName
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveNestedModelByIndex(
    const Identity &/* _modelID */, std::size_t /* _nestedModelIndex */)
{
  // TODO(azeey) Implement RemoveNestedModelByIndex
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveNestedModelByName(
    const Identity & /* _modelID */, const std::string & /* _modelName */)
{
  // TODO(azeey) Implement RemoveNestedModelByName
  return false;
}
}  // namespace mujoco
}  // namespace physics
}  // namespace gz
