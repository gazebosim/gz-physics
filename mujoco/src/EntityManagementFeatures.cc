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
  const auto &worldInfo = this->worlds[_worldIndex];
  return this->GenerateIdentity(worldInfo->entityId, worldInfo);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(const Identity &,
                                            const std::string &_worldName) const
{
  // TODO(azeey) Get world by name
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
    const Identity &_worldID) const
{
  // TODO(azeey) Implement GetWorldIndex
  return 0;
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
  this->worlds.push_back(worldInfo);

  mjSpec *spec = mj_makeSpec();
  worldInfo->mjSpecObj = spec;
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
  if (_modelIndex < worldInfo->models.size())
  {
    auto modelInfo = worldInfo->models[_modelIndex];
    return this->GenerateIdentity(modelInfo->entityId, modelInfo);
  }
  return this->GenerateInvalidId();
}
/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(const Identity &_worldID,
                                            const std::string &_modelName) const
{
  // TODO(azeey): Imeplement GetModel
  return this->GenerateInvalidId();
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
  // TODO(azeey): Implement GetModelIndex
  return 0;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorldOfModel(
    const Identity &_modelID) const
{
  // TODO(azeey) Implement GetWorldOfModel
  return this->GenerateInvalidId();
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
  if (_linkIndex >= model->links.size())
    return this->GenerateInvalidId();

  const auto linkInfo = model->links[_linkIndex];
  return this->GenerateIdentity(linkInfo->entityId, linkInfo);
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

  auto linkInfo = modelInfo->LinkFromBody(child);
  if (!linkInfo) {
    return this->GenerateInvalidId();
  }

  return this->GenerateIdentity(linkInfo->entityId, linkInfo);
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetLinkName(
    const Identity &_linkID) const
{
  // TODO(azeey) Implement GetLinkName
  static std::string name = "";
  return name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkIndex(
    const Identity &_linkID) const
{
  // TODO(azeey) Implement GetLinkIndex
  return 0;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfLink(const Identity &_linkID) const
{
  // TODO(azeey) Implement GetModelOfLink
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  // TODO(azeey) Implement RemoveModel
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::ModelRemoved(
    const Identity &_modelID) const
{
  // TODO(azeey) Implement RemoveModel
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(
    const Identity & _worldID, std::size_t _modelIndex)
{
  // TODO(azeey) Implement RemoveModelByIndex
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(
    const Identity & _worldID, const std::string & _modelName )
{
  // TODO(azeey) Implement RemoveModelByName
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveNestedModelByIndex(
    const Identity &_modelID, std::size_t _nestedModelIndex)
{
  // TODO(azeey) Implement RemoveNestedModelByIndex
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveNestedModelByName(const Identity &_modelID,
    const std::string &_modelName)
{
  // TODO(azeey) Implement RemoveNestedModelByName
  return false;
}
}  // namespace mujoco
}  // namespace physics
}  // namespace gz
