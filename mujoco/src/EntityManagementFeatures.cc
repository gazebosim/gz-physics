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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gz/physics/Entity.hh>
#include <sdf/Types.hh>

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
  // The Mujoco docs recommend the implicitfast integrator
  worldInfo->mjSpecObj->option.integrator = mjtIntegrator::mjINT_IMPLICITFAST;
  worldInfo->mjSpecObj->compiler.degree = false;
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
  // GetModelCount should only return the number of top-level models that are
  // direct children of the world. So, we cannot simply return
  // worldInfo->models.size(), because that returns the total count of all
  // models (including recursively nested submodels).
  if (worldInfo->models.size() == 0)
  {
    return 0u;
  }
  return worldInfo->models.indexInContainerToID.at(_worldID).size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(const Identity &_worldID,
                                            std::size_t _modelIndex) const
{
  const auto *worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  const auto &models = worldInfo->models;
  const auto &indexInContainerToID =
      models.indexInContainerToID.at(_worldID);
  if (indexInContainerToID.size() <= _modelIndex)
  {
    return this->GenerateInvalidId();
  }
  const std::size_t id = indexInContainerToID[_modelIndex];
  return this->GenerateIdentity(id, models.at(id));
}
/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(const Identity &_worldID,
                                            const std::string &_modelName) const
{
  const auto *worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  const std::string scopedName = this->JoinNames(worldInfo->name, _modelName);
  if (!worldInfo->models.HasEntity(scopedName))
  {
    return this->GenerateInvalidId();
  }
  const std::size_t id = worldInfo->models.IdentityOf(scopedName);
  return this->GenerateIdentity(id, worldInfo->models.at(id));
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetModelName(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->localName;
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
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  if (modelInfo->links.indexInContainerToID.at(_modelID).size() <= _linkIndex)
  {
    return this->GenerateInvalidId();
  }

  const std::size_t id =
      modelInfo->links.indexInContainerToID.at(_modelID)[_linkIndex];
  return this->GenerateIdentity(id, modelInfo->links.at(id));
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
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  if (this->ModelRemoved(_modelID))
  {
    return false;
  }

  const auto *worldInfo =
      this->ReferenceInterface<ModelInfo>(_modelID)->worldInfo;
  return this->RemoveModelImpl(worldInfo->entityId, _modelID);
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::ModelRemoved(const Identity &_modelID) const
{
  const auto *worldInfo =
      this->ReferenceInterface<ModelInfo>(_modelID)->worldInfo;
  return !worldInfo->models.HasEntity(_modelID);
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetJointCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->joints.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetJoint(
    const Identity &_modelID, const std::size_t _jointIndex) const
{
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  if (modelInfo->joints.indexInContainerToID.at(_modelID).size() <= _jointIndex)
  {
    return this->GenerateInvalidId();
  }

  const std::size_t id =
      modelInfo->joints.indexInContainerToID.at(_modelID)[_jointIndex];
  return this->GenerateIdentity(id, modelInfo->joints.at(id));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetJoint(
    const Identity &_modelID, const std::string &_jointName) const
{
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  if (modelInfo->joints.HasEntity(_jointName))
  {
    auto jointInfo = modelInfo->joints.at(_jointName);
    return this->GenerateIdentity(jointInfo->entityId, jointInfo);
  }

  return this->GenerateInvalidId();
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
  const auto modelInfo =
      this->ReferenceInterface<JointInfo>(_jointID)->modelInfo.lock();
  return modelInfo->joints.idToIndexInContainer.at(_jointID);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfJoint(
    const Identity &_jointID) const
{
  auto modelInfo =
      this->ReferenceInterface<JointInfo>(_jointID)->modelInfo.lock();
  if (modelInfo)
  {
    return this->GenerateIdentity(modelInfo->entityId, modelInfo);
  }
  return this->GenerateInvalidId();
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
  if (linkInfo->shapes.indexInContainerToID.at(_linkID).size() <= _shapeIndex)
  {
    return this->GenerateInvalidId();
  }
  const std::size_t id =
      linkInfo->shapes.indexInContainerToID.at(_linkID)[_shapeIndex];
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
void EntityManagementFeatures::SetCollisionFilterMask(
    const Identity &_shapeID, uint16_t _mask)
{
  auto shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  if (shapeInfo && shapeInfo->geom)
  {
    shapeInfo->geom->conaffinity = static_cast<int>(_mask);

    // For backward compatibility, if category bitmask is not set, it
    // defaults to the same value as collide bitmask. So update contype
    // as well
    if (!shapeInfo->categoryMask.has_value())
    {
      shapeInfo->geom->contype = static_cast<int>(_mask);
    }

    // Update the model directly for immediate effect
    if (shapeInfo->worldInfo)
    {
      if (shapeInfo->worldInfo->mjModelObj)
      {
        int geomId = mjs_getId(shapeInfo->geom->element);
        if (geomId >= 0 && geomId < shapeInfo->worldInfo->mjModelObj->ngeom)
        {
          shapeInfo->worldInfo->mjModelObj->geom_conaffinity[geomId] =
              static_cast<int>(_mask);
          if (!shapeInfo->categoryMask.has_value())
          {
            shapeInfo->worldInfo->mjModelObj->geom_contype[geomId] =
                static_cast<int>(_mask);
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
uint16_t EntityManagementFeatures::GetCollisionFilterMask(
    const Identity &_shapeID) const
{
  auto shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  return static_cast<uint16_t>(shapeInfo->geom->conaffinity);
}

/////////////////////////////////////////////////
void EntityManagementFeatures::RemoveCollisionFilterMask(
    const Identity &_shapeID)
{
  this->SetCollisionFilterMask(_shapeID, std::numeric_limits<uint16_t>::max());
}

/////////////////////////////////////////////////
void EntityManagementFeatures::SetCategoryFilterMask(
    const Identity &_shapeID, uint16_t _mask)
{
  auto shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  if (shapeInfo && shapeInfo->geom)
  {
    shapeInfo->categoryMask = _mask;
    shapeInfo->geom->contype = static_cast<int>(_mask);

    // Update the model directly for immediate effect
    if (shapeInfo->worldInfo)
    {
      if (shapeInfo->worldInfo->mjModelObj)
      {
        int geomId = mjs_getId(shapeInfo->geom->element);
        if (geomId >= 0 && geomId < shapeInfo->worldInfo->mjModelObj->ngeom)
        {
          shapeInfo->worldInfo->mjModelObj->geom_contype[geomId] =
              static_cast<int>(_mask);
        }
      }
    }
  }
}

/////////////////////////////////////////////////
uint16_t EntityManagementFeatures::GetCategoryFilterMask(
    const Identity &_shapeID) const
{
  auto shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  if (shapeInfo && shapeInfo->geom)
  {
    return static_cast<uint16_t>(shapeInfo->geom->contype);
  }
  return std::numeric_limits<uint16_t>::max();
}

/////////////////////////////////////////////////
void EntityManagementFeatures::RemoveCategoryFilterMask(
    const Identity &_shapeID)
{
  auto shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  if (!shapeInfo || !shapeInfo->geom)
    return;
  // Removing categority bitmask means setting it to the same value as
  // collide bitmask
  // Do this by calling SetCategoryFilterMask so both the spec and the model
  // are updated. Make sure to reset the categoryMask optional var afterwards.
  this->SetCategoryFilterMask(
      _shapeID, static_cast<uint16_t>(shapeInfo->geom->conaffinity));
  shapeInfo->categoryMask.reset();
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(
    const Identity &_worldID, std::size_t _modelIndex)
{
  const auto *worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  auto it = worldInfo->models.indexInContainerToID.find(_worldID);
  if (it == worldInfo->models.indexInContainerToID.end() ||
      _modelIndex >= it->second.size())
  {
    return false;
  }

  const std::size_t modelId = it->second[_modelIndex];
  return this->RemoveModelImpl(_worldID, modelId);
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(
    const Identity &_worldID, const std::string &_modelName)
{
  const std::size_t modelId = this->GetModel(_worldID, _modelName);
  return this->RemoveModelImpl(_worldID.id, modelId);
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

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelImpl(const std::size_t _worldID,
                                               const std::size_t _modelID)
{
  auto worldInfo = this->worlds.at(_worldID);
  if (!worldInfo || !worldInfo->models.HasEntity(_modelID))
  {
    return false;
  }

  auto modelInfo = worldInfo->models.at(_modelID);
  const std::string scopedName =
      this->JoinNames(worldInfo->name, modelInfo->name);

  // Recursively remove nested models first.
  // Copy entity IDs to a vector to avoid iterator invalidation when nested
  // models erase themselves from parentModelInfo->nestedModelNameToEntityId.
  std::vector<std::size_t> nestedEntityIds;
  nestedEntityIds.reserve(modelInfo->nestedModelNameToEntityId.size());
  for (const auto &[nestedName, nestedEntityId] :
       modelInfo->nestedModelNameToEntityId)
  {
    nestedEntityIds.push_back(nestedEntityId);
  }

  for (std::size_t nestedEntityId : nestedEntityIds)
  {
    this->RemoveModelImpl(_worldID, nestedEntityId);
  }
  modelInfo->nestedModelNameToEntityId.clear();

  // If this is a nested model, remove it from its parent model's nested map
  if (modelInfo->parentModelInfo)
  {
    modelInfo->parentModelInfo->nestedModelNameToEntityId.erase(
        modelInfo->localName);
  }

  for (const auto &[linkId, linkInfo] : modelInfo->links.idToObject)
  {
    for (const auto &[shapeId, shapeInfo] : linkInfo->shapes.idToObject)
    {
      this->frames.erase(shapeId);
    }
    this->frames.erase(linkId);
  }

  for (const auto &[jointId, jointInfo] : modelInfo->joints.idToObject)
  {
    this->frames.erase(jointId);
  }

  this->frames.erase(_modelID);

  // Remove the body from mjSpec
  if (modelInfo->body)
  {
    mjs_delete(worldInfo->mjSpecObj, modelInfo->body->element);
  }

  worldInfo->models.RemoveEntity(scopedName);
  worldInfo->specDirty = true;

  return true;
}
}  // namespace mujoco
}  // namespace physics
}  // namespace gz
