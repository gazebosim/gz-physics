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

#include <string>

#include "EntityManagementFeatures.hh"

using namespace ignition;
using namespace physics;
using namespace tpeplugin;

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetEngineName(
  const Identity &) const
{
  // engine name should not change
  static const std::string engineName = "tpe";
  return engineName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetEngineIndex(
  const Identity &) const
{
  return 0;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldCount(
  const Identity &) const
{
  // should always be 1
  return this->worlds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
  const Identity &, std::size_t _worldIndex) const
{
  auto it = this->worlds.begin();
  std::advance(it, _worldIndex);
  if (it != this->worlds.end() && it->second != nullptr)
  {
    return this->GenerateIdentity(it->first, it->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
  const Identity &, const std::string &_worldName) const
{
  for (auto it = this->worlds.begin(); it != this->worlds.end(); ++it)
  {
    if (it->second != nullptr)
    {
      if (it->second->world->GetName() == _worldName)
      {
        return this->GenerateIdentity(it->first, it->second);
      }
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetWorldName(
  const Identity &_worldID) const
{
  return this->ReferenceInterface<WorldInfo>(_worldID)->world->GetNameRef();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldIndex(
  const Identity &_worldID) const
{
  // index should be 0 assuming there's only one world
  return this->idToIndexInContainer(_worldID.id);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetEngineOfWorld(
  const Identity &) const
{
  return this->GenerateIdentity(0);
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelCount(
  const Identity &_worldID) const
{
  return this->ReferenceInterface<WorldInfo>(_worldID)->world->GetChildCount();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
  const Identity &_worldID, const std::size_t _modelIndex) const
{
  std::size_t modelId = this->indexInContainerToId(_worldID.id, _modelIndex);
  auto it = this->models.find(modelId);
  if (it != this->models.end() && it->second != nullptr)
  {
    return this->GenerateIdentity(modelId, it->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
  const Identity &_worldID, const std::string &_modelName) const
{
  auto worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  if (worldInfo != nullptr)
  {
    tpelib::Entity &modelEnt = worldInfo->world->GetChildByName(_modelName);
    for (auto it = this->models.begin(); it != this->models.end(); ++it)
    {
      if (it->second != nullptr)
      {
        std::string name = it->second->model->GetName();
        if (it->first == modelEnt.GetId() && name == modelEnt.GetName())
        {
          return this->GenerateIdentity(it->first, it->second);
        }
      }
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetModelName(
  const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->model->GetNameRef();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelIndex(
  const Identity &_modelID) const
{
  return this->idToIndexInContainer(_modelID.id);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorldOfModel(
  const Identity &_modelID) const
{
  auto it = this->childIdToParentId.find(_modelID.id);
  if (it != this->childIdToParentId.end())
  {
    auto worldIt = this->worlds.find(it->second);
    if (worldIt != this->worlds.end() && worldIt->second != nullptr)
    {
      return this->GenerateIdentity(it->second, worldIt->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkCount(
  const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->model->GetChildCount();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
  const Identity &_modelID, const std::size_t _linkIndex) const
{
  std::size_t linkId = this->indexInContainerToId(_modelID.id, _linkIndex);
  auto it = this->links.find(linkId);
  if (it != this->links.end() && it->second != nullptr)
  {
    return this->GenerateIdentity(it->first, it->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
  const Identity &_modelID, const std::string &_linkName) const
{
  auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  if (modelInfo != nullptr)
  {
    tpelib::Entity &linkEnt = modelInfo->model->GetChildByName(_linkName);
    for (auto it = this->links.begin(); it != this->links.end(); ++it)
    {
      if (it->second != nullptr)
      {
        std::string name = it->second->link->GetName();
        if (it->first == linkEnt.GetId() && name == linkEnt.GetName())
        {
          return this->GenerateIdentity(it->first, it->second);
        }
      }
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetLinkName(
  const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->link->GetNameRef();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkIndex(
  const Identity &_linkID) const
{
  return this->idToIndexInContainer(_linkID.id);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfLink(
  const Identity &_linkID) const
{
  auto it = this->childIdToParentId.find(_linkID.id);
  if (it != this->childIdToParentId.end())
  {
    auto modelIt = this->models.find(it->second);
    if (modelIt != this->models.end() && modelIt->second != nullptr)
    {
      return this->GenerateIdentity(it->second, modelIt->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeCount(
  const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->link->GetChildCount();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::size_t _shapeIndex) const
{
  std::size_t shapeId = this->indexInContainerToId(_linkID.id, _shapeIndex);
  auto it = this->collisions.find(shapeId);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    return this->GenerateIdentity(it->first, it->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::string &_shapeName) const
{
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_linkID);
  if (linkInfo != nullptr)
  {
    tpelib::Entity &shapeEnt = linkInfo->link->GetChildByName(_shapeName);
    for (auto it = this->collisions.begin();
         it != this->collisions.end();
         ++it)
    {
      if (it->second != nullptr)
      {
        std::string name = it->second->collision->GetName();
        if (it->first == shapeEnt.GetId() && name == shapeEnt.GetName())
        {
          return this->GenerateIdentity(it->first, it->second);
        }
      }
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetShapeName(
  const Identity &_shapeID) const
{
  return this->ReferenceInterface<CollisionInfo>(
      _shapeID)->collision->GetNameRef();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeIndex(
  const Identity &_shapeID) const
{
  return this->idToIndexInContainer(_shapeID.id);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLinkOfShape(
  const Identity &_shapeID) const
{
  auto it = this->childIdToParentId.find(_shapeID.id);
  if (it != this->childIdToParentId.end())
  {;
    auto linkIt = this->links.find(it->second);
    if (linkIt != this->links.end() && linkIt->second != nullptr)
    {
      return this->GenerateIdentity(it->second, linkIt->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(
  const Identity &_worldID, std::size_t _modelIndex)
{
  auto worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  if (worldInfo != nullptr)
  {
    auto modelId = this->indexInContainerToId(_worldID.id, _modelIndex);
    if (this->models.find(modelId) != this->models.end())
    {
      this->models.erase(modelId);
      this->childIdToParentId.erase(modelId);
      return worldInfo->world->RemoveChildById(modelId);
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(
  const Identity &_worldID, const std::string &_modelName)
{
  auto worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  if (worldInfo != nullptr)
  {
    std::size_t modelId =
      worldInfo->world->GetChildByName(_modelName).GetId();
    this->models.erase(modelId);
    this->childIdToParentId.erase(modelId);
    return worldInfo->world->RemoveChildById(modelId);
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  auto it = this->childIdToParentId.find(_modelID.id);
  if (it != this->childIdToParentId.end())
  {
    auto worldIt = this->worlds.find(it->second);
    if (worldIt != this->worlds.end() && worldIt->second != nullptr)
    {
      this->models.erase(_modelID.id);
      this->childIdToParentId.erase(_modelID.id);
      return worldIt->second->world->RemoveChildById(_modelID.id);
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::ModelRemoved(const Identity &_modelID) const
{
  if (this->models.find(_modelID.id) == this->models.end()
    && this->childIdToParentId.find(_modelID.id) ==
      this->childIdToParentId.end())
        return true;
  return false;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
  const Identity &, const std::string &_name)
{
  auto world = std::make_shared<tpelib::World>();
  world->SetName(_name);
  return this->AddWorld(world);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyModel(
  const Identity &_worldID, const std::string &_name)
{
  auto worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  if (worldInfo != nullptr)
  {
    auto &modelEnt = worldInfo->world->AddModel();
    modelEnt.SetName(_name);
    tpelib::Model *model = static_cast<tpelib::Model *>(&modelEnt);
    return this->AddModel(_worldID.id, *model);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyLink(
  const Identity &_modelID, const std::string &_name)
{
  auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  if (modelInfo != nullptr)
  {
    auto &linkEnt = modelInfo->model->AddLink();
    linkEnt.SetName(_name);
    tpelib::Link *link = static_cast<tpelib::Link *>(&linkEnt);
    return this->AddLink(_modelID.id, *link);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void EntityManagementFeatures::SetCollisionFilterMask(
    const Identity &_shapeID, const uint16_t _mask)
{
/*  const auto shapeNode = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  const std::size_t worldID = GetWorldOfShapeNode(this, shapeNode);
  const auto filterPtr = GetFilterPtr(this, worldID);
  filterPtr->SetIgnoredCollision(shapeNode, _mask);
*/
  auto collision =
      this->ReferenceInterface<CollisionInfo>(_shapeID)->collision;
  collision->SetCollideBitmask(_mask);
}

/////////////////////////////////////////////////
uint16_t EntityManagementFeatures::GetCollisionFilterMask(
    const Identity &_shapeID) const
{
/*
  const auto shapeNode = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  const std::size_t worldID = GetWorldOfShapeNode(this, shapeNode);
  const auto filterPtr = GetFilterPtr(this, worldID);
  return filterPtr->GetIgnoredCollision(shapeNode);
*/
  const auto collision =
      this->ReferenceInterface<CollisionInfo>(_shapeID)->collision;
  return collision->GetCollideBitmask();
}

/////////////////////////////////////////////////
void EntityManagementFeatures::RemoveCollisionFilterMask(
    const Identity &_shapeID)
{
/*
  const auto shapeNode = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  const std::size_t worldID = GetWorldOfShapeNode(this, shapeNode);
  const auto filterPtr = GetFilterPtr(this, worldID);
  filterPtr->RemoveIgnoredCollision(shapeNode);
*/
  auto collision =
      this->ReferenceInterface<CollisionInfo>(_shapeID)->collision;
  // remove = reset to default bitmask
  collision->SetCollideBitmask(0xFF);
}

