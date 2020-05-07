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
  static std::string worldName {};
  auto it = this->worlds.find(_worldID);
  if (it != this->worlds.end() && it->second != nullptr)
  {
    worldName = it->second->world->GetName();
  }
  return worldName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldIndex(
  const Identity &_worldID) const
{
  // index should be 0 assuming there's only one world
  auto it = this->worlds.find(_worldID);
  if (it != this->worlds.end() && it->second != nullptr)
    return std::distance(it, this->worlds.begin());
  return -1;
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
  auto it = this->worlds.find(_worldID);
  if (it != this->worlds.end() && it->second != nullptr)
  {
    return this->worlds.at(_worldID)->world->GetChildCount();
  }
  // return invalid index if world not found
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
  const Identity &_worldID, const std::size_t _modelIndex) const
{
  if (this->worlds.find(_worldID) != this->worlds.end())
  {
    std::size_t modelId = this->indexInContainerToId(_worldID, _modelIndex);
    auto it = this->models.find(modelId);
    if (it != this->models.end() && it->second != nullptr)
    {
      return this->GenerateIdentity(it->first, it->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
  const Identity &_worldID, const std::string &_modelName) const
{
  auto worldIt = this->worlds.find(_worldID);
  if (worldIt != this->worlds.end() && worldIt->second != nullptr)
  {
    tpelib::Entity &modelEnt =
      worldIt->second->world->GetChildByName(_modelName);
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
  static std::string modelName {};
  auto it = this->models.find(_modelID);
  if (it != this->models.end() && it->second != nullptr)
  {
    modelName = it->second->model->GetName();
  }
  return modelName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelIndex(
  const Identity &_modelID) const
{
  auto it = this->models.find(_modelID);
  if (it != this->models.end() && it->second != nullptr)
  {
    return this->idToIndexInContainer(_modelID.id);
  }
  // return invalid index if model not found
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorldOfModel(
  const Identity &_modelID) const
{
  auto it = this->childIdToParentId.find(_modelID);
  if (it != this->childIdToParentId.end())
  {
    std::size_t worldId = this->childIdToParentId.at(_modelID);
    auto worldIt = this->worlds.find(worldId);
    if (worldIt != this->worlds.end() && worldIt->second != nullptr)
    {
      return this->GenerateIdentity(worldId, worldIt->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkCount(
  const Identity &_modelID) const
{
  auto it = this->models.find(_modelID);
  if (it != this->models.end() && it->second != nullptr)
  {
    return it->second->model->GetChildCount();
  }
  // return invalid count if model not found
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
  const Identity &_modelID, const std::size_t _linkIndex) const
{
  if (this->models.find(_modelID) != this->models.end())
  {
    std::size_t linkId = this->indexInContainerToId(_modelID, _linkIndex);
    auto it = this->links.find(linkId);
    if (it != this->links.end() && it->second != nullptr)
    {
      return this->GenerateIdentity(it->first, it->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
  const Identity &_modelID, const std::string &_linkName) const
{
  auto modelIt = this->models.find(_modelID);
  if (modelIt != this->models.end() && modelIt->second != nullptr)
  {
    tpelib::Entity &linkEnt =
      modelIt->second->model->GetChildByName(_linkName);
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
  static std::string linkName {};
  auto it = this->links.find(_linkID);
  if (it != this->links.end() && it->second != nullptr)
  {
    linkName = it->second->link->GetName();
  }
  return linkName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkIndex(
  const Identity &_linkID) const
{
  auto it = this->links.find(_linkID);
  if (it != this->links.end() && it->second != nullptr)
  {
    return this->idToIndexInContainer(_linkID.id);
  }
  // return invalid index if model not found
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfLink(
  const Identity &_linkID) const
{
  auto it = this->childIdToParentId.find(_linkID);
  if (it != this->childIdToParentId.end())
  {
    std::size_t modelId = this->childIdToParentId.at(_linkID);
    auto modelIt = this->models.find(modelId);
    if (modelIt != this->models.end() && modelIt->second != nullptr)
    {
      return this->GenerateIdentity(modelId, modelIt->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeCount(
  const Identity &_linkID) const
{
  auto it = this->links.find(_linkID);
  if (it != this->links.end() && it->second != nullptr)
  {
    return it->second->link->GetChildCount();
  }
  // return invalid count if link not found
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::size_t _shapeIndex) const
{
  if (this->links.find(_linkID) != this->links.end())
  {
    std::size_t shapeId = this->indexInContainerToId(_linkID, _shapeIndex);
    auto it = this->collisions.find(shapeId);
    if (it != this->collisions.end() && it->second != nullptr)
    {
      return this->GenerateIdentity(it->first, it->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::string &_shapeName) const
{
  auto linkIt = this->links.find(_linkID);
  if (linkIt != this->links.end() && linkIt->second != nullptr)
  {
    tpelib::Entity &shapeEnt = 
      linkIt->second->link->GetChildByName(_shapeName);
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
  static std::string shapeName {};
  auto it = this->collisions.find(_shapeID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    shapeName = it->second->collision->GetName();
  }
  return shapeName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeIndex(
  const Identity &_shapeID) const
{
  auto it = this->collisions.find(_shapeID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    return this->idToIndexInContainer(_shapeID.id);
  }
  // return invalid index if model not found
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLinkOfShape(
  const Identity &_shapeID) const
{
  auto it = this->childIdToParentId.find(_shapeID);
  if (it != this->childIdToParentId.end())
  {
    std::size_t linkId = this->childIdToParentId.at(_shapeID);
    auto linkIt = this->links.find(linkId);
    if (linkIt != this->links.end() && linkIt->second != nullptr)
    {
      return this->GenerateIdentity(linkId, linkIt->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(
  const Identity &_worldID, std::size_t _modelIndex)
{
  auto it = this->worlds.find(_worldID);
  if (it != this->worlds.end() && it->second != nullptr)
  {
    auto modelId = this->indexInContainerToId(_worldID, _modelIndex);
    if (this->models.find(modelId) != this->models.end())
    {
      this->models.erase(modelId);
      this->childIdToParentId.erase(modelId);
      return it->second->world->RemoveChildById(modelId);
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(
  const Identity &_worldID, const std::string &_modelName)
{
  auto it = this->worlds.find(_worldID);

  if (it != this->worlds.end() && it->second != nullptr)
  {
    std::size_t modelId =
      it->second->world->GetChildByName(_modelName).GetId();
    this->models.erase(modelId);
    this->childIdToParentId.erase(modelId);
    return it->second->world->RemoveChildById(modelId);
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  auto it = this->childIdToParentId.find(_modelID);

  if (it != this->childIdToParentId.end())
  {
    auto worldIt = this->worlds.find(it->second);
    if (worldIt != this->worlds.end() && worldIt->second != nullptr)
    {
      this->models.erase(_modelID);
      this->childIdToParentId.erase(_modelID);
      return worldIt->second->world->RemoveChildById(_modelID);
    }
  }
  return false;  
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::ModelRemoved(const Identity &_modelID) const
{
  return this->models.find(_modelID) == this->models.end();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
  const Identity &/*_engineID*/, const std::string &_name)
{
  auto world = std::make_shared<tpelib::World>();
  world->SetName(_name);
  return this->AddWorld(world);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyModel(
  const Identity &_worldID, const std::string &_name)
{
  auto it = this->worlds.find(_worldID);
  if (it != this->worlds.end() && it->second != nullptr)
  {
    auto &modelEnt = it->second->world->AddModel();
    modelEnt.SetName(_name);
    tpelib::Model *model = static_cast<tpelib::Model *>(&modelEnt);
    return this->AddModel(_worldID, *model);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyLink(
  const Identity &_modelID, const std::string &_name)
{
  auto it = this->models.find(_modelID);
  if (it != this->models.end() && it->second != nullptr)
  {
    auto &linkEnt = it->second->model->AddLink();
    linkEnt.SetName(_name);
    tpelib::Link *link = static_cast<tpelib::Link *>(&linkEnt);
    return this->AddLink(_modelID, *link);
  }
  return this->GenerateInvalidId();
}
