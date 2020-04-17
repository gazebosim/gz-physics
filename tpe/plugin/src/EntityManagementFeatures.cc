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
  const Identity &/*_engineID*/) const
{
  static const std::string engineName = "tpe";
  return engineName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetEngineIndex(
  const Identity &/*_engineID*/) const
{
  return 0;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldCount(
  const Identity &/*_engineID*/) const
{
  return worlds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
  const Identity &, std::size_t _worldIndex) const
{
  auto it = worlds.begin();
  std::advance(it, _worldIndex);
  if (it != worlds.end())
    return this->GenerateIdentity(it->first, it->second);
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
  const Identity &, const std::string &_worldName) const
{
  for (auto it = worlds.begin(); it != worlds.end(); ++it)
  {
    if (it->second->world->GetName() == _worldName)
      return this->GenerateIdentity(it->first, it->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetWorldName(
  const Identity &_worldID) const
{
  static std::string name{""};
  auto it = worlds.find(_worldID);
  if (it != worlds.end())
    name = it->second->world->GetName();
  return name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldIndex(
  const Identity &_worldID) const
{
  auto it = worlds.find(_worldID);
  if (it != worlds.end())
    return std::distance(it, worlds.begin());
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetEngineOfWorld(
  const Identity &/*_worldID*/) const
{
  return this->GenerateIdentity(0);
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelCount(
  const Identity &_worldID) const
{
  auto it = this->worlds.find(_worldID);
  if (it != this->worlds.end())
    return it->second->world->GetChildCount();
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
  const Identity &_worldID, const std::size_t _modelIndex) const
{
  // Get the selected world
  auto worldIt = worlds.find(_worldID);

  // Get the selected model
  auto modelIt = models.begin();
  std::advance(modelIt, _modelIndex);

  if (worldIt != worlds.end() && modelIt != models.end())
  {
    // Get the model
    std::size_t modelId = modelIt->first;
    auto model = worldIt->second->world->GetChildById(modelId);
    auto modelPtr = std::make_shared<ModelInfo>();
    modelPtr->model = static_cast<tpelib::Model *>(&model);
    return this->GenerateIdentity(modelId, modelPtr);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
  const Identity &_worldID, const std::string &_modelName) const
{
  // Get the selected world
  auto worldIt = worlds.find(_worldID);
  if (worldIt == worlds.end())
    return this->GenerateInvalidId();

  // Get the selected model and check name and id
  auto model = worldIt->second->world->GetChildByName(_modelName);
  for (auto it = models.begin(); it != models.end(); ++it)
  {
    if (it->second->model->GetName() == _modelName &&
      it->second->model->GetId() == model.GetId())
    {
      return this->GenerateIdentity(model.GetId(), it->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetModelName(
  const Identity &_modelID) const
{
  static std::string name{""};
  auto it = models.find(_modelID);
  if (it != models.end())
    name =  it->second->model->GetName();
  return name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelIndex(
  const Identity &_modelID) const
{
  auto it = models.find(_modelID);
  if (it != models.end())
    return std::distance(it, models.begin());
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorldOfModel(
  const Identity &_modelID) const
{
  auto it = childIdToParentId.find(_modelID);
  if (it != childIdToParentId.end())
  {
    auto worldIt = worlds.find(it->second);
    if (worldIt != worlds.end())
      return this->GenerateIdentity(worldIt->first, worldIt->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkCount(
  const Identity &_modelID) const
{
  auto it = models.find(_modelID);
  if (it != models.end())
    return it->second->model->GetChildCount();
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
  const Identity &_modelID, const std::size_t _linkIndex) const
{
  auto linkIt = links.begin();
  std::advance(linkIt, _linkIndex);
  auto modelIt = models.find(_modelID);
  if (linkIt != links.end() && modelIt != models.end())
  {
    std::size_t linkId = linkIt->first;
    auto link = modelIt->second->model->GetChildById(linkId);
    auto linkPtr = std::make_shared<tpelib::Entity>(link);
    return this->GenerateIdentity(linkId, linkPtr);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
  const Identity &_modelID, const std::string &_linkName) const
{
  auto modelIt = models.find(_modelID);
  if (modelIt != models.end())
  {
    auto link = modelIt->second->model->GetChildByName(_linkName);
    for (auto it = links.begin(); it != links.end(); ++it)
    {
      if (it->second->link->GetName() == _linkName &&
          it->first == link.GetId())
      {
        return this->GenerateIdentity(link.GetId(), it->second);
      }
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetLinkName(
  const Identity &_linkID) const
{
  static std::string name{""};
  auto it = links.find(_linkID);
  if (it != links.end())
    name = it->second->link->GetName();
  return name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkIndex(
  const Identity &_linkID) const
{
  auto it = links.find(_linkID);
  if (it != links.end())
    return std::distance(it, links.begin());
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfLink(
  const Identity &_linkID) const
{
  auto it = childIdToParentId.find(_linkID);
  auto modelIt = models.find(it->second);
  if (it != childIdToParentId.end() && modelIt != models.end())
    return this->GenerateIdentity(modelIt->first, modelIt->second);
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeCount(
  const Identity &_linkID) const
{
  auto it = links.find(_linkID);
  if (it != links.end())
    return it->second->link->GetChildCount();
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::size_t _shapeIndex) const
{
  // assume _shapeIndex ~= collisionIndex
  auto shapeIt = collisions.begin();
  std::advance(shapeIt, _shapeIndex);
  auto linkIt = links.find(_linkID);
  if (shapeIt != collisions.end() && linkIt != links.end())
  {
    std::size_t shapeID = shapeIt->first;
    auto shape = linkIt->second->link->GetChildById(shapeID);
    auto shapePtr = std::make_shared<tpelib::Entity>(shape);
    return this->GenerateIdentity(shapeID, shapePtr);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::string &_shapeName) const
{
  // assume shapeName ~= collisionName
  auto linkIt = links.find(_linkID);
  if (linkIt != links.end())
  {
    auto shape = linkIt->second->link->GetChildByName(_shapeName);
    for (auto it = collisions.begin(); it != collisions.end(); ++it)
    {
      if (it->second->collision->GetName() == _shapeName &&
        it->first == shape.GetId())
      {
        return this->GenerateIdentity(shape.GetId(), it->second);
      }
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetShapeName(
  const Identity &_shapeID) const
{
  // assume _shapeID ~= collisionID
  static std::string name{""};
  auto it = collisions.find(_shapeID);
  if (it != collisions.end())
    name = it->second->collision->GetName();
  return name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeIndex(
  const Identity &_shapeID) const
{
  // assume _shapeID ~= collisionID
  auto it = collisions.find(_shapeID);
  if (it != collisions.end())
    return std::distance(it, collisions.begin());
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLinkOfShape(
  const Identity &_shapeID) const
{
  // assume _shapeID ~= collisionID
  auto it = childIdToParentId.find(_shapeID);
  auto linkIt = links.find(it->second);
  if (it != childIdToParentId.end() && linkIt != links.end())
    return this->GenerateIdentity(linkIt->first, linkIt->second);
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(
  const Identity &_worldID, std::size_t _modelIndex)
{
  auto modelIt = this->models.begin();
  std::advance(modelIt, _modelIndex);
  auto worldIt = this->worlds.find(_worldID);

  if (modelIt != models.end() && worldIt != worlds.end())
  {
    std::size_t modelId = modelIt->first;
    models.erase(modelId);
    this->childIdToParentId.erase(modelId);
    return worldIt->second->world->RemoveChildById(modelId);
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(
  const Identity &_worldID, const std::string &_modelName)
{
  auto worldIt = worlds.find(_worldID);
  if (worldIt != worlds.end())
  {
    auto modelId = worldIt->second->world->GetChildByName(_modelName).GetId();
    auto modelIt = this->models.find(modelId);
    if (modelIt != this->models.end())
    {
      this->models.erase(modelId);
      this->childIdToParentId.erase(modelId);
      return worldIt->second->world->RemoveChildByName(_modelName);
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  auto it = childIdToParentId.find(_modelID);
  if (it != childIdToParentId.end())
  {
    auto worldIt = worlds.find(it->second);
    if (worldIt != worlds.end())
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
  auto it = models.find(_modelID);
  return it == models.end();
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
  if (it != this->worlds.end())
  {
    // add model to the corresponding world
    std::shared_ptr<WorldInfo> worldPtr = it->second;
    auto &modelEnt = worldPtr->world->AddModel();
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
  if (it != this->models.end())
  {
    // add link to the corresponding model
    std::shared_ptr<ModelInfo> modelPtr = it->second;
    auto &linkEnt = modelPtr->model->AddLink();
    linkEnt.SetName(_name);
    tpelib::Link *link = static_cast<tpelib::Link *>(&linkEnt);
    return this->AddLink(_modelID, *link);
  }
  return this->GenerateInvalidId();
}
