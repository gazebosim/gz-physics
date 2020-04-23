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
  // engine name should not change
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
  // should always be 1
  return this->worlds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
  const Identity &, std::size_t _worldIndex) const
{
  auto it = this->worlds.begin();
  std::advance(it, _worldIndex);
  if (it != this->worlds.end())
    return this->GenerateIdentity(it->first, it->second);
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
  const Identity &, const std::string &_worldName) const
{
  for (auto it = this->worlds.begin(); it != this->worlds.end(); ++it)
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
  static std::string worldName =
    this->worlds.at(_worldID)->world->GetName();
  return worldName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldIndex(
  const Identity &_worldID) const
{
  auto it = this->worlds.find(_worldID);
  if (it != this->worlds.end())
    return std::distance(it, this->worlds.begin());
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
  return this->worlds.at(_worldID)->world->GetChildCount();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
  const Identity &_worldID, const std::size_t _modelIndex) const
{
  std::shared_ptr<tpelib::World> world = this->worlds.at(_worldID)->world;

  auto modelIt = this->models.begin();
  std::advance(modelIt, _modelIndex);

  if (modelIt != this->models.end())
  {
    std::size_t modelId = modelIt->first;
    tpelib::Entity &modelEnt = world->GetChildById(modelId);
    auto modelPtr = std::make_shared<ModelInfo>();
    modelPtr->model = static_cast<tpelib::Model *>(&modelEnt);
    return this->GenerateIdentity(modelId, modelPtr);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
  const Identity &_worldID, const std::string &_modelName) const
{
  tpelib::Entity &modelEnt =
    this->worlds.at(_worldID)->world->GetChildByName(_modelName);

  for (auto it = this->models.begin(); it != this->models.end(); ++it)
  {
    if (it->first == modelEnt.GetId() &&
      it->second->model->GetName() == modelEnt.GetName())
    {
      return this->GenerateIdentity(it->first, it->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetModelName(
  const Identity &_modelID) const
{
  static std::string modelName =
    this->models.at(_modelID)->model->GetName();
  return modelName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelIndex(
  const Identity &_modelID) const
{
  auto it = this->models.find(_modelID);
  if (it != this->models.end())
    return std::distance(it, this->models.begin());
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorldOfModel(
  const Identity &_modelID) const
{
  std::size_t worldId = this->childIdToParentId.at(_modelID);
  return this->GenerateIdentity(worldId, this->worlds.at(worldId));
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkCount(
  const Identity &_modelID) const
{
  return this->models.at(_modelID)->model->GetChildCount();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
  const Identity &_modelID, const std::size_t _linkIndex) const
{
  tpelib::Model *model = this->models.at(_modelID)->model;
  
  auto linkIt = this->links.begin();
  std::advance(linkIt, _linkIndex);

  if (linkIt != this->links.end())
  {
    std::size_t linkId = linkIt->first;
    tpelib::Entity &linkEnt = model->GetChildById(linkId);
    auto linkPtr = std::make_shared<LinkInfo>();
    linkPtr->link = static_cast<tpelib::Link *>(&linkEnt);
    return this->GenerateIdentity(linkId, linkPtr);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
  const Identity &_modelID, const std::string &_linkName) const
{
  tpelib::Entity &linkEnt =
    this->models.at(_modelID)->model->GetChildByName(_linkName);
  
  for (auto it = this->links.begin(); it != this->links.end(); ++it)
  {
    if (it->first == linkEnt.GetId() &&
      it->second->link->GetName() == linkEnt.GetName())
    {
      return this->GenerateIdentity(it->first, it->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetLinkName(
  const Identity &_linkID) const
{
  static std::string linkName = this->links.at(_linkID)->link->GetName();
  return linkName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkIndex(
  const Identity &_linkID) const
{
  auto it = this->links.find(_linkID);
  if (it != this->links.end())
    return std::distance(it, this->links.begin());
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfLink(
  const Identity &_linkID) const
{
  std::size_t modelId = this->childIdToParentId.at(_linkID);
  return this->GenerateIdentity(modelId, this->models.at(modelId));
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeCount(
  const Identity &_linkID) const
{
  return this->links.at(_linkID)->link->GetChildCount();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::size_t _shapeIndex) const
{
  // assume _shapeIndex ~= collisionIndex
  tpelib::Link *link = this->links.at(_linkID)->link;
  
  auto collisionIt = this->collisions.begin();
  std::advance(collisionIt, _shapeIndex);

  if (collisionIt != this->collisions.end())
  {
    std::size_t collisionId = collisionIt->first;
    tpelib::Entity &collisionEnt = link->GetChildById(collisionId);
    auto collisionPtr = std::make_shared<CollisionInfo>();
    collisionPtr->collision = static_cast<tpelib::Collision *>(&collisionEnt);
    return this->GenerateIdentity(collisionId, collisionPtr);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
  const Identity &_linkID, const std::string &_shapeName) const
{
  // assume shapeName ~= collisionName
  tpelib::Entity &shapeEnt = 
    this->links.at(_linkID)->link->GetChildByName(_shapeName);
  for (auto it = this->collisions.begin(); it != this->collisions.end(); ++it)
  {
    if (it->first == shapeEnt.GetId() &&
      it->second->collision->GetName() == shapeEnt.GetName())
    {
      return this->GenerateIdentity(it->first, it->second);
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetShapeName(
  const Identity &_shapeID) const
{
  // assume _shapeID ~= collisionID
  static std::string shapeName =
    this->collisions.at(_shapeID)->collision->GetName();
  return shapeName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeIndex(
  const Identity &_shapeID) const
{
  // assume _shapeID ~= collisionID
  auto it = this->collisions.find(_shapeID);
  if (it != this->collisions.end())
    return std::distance(it, this->collisions.begin());
  return -1;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLinkOfShape(
  const Identity &_shapeID) const
{
  // assume _shapeID ~= collisionID
  std::size_t linkId = this->childIdToParentId.at(_shapeID);
  return this->GenerateIdentity(linkId, this->links.at(linkId));
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(
  const Identity &_worldID, std::size_t _modelIndex)
{
  std::shared_ptr<tpelib::World> world = this->worlds.at(_worldID)->world;

  auto it = this->models.begin();
  std::advance(it, _modelIndex);

  if (it != this->models.end())
  {
    std::size_t modelId = it->first;
    this->models.erase(modelId);
    this->childIdToParentId.erase(modelId);
    return world->RemoveChildById(modelId);
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(
  const Identity &_worldID, const std::string &_modelName)
{
  std::shared_ptr<tpelib::World> world = this->worlds.at(_worldID)->world;
  std::size_t modelId = world->GetChildByName(_modelName).GetId();
  this->models.erase(modelId);
  this->childIdToParentId.erase(modelId);
  return world->RemoveChildById(modelId);
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  // find world
  std::size_t worldId = this->childIdToParentId.at(_modelID);
  std::shared_ptr<tpelib::World> world = this->worlds.at(worldId)->world;
  this->models.erase(_modelID);
  this->childIdToParentId.erase(_modelID);
  return world->RemoveChildById(_modelID);
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
