/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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


namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  auto newWorld = std::make_unique<World>(_name);

  return this->AddWorld(std::move(newWorld));
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  auto* modelToRemove = std::get<Model*>(this->entities.at(_modelID));
  auto& models = std::get<World*>(this->containers.at(_modelID))->models;
  auto sizeBefore =  models.size();
  models.erase(std::remove_if(models.begin(), models.end(), [&] (auto& model) -> bool
                { return model.get() == modelToRemove; }), models.end());
  return models.size() != sizeBefore;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::ModelRemoved(
    const Identity &_modelID) const
{
  auto* modelRemoved = std::get<Model*>(this->entities.at(_modelID));
  auto& models = std::get<World*>(this->containers.at(_modelID))->models;
  auto it = std::find_if(models.cbegin(), models.cend(), [&] (auto& model) -> bool
    { return model.get() == modelRemoved; });
  return it == models.cend();
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(
    const Identity & _worldID, std::size_t _modelIndex)
{
  auto& models = std::get<World*>(this->entities.at(_worldID))->models;

  if (_modelIndex >= models.size()) {
    return false;
  }

  models.erase(models.begin()+_modelIndex);

  return true;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(
    const Identity & _worldID, const std::string & _modelName )
{
  auto& models = std::get<World*>(this->entities.at(_worldID))->models;

  auto it = std::find_if(models.begin(), models.end(), [&] (auto& model) -> bool
    { return model->name == _modelName; });
  
  if (it == models.end()) {
    return false;
  }

  models.erase(it);

  return true;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveNestedModelByIndex(
    const Identity &_modelID, std::size_t _nestedModelIndex)
{
  auto model = std::get<Model*>(this->entities.at(_modelID));
  auto& nestedModels = model->models;

  if (_nestedModelIndex >= nestedModels.size()) {
    return false;
  }

  nestedModels.erase(nestedModels.begin()+_nestedModelIndex);

  return true;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveNestedModelByName(const Identity &_modelID,
                                                       const std::string &_modelName)
{
  auto model = std::get<Model*>(this->entities.at(_modelID));
  auto& nestedModels = model->models;

  auto it = std::find_if(nestedModels.begin(), nestedModels.end(), [&] (auto& nestedModel) -> bool
    { return nestedModel->name == _modelName; });
  
  if (it == nestedModels.end()) {
    // The model to remove as not found
    return false;
  }

  nestedModels.erase(it);

  return true;
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
