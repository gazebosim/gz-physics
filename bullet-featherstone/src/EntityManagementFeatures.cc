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
    assert(joint->constraint);
    world->world->removeMultiBodyConstraint(joint->constraint.get());
    this->joints.erase(constraint_index);
  }

  world->world->removeMultiBody(model->body.get());
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

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
