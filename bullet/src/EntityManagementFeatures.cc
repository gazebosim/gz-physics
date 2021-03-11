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

#include <btBulletDynamicsCommon.h>

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
  // Create bullet empty multibody dynamics world
  btDefaultCollisionConfiguration* collisionConfiguration =
    new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher =
    new btCollisionDispatcher(collisionConfiguration);
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  auto solver = new btSequentialImpulseConstraintSolver;
  auto world = new btDiscreteDynamicsWorld(
    dispatcher, broadphase, solver, collisionConfiguration);

  world->getSolverInfo().m_globalCfm = 0;

  return this->AddWorld(
    {world, _name, collisionConfiguration, dispatcher, broadphase, solver});
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  // Check if the model exists
  if (this->models.find(_modelID.id) == this->models.end())
  {
    return false;
  }

  auto model = this->models.at(_modelID);
  auto bulletWorld = this->worlds.at(model->world)->world;

  // Clean up joints, this section considers both links in the joint
  // are part of the same world
  auto joint_it = this->joints.begin();
  while (joint_it != this->joints.end())
  {
    const auto &jointInfo = joint_it->second;
    const auto &childLinkInfo = this->links[jointInfo->childLinkId];
    if (childLinkInfo->model.id == _modelID.id)
    {
      btTypedConstraint* constraint = jointInfo->joint;
      bulletWorld->removeConstraint(constraint);
      delete constraint;
      joint_it = this->joints.erase(joint_it);
      continue;
    }
    joint_it++;
  }

  // Clean up collisions
  auto collision_it = this->collisions.begin();
  while (collision_it != this->collisions.end())
  {
    const auto &collisionInfo = collision_it->second;
    if (collisionInfo->model.id == _modelID.id)
    {
      delete collisionInfo->shape;
      collision_it = this->collisions.erase(collision_it);
      continue;
    }
    collision_it++;
  }

  // Clean up links
  auto it = this->links.begin();
  while (it != this->links.end())
  {
    const auto &linkInfo = it->second;
    if (linkInfo->model.id == _modelID.id)
    {
      bulletWorld->removeRigidBody(linkInfo->link);
      delete linkInfo->link;
      it = this->links.erase(it);
      continue;
    }
    it++;
  }

  // Clean up model, links are erased and the model is just a name to tie
  // them together
  this->models.erase(_modelID.id);

  return true;
}

bool EntityManagementFeatures::ModelRemoved(
    const Identity &_modelID) const
{
  return this->models.find(_modelID) == this->models.end();
}

bool EntityManagementFeatures::RemoveModelByIndex(
    const Identity & _worldID, std::size_t _modelIndex)
{
  // Check if the model exists
  if (this->models.find(_modelIndex) == this->models.end() ||
      this->models.at(_modelIndex)->world.id != _worldID.id)
  {
    return false;
  }

  auto model = this->models.at(_modelIndex);
  auto bulletWorld = this->worlds.at(model->world)->world;

  // Clean up joints, this section considers both links in the joint
  // are part of the same world
  auto joint_it = this->joints.begin();
  while (joint_it != this->joints.end())
  {
    const auto &jointInfo = joint_it->second;
    const auto &childLinkInfo = this->links[jointInfo->childLinkId];
    if (childLinkInfo->model.id == _modelIndex)
    {
      btTypedConstraint* constraint = jointInfo->joint;
      bulletWorld->removeConstraint(constraint);
      delete constraint;
      joint_it = this->joints.erase(joint_it);
      continue;
    }
    joint_it++;
  }

  // Clean up collisions
  auto collision_it = this->collisions.begin();
  while (collision_it != this->collisions.end())
  {
    const auto &collisionInfo = collision_it->second;
    if (collisionInfo->model.id == _modelIndex)
    {
      delete collisionInfo->shape;
      collision_it = this->collisions.erase(collision_it);
      continue;
    }
    collision_it++;
  }

  // Clean up links
  auto it = this->links.begin();
  while (it != this->links.end())
  {
    const auto &linkInfo = it->second;

    if (linkInfo->model.id == _modelIndex)
    {
      bulletWorld->removeRigidBody(linkInfo->link);
      delete linkInfo->link;
      it = this->links.erase(it);
      continue;
    }
    it++;
  }

  // Clean up model
  this->models.erase(_modelIndex);

  return true;
}

bool EntityManagementFeatures::RemoveModelByName(
    const Identity & _worldID, const std::string & _modelName )
{
  // Check if there is a model with the requested name
  bool found = false;
  size_t index_id = 0;
  // We need a link to model relationship
  for (const auto &model : this->models)
  {
    const auto &modelInfo = model.second;
    if (modelInfo->name == _modelName)
    {
      found = true;
      index_id = model.first;
      break;
    }
  }

  if (found)
  {
    return this->RemoveModelByIndex(_worldID, index_id);
  }

  return false;
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
