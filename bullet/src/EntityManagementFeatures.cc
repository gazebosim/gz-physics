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
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  // Create bullet empty multibody dynamics world
  const auto collisionConfiguration =
    std::make_shared<btDefaultCollisionConfiguration>();
  const auto dispatcher =
    std::make_shared<btCollisionDispatcher>(collisionConfiguration.get());
  const auto broadphase = std::make_shared<btDbvtBroadphase>();
  const auto solver =
    std::make_shared<btSequentialImpulseConstraintSolver>();
  const auto world = std::make_shared<btDiscreteDynamicsWorld>(
    dispatcher.get(), broadphase.get(), solver.get(),
    collisionConfiguration.get());

  /* TO-DO(Lobotuerk): figure out what this line does*/
  world->getSolverInfo().m_globalCfm = 0;

  btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher.get());

  return this->AddWorld(
    {_name, collisionConfiguration, dispatcher, broadphase, solver, world});
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  // Check if the model exists
  if (this->models.find(_modelID.id) == this->models.end())
  {
    return false;
  }

  auto worldID = this->models.at(_modelID)->world;
  auto modelIndex = idToIndexInContainer(_modelID);

  return this->RemoveModelByIndex(worldID, modelIndex);
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
  auto _modelEntity = indexInContainerToId(_worldID, _modelIndex);
  if (this->models.find(_modelEntity) == this->models.end())
  {
    return false;
  }

  auto model = this->models.at(_modelEntity);
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
      bulletWorld->removeConstraint(jointInfo->joint.get());
      this->childIdToParentId.erase(joint_it->first);
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
      this->childIdToParentId.erase(collision_it->first);
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
      bulletWorld->removeRigidBody(linkInfo->link.get());
      this->childIdToParentId.erase(it->first);
      it = this->links.erase(it);
      continue;
    }
    it++;
  }

  // Clean up model
  this->models.erase(_modelEntity);
  this->childIdToParentId.erase(_modelIndex);

  return true;
}

bool EntityManagementFeatures::RemoveModelByName(
    const Identity & _worldID, const std::string & _modelName )
{
  // Check if there is a model with the requested name
  bool found = false;
  size_t entity = 0;
  // We need a link to model relationship
  for (const auto &model : this->models)
  {
    const auto &modelInfo = model.second;
    if (modelInfo->name == _modelName)
    {
      found = true;
      entity = model.first;
      break;
    }
  }

  if (found)
  {
    auto modelIndex = idToIndexInContainer(entity);
    return this->RemoveModelByIndex(_worldID, modelIndex);
  }

  return false;
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
