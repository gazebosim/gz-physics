/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <btBulletDynamicsCommon.h>

#include <string>

#include "EntityManagementFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetEngineName(
    const Identity &/*_engineID*/) const
{
  static const std::string engineName = "bullet";
  return engineName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetEngineIndex(
    const Identity &/*_engineID*/) const
{
  return 0;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetEngineOfWorld(
    const Identity &/*_worldID*/) const
{
  return this->GenerateIdentity(0);
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetWorldName(
    const Identity &_worldID) const
{
  return this->worlds.at(_worldID)->name;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  // Create bullet empty multibody dynamics world
  auto collisionConfiguration = new btDefaultCollisionConfiguration();
  auto dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  auto solver = new btMultiBodyConstraintSolver;
  auto world = new btMultiBodyDynamicsWorld(
      dispatcher, broadphase, solver, collisionConfiguration);

  world->getSolverInfo().m_globalCfm = 0;

  return this->AddWorld(
      {world, _name, collisionConfiguration, dispatcher, broadphase, solver});
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, const std::string &_linkName) const
{
  for (const auto &linkEntry : this->links)
  {
    const auto &linkInfo = linkEntry.second;
    if (linkInfo->model.id == _modelID.id && linkInfo->name == _linkName)
      return this->GenerateIdentity(linkEntry.first, linkInfo);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetJoint(
    const Identity &_modelID, const std::string &_jointName) const
{
  for (const auto &jointEntry : this->joints)
  {
    const auto &jointInfo = jointEntry.second;
    if (jointInfo->model.id == _modelID.id && jointInfo->name == _jointName)
      return this->GenerateIdentity(jointEntry.first, jointInfo);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
    const Identity &_linkID, const std::string &_shapeName) const
{
  for (const auto &collisionEntry : this->collisions)
  {
    const auto &collisionInfo = collisionEntry.second;
    if (collisionInfo->link.id == _linkID.id &&
        collisionInfo->name == _shapeName)
      return this->GenerateIdentity(collisionEntry.first, collisionInfo);
  }
  return this->GenerateInvalidId();
}

///////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  const auto &modelInfo = this->models.at(_modelID);

  // Clean up collisions
  for (const auto &collisionEntry : this->collisions)
  {
    const auto &collisionInfo = collisionEntry.second;
    if (collisionInfo->model.id == _modelID.id)
    {
      delete collisionInfo->shape;
      delete collisionInfo->collider;
      this->collisions.erase(collisionEntry.first);
    }
  }

  // Clean up joints
  for (const auto &jointEntry : this->joints)
  {
    const auto &jointInfo = jointEntry.second;
    if (jointInfo->model.id == _modelID.id)
    {
      this->joints.erase(jointEntry.first);
    }
  }

  // Clean up links
  for (const auto &linkEntry : this->links)
  {
    const auto &linkInfo = linkEntry.second;
    if (linkInfo->model.id == _modelID.id)
    {
      this->links.erase(linkEntry.first);
    }
  }

  // Clean up model
  delete modelInfo->model;
  this->models.erase(_modelID);

  return true;
}

bool EntityManagementFeatures::ModelRemoved(
  const Identity &_modelID) const
{
  return this->models.find(_modelID) == this->models.end();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetModelName(
      const Identity &_modelID) const
{
  return this->models.at(_modelID)->name;
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetLinkName(
      const Identity &_linkID) const
{
  return this->links.at(_linkID)->name;
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetJointName(
    const Identity &_jointID) const
{
  return this->joints.at(_jointID)->name;
}

}
}
}
