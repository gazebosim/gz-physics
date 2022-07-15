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

namespace gz {
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

Identity EntityManagementFeatures::ConstructEmptyModel(
  const Identity &_worldID, const std::string &_name)
{
  return this->AddModel(_worldID, {_name, _worldID, false, math::Pose3d(), {}});
}

Identity EntityManagementFeatures::ConstructEmptyLink(
  const Identity &_modelID, const std::string &_name)
{
  double mass = 1;
  btVector3 linkInertiaDiag(1, 1, 1);
  // Fixed links have 0 mass and inertia
  if (this->models.at(_modelID)->fixed)
  {
    mass = 0;
    linkInertiaDiag = btVector3(0, 0, 0);
  }

  auto myMotionState = std::make_shared<btDefaultMotionState>(btTransform());
  auto collisionShape = std::make_shared<btCompoundShape>();
  btRigidBody::btRigidBodyConstructionInfo
    rbInfo(mass, myMotionState.get(), collisionShape.get(), linkInertiaDiag);

  auto body = std::make_shared<btRigidBody>(rbInfo);
  body.get()->setActivationState(DISABLE_DEACTIVATION);

  return this->AddLink(
    _modelID,
    {_name, _modelID, math::Pose3d(), math::Pose3d(),
     mass, linkInertiaDiag,
     myMotionState, collisionShape, body});
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

const std::string &EntityManagementFeatures::GetEngineName(
  const Identity &) const
{
  static const std::string engineName = "bullet";
  return engineName;
}

std::size_t EntityManagementFeatures::GetEngineIndex(const Identity &) const
{
  return 0;
}

std::size_t EntityManagementFeatures::GetWorldCount(const Identity &) const
{
  return worlds.size();
}

Identity EntityManagementFeatures::GetWorld(
    const Identity &, std::size_t _worldIndex) const
{
  std::size_t index = 0;
  std::size_t worldEntity = 0;
  WorldInfoPtr worldPtr = nullptr;

  for (auto itWorld = this->worlds.begin();
       itWorld != this->worlds.end();
       ++itWorld)
  {
    if (index++ == _worldIndex)
    {
      worldPtr = itWorld->second;
      worldEntity = itWorld->first;
      break;
    }
  }

  return this->GenerateIdentity(worldEntity, worldPtr);
}

Identity EntityManagementFeatures::GetWorld(
    const Identity &, const std::string &_worldName) const
{
  std::size_t worldEntity = 0;
  WorldInfoPtr worldPtr = nullptr;
  for (auto itWorld = this->worlds.begin();
       itWorld != this->worlds.end();
       ++itWorld)
  {
    if (itWorld->second->name == _worldName)
    {
      worldPtr = itWorld->second;
      worldEntity = itWorld->first;
      break;
    }
  }

  return this->GenerateIdentity(worldEntity, worldPtr);
}

const std::string &EntityManagementFeatures::GetWorldName(
    const Identity &_worldID) const
{
  // Check if the world exists
  if (!this->worlds.count(_worldID.id))
  {
    static const std::string worldName = "";
    return worldName;
  }
  else
  {
    return this->worlds.at(_worldID)->name;
  }
}

std::size_t EntityManagementFeatures::GetWorldIndex(const Identity &) const
{
  return 0;
}

Identity EntityManagementFeatures::GetEngineOfWorld(const Identity &) const
{
  return this->GenerateIdentity(0);
}

std::size_t EntityManagementFeatures::GetModelCount(
    const Identity &) const
{
  return this->models.size();
}

Identity EntityManagementFeatures::GetModel(
    const Identity &, std::size_t _modelIndex) const
{
  std::size_t index = 0;
  std::size_t modelEntity = 0;
  ModelInfoPtr modelPtr = nullptr;

  for (auto itModel = this->models.begin();
       itModel != this->models.end();
       ++itModel)
  {
    if (index++ == _modelIndex)
    {
      modelPtr = itModel->second;
      modelEntity = itModel->first;
      break;
    }
  }

  return this->GenerateIdentity(modelEntity, modelPtr);
}

Identity EntityManagementFeatures::GetModel(
    const Identity &, const std::string &_modelName) const
{
  std::size_t modelEntity = 0;
  ModelInfoPtr modelPtr = nullptr;
  for (auto itModel = this->models.begin();
       itModel != this->models.end();
       ++itModel)
  {
    if (itModel->second->name == _modelName)
    {
      modelPtr = itModel->second;
      modelEntity = itModel->first;
      break;
    }
  }

  return this->GenerateIdentity(modelEntity, modelPtr);
}

const std::string &EntityManagementFeatures::GetModelName(
    const Identity &_modelID) const
{
  // Check if the model exists
  if (!this->models.count(_modelID.id))
  {
    static const std::string modelName = "";
    return modelName;
  }
  else
  {
    return this->models.at(_modelID)->name;
  }
}

std::size_t EntityManagementFeatures::GetModelIndex(
  const Identity &_modelId) const
{
  if (!this->models.count(_modelId.id))
  {
    return 0;
  }

  std::size_t index = 0;
  ModelInfoPtr modelPtr = nullptr;

  for (const auto& [key, value] : this->models)
  {
    if (key == _modelId.id)
    {
      return index;
    }
    index++;
  }

  return 0;
}

Identity EntityManagementFeatures::GetWorldOfModel(
  const Identity &_modelId) const
{
  if (!this->models.count(_modelId.id))
  {
    return this->GenerateInvalidId();
  }

  auto worldID = this->models.at(_modelId)->world;

  return this->GenerateIdentity(worldID, this->worlds.at(worldID));
}

std::size_t EntityManagementFeatures::GetNestedModelCount(
  const Identity &) const
{
  return 0;
}

Identity EntityManagementFeatures::GetNestedModel(
  const Identity &, std::size_t ) const
{
  return this->GenerateInvalidId();
}

Identity EntityManagementFeatures::GetNestedModel(
  const Identity &, const std::string &) const
{
  return this->GenerateInvalidId();
}

std::size_t EntityManagementFeatures::GetLinkCount(const Identity &) const
{
  return this->links.size();
}

Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, std::size_t _linkIndex) const
{
  if (!this->models.count(_modelID.id))
  {
    return this->GenerateInvalidId();
  }

  std::size_t index = 0;
  auto model = this->models.at(_modelID);

  for (const auto & link : model->links)
  {
    if (this->links.count(link))
    {
      if (index++ == _linkIndex)
      {
        auto linkFound = this->links.at(link);
        return this->GenerateIdentity(link, linkFound);
      }
    }
  }
  return this->GenerateInvalidId();
}

Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, const std::string &_linkName) const
{
  if (!this->models.count(_modelID.id))
  {
    return this->GenerateInvalidId();
  }

  auto model = this->models.at(_modelID);
  for (const auto & link : model->links)
  {
    if (this->links.count(link))
    {
        auto linkFound = this->links.at(link);
        if (linkFound->name == _linkName)
        {
          return this->GenerateIdentity(link, linkFound);
        }
    }
  }

  return this->GenerateInvalidId();
}

std::size_t EntityManagementFeatures::GetJointCount(const Identity &) const
{
  return 0;
}

Identity EntityManagementFeatures::GetJoint(
    const Identity &, std::size_t ) const
{
  return this->GenerateInvalidId();
}

Identity EntityManagementFeatures::GetJoint(
    const Identity &, const std::string &) const
{
  return this->GenerateInvalidId();
}

const std::string &EntityManagementFeatures::GetLinkName(
    const Identity &_linkId) const
{
  if (!this->links.count(_linkId.id))
  {
    static const std::string linkName = "";
    return linkName;
  }
  else
  {
    return this->links.at(_linkId)->name;
  }
}

std::size_t EntityManagementFeatures::GetLinkIndex(
  const Identity &_linkId) const
{
  if (!this->links.count(_linkId.id))
  {
    return 0;
  }

  std::size_t index = 0;
  for (const auto& [key, value] : this->links)
  {
    if (key == _linkId.id)
    {
      return index;
    }
    index++;
  }

  return 0;
}

Identity EntityManagementFeatures::GetModelOfLink(
  const Identity &_linkId) const
{
  if (!this->links.count(_linkId.id))
  {
    return this->GenerateInvalidId();
  }

  auto modelID = this->links.at(_linkId)->model;
  return this->GenerateIdentity(modelID, this->models.at(modelID));
}

std::size_t EntityManagementFeatures::GetShapeCount(const Identity &) const
{
  return 0;
}

Identity EntityManagementFeatures::GetShape(
    const Identity &, std::size_t) const
{
  return this->GenerateInvalidId();
}

Identity EntityManagementFeatures::GetShape(
    const Identity &, const std::string &) const
{
  return this->GenerateInvalidId();
}

const std::string &EntityManagementFeatures::GetJointName(
    const Identity &) const
{
  static const std::string jointName = "bulletJoint";
  return jointName;
}

std::size_t EntityManagementFeatures::GetJointIndex(const Identity &) const
{
  return 0;
}

Identity EntityManagementFeatures::GetModelOfJoint(const Identity &) const
{
  return this->GenerateInvalidId();
}

const std::string &EntityManagementFeatures::GetShapeName(
    const Identity &) const
{
  static const std::string shapeName = "bulletShape";
  return shapeName;
}

std::size_t EntityManagementFeatures::GetShapeIndex(const Identity &) const
{
  return 0;
}

Identity EntityManagementFeatures::GetLinkOfShape(const Identity &) const
{
  return this->GenerateInvalidId();
}
}  // namespace bullet
}  // namespace physics
}  // namespace gz
