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

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  const auto model = this->ReferenceInterface<ModelInfo>(_modelID);
  auto worldID = model->world;
  auto bulletWorld = this->worlds.at(model->world)->world;

  // Clean up joints, this section considers both links in the joint
  // are part of the same world
  for (const auto jointID : model->joints)
  {
    const auto joint = this->joints.at(jointID);
    bulletWorld->removeConstraint(joint->joint.get());
    this->joints.erase(jointID);
  }

  for (const auto linkID : model->links)
  {
    const auto link = this->links.at(linkID);
    for (const auto shapeID : link->shapes)
      this->collisions.erase(shapeID);

    bulletWorld->removeRigidBody(link->link.get());
    this->links.erase(linkID);
  }

  // Clean up model
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
  const auto modelID = this->GetModel(_worldID, _modelIndex);
  // Check if the model exists
  if (!modelID)
    return false;

  return this->RemoveModel(modelID);
}

bool EntityManagementFeatures::RemoveModelByName(
    const Identity & _worldID, const std::string & _modelName )
{
  const auto modelID = this->GetModel(_worldID, _modelName);
  if (!modelID)
    return false;

  return this->RemoveModel(modelID);
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
    const Identity &, const std::size_t _worldIndex) const
{
  if (_worldIndex >= this->worldsByIndex.size())
    return this->GenerateInvalidId();

  const auto worldID = this->worldsByIndex[_worldIndex];
  return this->GenerateIdentity(worldID, this->worlds.at(worldID));
}

Identity EntityManagementFeatures::GetWorld(
    const Identity &, const std::string &_name) const
{
  const auto it = this->worldsByName.find(_name);
  if (it == this->worldsByName.end())
    return this->GenerateInvalidId();

  const auto id = it->second;
  return this->GenerateIdentity(id, this->worlds.at(id));
}

const std::string &EntityManagementFeatures::GetWorldName(
    const Identity &_worldID) const
{
  return this->ReferenceInterface<WorldInfo>(_worldID)->name;
}

std::size_t EntityManagementFeatures::GetWorldIndex(
    const Identity &_worldID) const
{
  const auto it = std::find(
        this->worldsByIndex.begin(),
        this->worldsByIndex.end(),
        _worldID.id);

  if (it == this->worldsByIndex.end())
  {
    throw std::runtime_error(
      "World [" + std::to_string(_worldID.id) + "] cannot be found in engine "
      "with " + std::to_string(this->worlds.size()) + " worlds");
  }

  return *it;
}

Identity EntityManagementFeatures::GetEngineOfWorld(const Identity &) const
{
  return this->GenerateIdentity(0);
}

std::size_t EntityManagementFeatures::GetModelCount(
    const Identity &_worldId) const
{
  return this->ReferenceInterface<WorldInfo>(_worldId)->models.size();
}

Identity EntityManagementFeatures::GetModel(
    const Identity &_worldID, const std::size_t index) const
{
  const auto modelID =
      this->ReferenceInterface<WorldInfo>(_worldID)->models.at(index);
  return this->GenerateIdentity(modelID, this->models.at(modelID));
}

Identity EntityManagementFeatures::GetModel(
    const Identity &_worldID, const std::string &_name) const
{
  const auto world = this->ReferenceInterface<WorldInfo>(_worldID);
  const auto it = world->modelsByName.find(_name);
  if (it == world->modelsByName.end())
    return this->GenerateInvalidId();

  const auto id = it->second;
  return this->GenerateIdentity(id, this->models.at(id));
}

const std::string &EntityManagementFeatures::GetModelName(
    const Identity &) const
{
  static const std::string modelName = "bulletModel";
  return modelName;
}

std::size_t EntityManagementFeatures::GetModelIndex(
    const Identity &_modelID) const
{
  const auto model = this->ReferenceInterface<ModelInfo>(_modelID);
  const auto world = this->ReferenceInterface<WorldInfo>(model->world);
  const auto it =
      std::find(world->models.begin(), world->models.end(), _modelID.id);

  if (it == world->models.end())
  {
    throw std::runtime_error(
          "Model [" + std::to_string(_modelID.id) + "] cannot be found in "
          "world [" + std::to_string(model->world.id) + "]");
  }

  return *it;
}

Identity EntityManagementFeatures::GetWorldOfModel(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->world;
}

std::size_t EntityManagementFeatures::GetLinkCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->links.size();
}

Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, std::size_t _linkIndex) const
{
  const auto model = this->ReferenceInterface<ModelInfo>(_modelID);
  if (_linkIndex < model->links.size())
  {
    const auto id = model->links[_linkIndex];
    const auto link = this->links.at(id);
    return this->GenerateIdentity(id, link);
  }

  return this->GenerateInvalidId();
}

Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, const std::string &_name) const

{
  const auto model = this->ReferenceInterface<ModelInfo>(_modelID);
  const auto it = model->linksByName.find(_name);
  if (it == model->linksByName.end())
    return this->GenerateInvalidId();

  const auto linkID = it->second;
  return this->GenerateIdentity(linkID, this->links.at(linkID));
}

std::size_t EntityManagementFeatures::GetJointCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->joints.size();
}

Identity EntityManagementFeatures::GetJoint(
    const Identity &_modelID, std::size_t _jointIndex) const
{
  const auto model = this->ReferenceInterface<ModelInfo>(_modelID);
  if (_jointIndex < model->joints.size())
  {
    const auto id = model->joints[_jointIndex];
    const auto joint = this->joints.at(id);
    return this->GenerateIdentity(id, joint);
  }

  return this->GenerateInvalidId();
}

Identity EntityManagementFeatures::GetJoint(
    const Identity &_modelID, const std::string &_name) const
{
  const auto model = this->ReferenceInterface<ModelInfo>(_modelID);
  const auto it = model->jointsByName.find(_name);
  if (it == model->jointsByName.end())
    return this->GenerateInvalidId();

  const auto jointID = it->second;
  return this->GenerateIdentity(jointID, this->joints.at(jointID));
}

const std::string &EntityManagementFeatures::GetLinkName(
    const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->name;
}

std::size_t EntityManagementFeatures::GetLinkIndex(
    const Identity &_linkID) const
{
  const auto link = this->ReferenceInterface<LinkInfo>(_linkID);
  const auto model = this->ReferenceInterface<ModelInfo>(link->model);
  const auto it =
      std::find(model->links.begin(), model->links.end(), _linkID.id);

  if (it == model->links.end())
  {
    throw std::runtime_error(
          "Link [" + std::to_string(_linkID.id) + "] cannot be found in "
          "model [" + std::to_string(link->model.id) + "]");
  }

  return *it;
}

Identity EntityManagementFeatures::GetModelOfLink(
    const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->model;
}

std::size_t EntityManagementFeatures::GetShapeCount(
    const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->shapes.size();
}

Identity EntityManagementFeatures::GetShape(
    const Identity &_linkID, std::size_t shapeIndex) const
{
  const auto link = this->ReferenceInterface<LinkInfo>(_linkID);
  if (link->shapes.size() >= shapeIndex)
  {
    throw std::runtime_error(
          "Link [" + std::to_string(_linkID.id) + "] cannot be found in "
          "model [" + std::to_string(link->model.id) + "]");
  }

  const auto shapeID = link->shapes[shapeIndex];
  return this->GenerateIdentity(shapeID, this->collisions.at(shapeID));
}

Identity EntityManagementFeatures::GetShape(
    const Identity &_linkID, const std::string &_name) const
{
  const auto link = this->ReferenceInterface<LinkInfo>(_linkID);
  const auto it = std::find_if(
        link->shapes.begin(),
        link->shapes.end(),
        [this, &_name](const auto& shapeID)
    {
      return _name == this->collisions.at(shapeID)->name;
    });

  if (it == link->shapes.end())
    return this->GenerateInvalidId();

  return this->GenerateIdentity(*it, this->collisions.at(*it));
}

const std::string &EntityManagementFeatures::GetJointName(
    const Identity &_jointID) const
{
  return this->ReferenceInterface<JointInfo>(_jointID)->name;
}

std::size_t EntityManagementFeatures::GetJointIndex(
    const Identity &_jointID) const
{
  const auto joint = this->ReferenceInterface<JointInfo>(_jointID);
  const auto parentLink = this->links.at(joint->parentLinkId);
  const auto model = this->ReferenceInterface<ModelInfo>(parentLink->model);
  const auto it =
      std::find(model->joints.begin(), model->joints.end(), _jointID.id);
  if (it == model->joints.end())
  {
    throw std::runtime_error(
          "Joint [" + std::to_string(_jointID.id) + "] cannot be found in "
          "model [" + std::to_string(parentLink->model.id) + "]");
  }

  return *it;
}

Identity EntityManagementFeatures::GetModelOfJoint(
    const Identity &_jointID) const
{
  const auto joint = this->ReferenceInterface<JointInfo>(_jointID);
  const auto parentLink = this->links.at(joint->parentLinkId);
  return parentLink->model;
}

const std::string &EntityManagementFeatures::GetShapeName(
    const Identity &_shapeID) const
{
  return this->ReferenceInterface<CollisionInfo>(_shapeID)->name;
}

std::size_t EntityManagementFeatures::GetShapeIndex(
    const Identity &_shapeID) const
{
  const auto shape = this->ReferenceInterface<CollisionInfo>(_shapeID);
  const auto link = this->ReferenceInterface<LinkInfo>(shape->link);
  const auto it =
      std::find(link->shapes.begin(), link->shapes.end(), _shapeID.id);
  if (it == link->shapes.end())
  {
    throw std::runtime_error(
          "Shape [" + std::to_string(_shapeID.id) + "] cannot be found in "
          "link [" + std::to_string(shape->link.id) + "]");
  }

  return *it;
}

Identity EntityManagementFeatures::GetLinkOfShape(
    const Identity &_shapeID) const
{
  return this->ReferenceInterface<CollisionInfo>(_shapeID)->link;
}
}  // namespace bullet
}  // namespace physics
}  // namespace gz
