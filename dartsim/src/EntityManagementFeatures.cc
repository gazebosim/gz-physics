/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "EntityManagementFeatures.hh"

#include <dart/config.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/FreeJoint.hpp>

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionObject.hpp>

#include <string>

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
/// This class helps to resolve an issue with excessive contacts being computed:
/// https://github.com/ignitionrobotics/ign-physics/issues/11/
///
/// TODO(MXG): Delete this class when we switch to using dartsim-6.8:
/// https://github.com/dartsim/dart/pull/1232
class ImmobileContactFilter : public dart::collision::BodyNodeCollisionFilter
{
  public: bool ignoresCollision(
      const dart::collision::CollisionObject *_object1,
      const dart::collision::CollisionObject *_object2) const override
  {
    const auto &shapeNode1 = _object1->getShapeFrame()->asShapeNode();
    const auto &shapeNode2 = _object2->getShapeFrame()->asShapeNode();

    if (!shapeNode1 || !shapeNode2)
      return false;

    const auto &skeleton1 = shapeNode1->getSkeleton();
    const auto &skeleton2 = shapeNode2->getSkeleton();

    if (!skeleton1->isMobile() && !skeleton2->isMobile())
      return true;

    return BodyNodeCollisionFilter::ignoresCollision(_object1, _object2);
  }
};

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetEngineName(
    const Identity &/*_engineID*/) const
{
  static const std::string engineName = "dartsim-" DART_VERSION;
  return engineName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetEngineIndex(
    const Identity &/*_engineID*/) const
{
  // The dartsim plugin does not make a distinction between different engine
  // indexes.
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
  const std::size_t id =
      this->worlds.indexInContainerToID.begin()->second[_worldIndex];
  return this->GenerateIdentity(id, this->worlds.idToObject.at(id));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
    const Identity &, const std::string &_worldName) const
{
  const std::size_t id = this->worlds.IdentityOf(_worldName);
  return this->GenerateIdentity(id, this->worlds.idToObject.at(id));
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetWorldName(
    const Identity &_worldID) const
{
  return this->ReferenceInterface<DartWorld>(_worldID)->getName();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldIndex(
    const Identity &_worldID) const
{
  // TODO(anyone) this will throw if the world has been removed
  return this->worlds.idToIndexInContainer.at(_worldID);
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
  return this->ReferenceInterface<DartWorld>(_worldID)->getNumSkeletons();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
    const Identity &_worldID, const std::size_t _modelIndex) const
{
  const DartSkeletonPtr &model =
      this->ReferenceInterface<DartWorld>(_worldID)->getSkeleton(_modelIndex);

  // If the model doesn't exist in "models", it means the containing entity has
  // been removed.
  if (this->models.HasEntity(model))
  {
    const std::size_t modelID = this->models.IdentityOf(model);
    return this->GenerateIdentity(modelID, this->models.at(modelID));
  }
  else
  {
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
    const Identity &_worldID, const std::string &_modelName) const
{
  const DartSkeletonPtr &model =
      this->ReferenceInterface<DartWorld>(_worldID)->getSkeleton(_modelName);

  // If the model doesn't exist in "models", it means the containing entity has
  // been removed.
  if (this->models.HasEntity(model))
  {
    const std::size_t modelID = this->models.IdentityOf(model);
    return this->GenerateIdentity(modelID, this->models.at(modelID));
  }
  else
  {
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetModelName(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->model->getName();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelIndex(
    const Identity &_modelID) const
{
  // TODO(anyone) this will throw if the model has been removed. The alternative
  // is to first check if the model exists, but what should we return if it
  // doesn't exist
  return this->models.idToIndexInContainer.at(_modelID);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorldOfModel(
    const Identity &_modelID) const
{
  // If the model doesn't exist in "models", it it has been removed.
  if (this->models.HasEntity(_modelID))
  {
    const std::size_t worldID = this->models.idToContainerID.at(_modelID);
    return this->GenerateIdentity(worldID, this->worlds.at(worldID));
  }
  else
  {
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)
      ->model->getNumBodyNodes();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, const std::size_t _linkIndex) const
{
  DartBodyNode *const bn =
      this->ReferenceInterface<ModelInfo>(_modelID)->model->getBodyNode(
          _linkIndex);

  // If the link doesn't exist in "links", it means the containing entity has
  // been removed.
  if (this->links.HasEntity(bn))
  {
    const std::size_t linkID = this->links.IdentityOf(bn);
    return this->GenerateIdentity(linkID, this->links.at(linkID));
  }
  else
  {
    // TODO(addisu) It's not clear what to do when `GetLink` is called on a
    // model that has been removed. Right now we are returning an invalid
    // identity, but that could cause a segfault if the use doesn't check if
    // returned value before using it.
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, const std::string &_linkName) const
{
  DartBodyNode *const bn =
      this->ReferenceInterface<ModelInfo>(_modelID)->model->getBodyNode(
          _linkName);

  // If the link doesn't exist in "links", it means the containing entity has
  // been removed.
  if (this->links.HasEntity(bn))
  {
    const std::size_t linkID = this->links.IdentityOf(bn);
    return this->GenerateIdentity(linkID, this->links.at(linkID));
  }
  else
  {
    // TODO(addisu) It's not clear what to do when `GetLink` is called on a
    // model that has been removed. Right now we are returning an invalid
    // identity, but that could cause a segfault if the use doesn't check if
    // returned value before using it.
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetJointCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->model->getNumJoints();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetJoint(
    const Identity &_modelID, const std::size_t _jointIndex) const
{
  DartJoint *const joint =
      this->ReferenceInterface<ModelInfo>(_modelID)->model->getJoint(
          _jointIndex);

  // If the joint doesn't exist in "joints", it means the containing entity has
  // been removed.
  if (this->joints.HasEntity(joint))
  {
    const std::size_t jointID = this->joints.IdentityOf(joint);
    return this->GenerateIdentity(jointID, this->joints.at(jointID));
  }
  else
  {
    // TODO(addisu) It's not clear what to do when `GetJoint` is called on a
    // model that has been removed. Right now we are returning an invalid
    // identity, but that could cause a segfault if the use doesn't check if
    // returned value before using it.
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetJoint(
    const Identity &_modelID, const std::string &_jointName) const
{
  DartJoint *const joint =
      this->ReferenceInterface<ModelInfo>(_modelID)->model->getJoint(
          _jointName);

  // If the joint doesn't exist in "joints", it means the containing entity has
  // been removed.
  if (this->joints.HasEntity(joint))
  {
    const std::size_t jointID = this->joints.IdentityOf(joint);
    return this->GenerateIdentity(jointID, this->joints.at(jointID));
  }
  else
  {
    // TODO(addisu) It's not clear what to do when `GetJoint` is called on a
    // model that has been removed. Right now we are returning an invalid
    // identity, but that could cause a segfault if the use doesn't check if
    // returned value before using it.
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetLinkName(
    const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->link->getName();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkIndex(
    const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)
      ->link->getIndexInSkeleton();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfLink(
    const Identity &_linkID) const
{
  const DartSkeletonPtr &model =
      this->ReferenceInterface<LinkInfo>(_linkID)->link->getSkeleton();

  // If the model containing the link doesn't exist in "models", it means this
  // link belongs to a removed model.
  if (this->models.HasEntity(model))
  {
    const std::size_t modelID = this->models.IdentityOf(model);
    return this->GenerateIdentity(modelID, this->models.at(modelID));
  }
  else
  {
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeCount(
    const Identity &_linkID) const
{
  return this->ReferenceInterface<LinkInfo>(_linkID)->link->getNumShapeNodes();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
    const Identity &_linkID, const std::size_t _shapeIndex) const
{
  DartShapeNode *const sn =
      this->ReferenceInterface<LinkInfo>(_linkID)->link->getShapeNode(
          _shapeIndex);

  // If the shape doesn't exist in "shapes", it means the containing entity has
  // been removed.
  if (this->shapes.HasEntity(sn))
  {
    const std::size_t shapeID = this->shapes.IdentityOf(sn);
    return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
  }
  else
  {
    // TODO(addisu) It's not clear what to do when `GetShape` is called on a
    // link that has been removed. Right now we are returning an invalid
    // identity, but that could cause a segfault if the use doesn't check if
    // returned value before using it.
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetShape(
    const Identity &_linkID, const std::string &_shapeName) const
{
  auto bn = this->ReferenceInterface<LinkInfo>(_linkID)->link;

  DartShapeNode *const sn = bn->getSkeleton()->getShapeNode(
          bn->getName() + ":" + _shapeName);

  // If the shape doesn't exist in "shapes", it means the containing entity has
  // been removed.
  if (this->shapes.HasEntity(sn))
  {
    const std::size_t shapeID = this->shapes.IdentityOf(sn);
    return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
  }
  else
  {
    // TODO(addisu) It's not clear what to do when `GetShape` is called on a
    // link that has been removed. Right now we are returning an invalid
    // identity, but that could cause a segfault if the use doesn't check if
    // returned value before using it.
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetJointName(
    const Identity &_jointID) const
{
  return this->ReferenceInterface<JointInfo>(_jointID)->joint->getName();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetJointIndex(
    const Identity &_jointID) const
{
  return this->ReferenceInterface<JointInfo>(_jointID)
      ->joint->getJointIndexInSkeleton();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfJoint(
    const Identity &_jointID) const
{
  const DartSkeletonPtr &model =
      this->ReferenceInterface<JointInfo>(_jointID)->joint->getSkeleton();

  // If the model containing the joint doesn't exist in "models", it means this
  // joint belongs to a removed model.
  if (this->models.HasEntity(model))
  {
    const std::size_t modelID = this->models.IdentityOf(model);
    return this->GenerateIdentity(modelID, this->models.at(modelID));
  }
  else
  {
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetShapeName(
    const Identity &_shapeID) const
{
  const auto shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  return shapeInfo->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetShapeIndex(
    const Identity &_shapeID) const
{
  const auto shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  return shapeInfo->node->getIndexInBodyNode();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLinkOfShape(
    const Identity &_shapeID) const
{
  auto shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  DartBodyNode *const bn = shapeInfo->node->getBodyNodePtr();

  // If the link containing the shape doesn't exist in "links", it means this
  // shape belongs to a removed link.
  if (this->links.HasEntity(bn))
  {
    const std::size_t linkID = this->links.IdentityOf(bn);
    return this->GenerateIdentity(linkID, this->links.at(linkID));
  }
  else
  {
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByIndex(const Identity &_worldID,
                                                  std::size_t _modelIndex)
{
  auto *const world = this->ReferenceInterface<DartWorld>(_worldID);
  const DartSkeletonPtr &model = world->getSkeleton(_modelIndex);

  if (model != nullptr && this->models.HasEntity(model))
  {
    this->RemoveModelImpl(_worldID, this->models.IdentityOf(model));
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModelByName(const Identity &_worldID,
                                                 const std::string &_modelName)
{
  auto *const world = this->ReferenceInterface<DartWorld>(_worldID);
  const DartSkeletonPtr &model = world->getSkeleton(_modelName);

  if (model != nullptr && this->models.HasEntity(model))
  {
    this->RemoveModelImpl(_worldID, this->models.IdentityOf(model));
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  if (this->models.HasEntity(_modelID))
  {
    this->RemoveModelImpl(this->models.idToContainerID.at(_modelID), _modelID);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::ModelRemoved(const Identity &_modelID) const
{
  return !this->models.HasEntity(_modelID);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  const auto &world = std::make_shared<dart::simulation::World>(_name);
  world->getConstraintSolver()->setCollisionDetector(
        dart::collision::OdeCollisionDetector::create());

  // TODO(anyone) We need a machanism to configure maxNumContacts at runtime.
  auto &collOpt = world->getConstraintSolver()->getCollisionOption();
  collOpt.maxNumContacts = 10000;

  world->getConstraintSolver()->getCollisionOption().collisionFilter =
      std::make_shared<ImmobileContactFilter>();

  const std::size_t worldID = this->AddWorld(world, _name);
  return this->GenerateIdentity(worldID, this->worlds.at(worldID));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyModel(
    const Identity &_worldID, const std::string &_name)
{
  dart::dynamics::SkeletonPtr model = dart::dynamics::Skeleton::create(_name);

  dart::dynamics::SimpleFramePtr modelFrame =
      dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        _name + "_frame");

  auto [modelID, modelInfo] = this->AddModel({model, modelFrame}, _worldID); // NOLINT

  return this->GenerateIdentity(modelID, this->models.at(modelID));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyLink(
    const Identity &_modelID, const std::string &_name)
{
  auto model = this->ReferenceInterface<ModelInfo>(_modelID)->model;

  dart::dynamics::FreeJoint::Properties prop_fj;
  prop_fj.mName = _name + "_FreeJoint";

  DartBodyNode::Properties prop_bn;
  prop_bn.mName = _name;

  DartBodyNode *bn =
      model->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
        nullptr, prop_fj, prop_bn).second;

  const std::size_t linkID = this->AddLink(bn);
  return this->GenerateIdentity(linkID, this->links.at(linkID));
}

}
}
}
