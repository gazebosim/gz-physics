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
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/FreeJoint.hpp>

#include <string>

namespace ignition {
namespace physics {
namespace dartsim {

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
  //TODO(anyone) this will throw if the model has been removed
  return this->models.idToIndexInContainer.at(_modelID);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorldOfModel(
    const Identity &_modelID) const
{
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
Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, const std::string &_linkName) const
{
  DartBodyNode *const bn =
      this->ReferenceInterface<ModelInfo>(_modelID)->model->getBodyNode(
          _linkName);

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
std::size_t EntityManagementFeatures::GetJointCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->model->getNumJoints();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetJoint(
    const Identity &_modelID, const std::size_t _jointIndex) const
{
  DartJoint *const jt =
      this->ReferenceInterface<ModelInfo>(_modelID)->model->getJoint(
          _jointIndex);

  if (this->joints.HasEntity(jt))
  {
    const std::size_t jointID = this->joints.IdentityOf(jt);
    return this->GenerateIdentity(jointID, this->joints.at(jointID));
  }
  else
  {
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetJoint(
    const Identity &_modelID, const std::string &_jointName) const
{
  DartJoint *const jt =
      this->ReferenceInterface<ModelInfo>(_modelID)->model->getJoint(
          _jointName);

  if (this->joints.HasEntity(jt))
  {
    const std::size_t jointID = this->joints.IdentityOf(jt);
    return this->GenerateIdentity(jointID, this->joints.at(jointID));
  }
  else
  {
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

  if (this->shapes.HasEntity(sn))
  {
    const std::size_t shapeID = this->shapes.IdentityOf(sn);
    return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
  }
  else
  {
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

  if (this->shapes.HasEntity(sn))
  {
    const std::size_t shapeID = this->shapes.IdentityOf(sn);
    return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
  }
  else
  {
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
  DartBodyNode * const bn = shapeInfo->node->getBodyNodePtr();

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
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  const auto &world = std::make_shared<dart::simulation::World>(_name);
  world->getConstraintSolver()->setCollisionDetector(
        dart::collision::BulletCollisionDetector::create());

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
