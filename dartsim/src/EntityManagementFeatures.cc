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

#include <memory>
#include <string>
#include <unordered_map>

#include <dart/config.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/FreeJoint.hpp>

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionObject.hpp>

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
/// This class filters collision based on a bitmask:
/// Each objects has a bitmask. If the bitwise-and of two objects' bitmasks
/// evaluates to 0, then collisions between them are ignored.
class BitmaskContactFilter : public dart::collision::BodyNodeCollisionFilter
{
  public: using DartCollisionConstPtr = const dart::collision::CollisionObject*;
  public: using DartShapeConstPtr = const dart::dynamics::ShapeNode*;

  private: std::unordered_map<DartShapeConstPtr, uint16_t> bitmaskMap;

  public: bool ignoresCollision(
      DartCollisionConstPtr _object1,
      DartCollisionConstPtr _object2) const override
  {
    auto shapeNode1 = _object1->getShapeFrame()->asShapeNode();
    auto shapeNode2 = _object2->getShapeFrame()->asShapeNode();

    if (dart::collision::BodyNodeCollisionFilter::ignoresCollision(
          _object1, _object2))
      return true;

    auto shape1Iter = bitmaskMap.find(shapeNode1);
    auto shape2Iter = bitmaskMap.find(shapeNode2);
    if (shape1Iter != bitmaskMap.end() && shape2Iter != bitmaskMap.end() &&
        ((shape1Iter->second & shape2Iter->second) == 0))
      return true;

    return false;
  }

  public: void SetIgnoredCollision(DartShapeConstPtr _shapePtr,
      const uint16_t _mask)
  {
    bitmaskMap[_shapePtr] = _mask;
  }

  public: uint16_t GetIgnoredCollision(DartShapeConstPtr _shapePtr) const
  {
    auto shapeIter = bitmaskMap.find(_shapePtr);
    if (shapeIter != bitmaskMap.end())
      return shapeIter->second;
    return 0xff;
  }

  public: void RemoveIgnoredCollision(DartShapeConstPtr _shapePtr)
  {
    auto shapeIter = bitmaskMap.find(_shapePtr);
    if (shapeIter != bitmaskMap.end())
      bitmaskMap.erase(shapeIter);
  }

  public: void RemoveSkeletonCollisions(dart::dynamics::SkeletonPtr _skelPtr)
  {
    for (std::size_t i = 0; i < _skelPtr->getNumShapeNodes(); ++i)
    {
      auto shapePtr = _skelPtr->getShapeNode(i);
      this->RemoveIgnoredCollision(shapePtr);
    }
  }

  public: virtual ~BitmaskContactFilter() = default;
};

/// Utility functions
/////////////////////////////////////////////////
/// TODO (addisu): There's a lot of code duplication in model removal code, such
/// as RemoveModelByIndex, where we call GetFilterPtr followed by
/// RemoveSkeletonCollisions(model). To de-duplicate this, move this logic into
/// RemoveModelImpl. To do that, we need to move GetFilterPtr into Base.hh.
static const std::shared_ptr<BitmaskContactFilter> GetFilterPtr(
    const EntityManagementFeatures* _emf, std::size_t _worldID)
{
  const auto world = _emf->worlds.at(_worldID);
  // We need to cast the base class pointer to the derived class
  const auto filterPtr = std::static_pointer_cast<BitmaskContactFilter>(
      world->getConstraintSolver()->getCollisionOption()
      .collisionFilter);
  return filterPtr;
}

/////////////////////////////////////////////////
static std::size_t GetWorldOfShapeNode(const EntityManagementFeatures *_emf,
    const dart::dynamics::ShapeNode* _shapeNode)
{
  // Get the body of the shape node
  const auto bn = _shapeNode->getBodyNodePtr();
  // Get the body node's skeleton
  const auto skelPtr = bn->getSkeleton();
  // Now find the skeleton's model
  const std::size_t modelID = _emf->models.objectToID.at(skelPtr);
  // And the world containing the model
  return _emf->GetWorldOfModelImpl(modelID);
}

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
  // dart::simulation::World::getNumSkeletons returns all the skeletons in the
  // world, including nested ones. We use the size of the indexInContainerToID
  // vector associated with the _worldID to determine the number of models
  // that are direct children of the world.
  return this->models.indexInContainerToID.at(_worldID).size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
    const Identity &_worldID, const std::size_t _modelIndex) const
{
  const auto &indexInContainerToID =
      this->models.indexInContainerToID.at(_worldID);

  if (_modelIndex >= indexInContainerToID.size())
  {
    return this->GenerateInvalidId();
  }
  const std::size_t modelID = indexInContainerToID[_modelIndex];

  // If the model doesn't exist in "models", it means the containing entity has
  // been removed.
  if (this->models.HasEntity(modelID))
  {
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
  return this->ReferenceInterface<ModelInfo>(_modelID)->localName;
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
  auto worldID = this->GetWorldOfModelImpl(_modelID);
  if (worldID != INVALID_ENTITY_ID )
  {
    return this->GenerateIdentity(worldID, this->worlds.at(worldID));
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetNestedModelCount(
    const Identity &_modelID) const
{
  return this->ReferenceInterface<ModelInfo>(_modelID)->nestedModels.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetNestedModel(
    const Identity &_modelID, const std::size_t _modelIndex) const
{
  const auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  if (_modelIndex >= modelInfo->nestedModels.size())
  {
    return this->GenerateInvalidId();
  }

  const auto nestedModelID = modelInfo->nestedModels[_modelIndex];

  // If the model doesn't exist in "models", it means the containing entity has
  // been removed.
  if (this->models.HasEntity(nestedModelID))
  {
    return this->GenerateIdentity(nestedModelID,
                                  this->models.at(nestedModelID));
  }
  else
  {
    return this->GenerateInvalidId();
  }
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetNestedModel(
    const Identity &_modelID, const std::string &_modelName) const
{
  const auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);

  const std::string fullName =
      ::sdf::JoinName(modelInfo->model->getName(), _modelName);

  if (this->models.HasEntity(_modelID))
  {
    auto worldID = this->GetWorldOfModelImpl(_modelID);
    auto nestedSkel = this->worlds.at(worldID)->getSkeleton(fullName);
    if (nullptr == nestedSkel)
    {
      return this->GenerateInvalidId();
    }
    const std::size_t nestedModelID = this->models.IdentityOf(nestedSkel);
    return this->GenerateIdentity(nestedModelID,
                                  this->models.at(nestedModelID));
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
      ->links.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetLink(
    const Identity &_modelID, const std::size_t _linkIndex) const
{
  auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);

  if (_linkIndex >= modelInfo->links.size())
    return this->GenerateInvalidId();

  const auto &linkInfo = modelInfo->links[_linkIndex];

  // If the link doesn't exist in "links", it means the containing entity has
  // been removed.
  if (this->links.HasEntity(linkInfo->link))
  {
    const std::size_t linkID = this->links.IdentityOf(linkInfo->link);
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
  const auto &modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  for (const auto &linkInfo : modelInfo->links)
  {
    if (_linkName == linkInfo->name)
    {
      // If the link doesn't exist in "links", it means the containing entity
      // has been removed.
      if (this->links.HasEntity(linkInfo->link))
      {
        const std::size_t linkID = this->links.IdentityOf(linkInfo->link);
        return this->GenerateIdentity(linkID, this->links.at(linkID));
      }
      else
      {
        // TODO(addisu) It's not clear what to do when `GetLink` is called on a
        // model that has been removed. Right now we are returning an invalid
        // identity, but that could cause a segfault if the user doesn't check
        // the returned value before using it.
        return this->GenerateInvalidId();
      }
    }
  }
  return this->GenerateInvalidId();
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
  return this->ReferenceInterface<LinkInfo>(_linkID)->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetLinkIndex(
    const Identity &_linkID) const
{
  return this->links.idToIndexInContainer.at(_linkID);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModelOfLink(
    const Identity &_linkID) const
{
  const std::size_t modelID = this->links.idToContainerID.at(_linkID);

  // If the model containing the link doesn't exist in "models", it means this
  // link belongs to a removed model.
  if (this->models.HasEntity(modelID))
  {
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
    auto filterPtr = GetFilterPtr(this, _worldID);
    filterPtr->RemoveSkeletonCollisions(model);
    return this->RemoveModelImpl(_worldID, this->models.IdentityOf(model));
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
    auto filterPtr = GetFilterPtr(this, _worldID);
    filterPtr->RemoveSkeletonCollisions(model);
    return this->RemoveModelImpl(_worldID, this->models.IdentityOf(model));
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  if (this->models.HasEntity(_modelID))
  {
    auto worldID = this->GetWorldOfModelImpl(_modelID);
    auto model = this->models.at(_modelID)->model;

    auto filterPtr = GetFilterPtr(this, worldID);
    filterPtr->RemoveSkeletonCollisions(model);

    return this->RemoveModelImpl(worldID, _modelID);
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::ModelRemoved(const Identity &_modelID) const
{
  return !this->models.HasEntity(_modelID);
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveNestedModelByIndex(
    const Identity &_modelID, std::size_t _nestedModelIndex)
{
  auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  if (_nestedModelIndex >= modelInfo->nestedModels.size())
  {
    return this->GenerateInvalidId();
  }
  const auto nestedModelID = modelInfo->nestedModels[_nestedModelIndex];
  if (this->models.HasEntity(nestedModelID))
  {
    const auto worldID = this->GetWorldOfModelImpl(nestedModelID);
    const auto model = this->models.at(nestedModelID)->model;
    const auto filterPtr = GetFilterPtr(this, worldID);
    filterPtr->RemoveSkeletonCollisions(model);
    return this->RemoveModelImpl(worldID, nestedModelID);
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveNestedModelByName(const Identity &_modelID,
                                                 const std::string &_modelName)
{
  auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  const std::string fullName =
      ::sdf::JoinName(modelInfo->model->getName(), _modelName);

  if (this->models.HasEntity(_modelID))
  {
    auto worldID = this->GetWorldOfModelImpl(_modelID);
    auto nestedSkel = this->worlds.at(worldID)->getSkeleton(fullName);
    if (nullptr == nestedSkel || !this->models.HasEntity(nestedSkel))
    {
      return false;
    }
    const std::size_t nestedModelID = this->models.IdentityOf(nestedSkel);
    const auto filterPtr = GetFilterPtr(this, worldID);
    filterPtr->RemoveSkeletonCollisions(nestedSkel);
    return this->RemoveModelImpl(worldID, nestedModelID);
  }
  return false;
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
      std::make_shared<BitmaskContactFilter>();

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

  const auto [modelID, modelInfo] =
      this->AddModel({model, _name, modelFrame, ""}, _worldID);  // NOLINT

  return this->GenerateIdentity(modelID, this->models.at(modelID));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyNestedModel(
    const Identity &_parentModelID, const std::string &_name)
{
  // find the world assocated with the model
  auto worldID = this->GetWorldOfModelImpl(_parentModelID);
  const auto &skel = this->models.at(_parentModelID)->model;
  const std::string modelFullName = ::sdf::JoinName(skel->getName(), _name);

  dart::dynamics::SkeletonPtr model =
      dart::dynamics::Skeleton::create(modelFullName);

  dart::dynamics::SimpleFramePtr modelFrame =
      dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        modelFullName + "_frame");

  auto [modelID, modelInfo] = this->AddNestedModel(
      {model, _name, modelFrame, ""}, _parentModelID, worldID);  // NOLINT

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

  auto worldID = this->GetWorldOfModelImpl(_modelID);
  if (worldID == INVALID_ENTITY_ID)
  {
    ignerr << "World of model [" << model->getName()
           << "] could not be found when creating link [" << _name
           << "]\n";
    return this->GenerateInvalidId();
  }

  auto world = this->worlds.at(worldID);
  const std::string fullName = ::sdf::JoinName(
      world->getName(),
      ::sdf::JoinName(model->getName(), bn->getName()));

  const std::size_t linkID = this->AddLink(bn, fullName, _modelID);
  return this->GenerateIdentity(linkID, this->links.at(linkID));
}

void EntityManagementFeatures::SetCollisionFilterMask(
    const Identity &_shapeID, const uint16_t _mask)
{
  const auto shapeNode = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  const std::size_t worldID = GetWorldOfShapeNode(this, shapeNode);
  const auto filterPtr = GetFilterPtr(this, worldID);
  filterPtr->SetIgnoredCollision(shapeNode, _mask);
}

uint16_t EntityManagementFeatures::GetCollisionFilterMask(
    const Identity &_shapeID) const
{
  const auto shapeNode = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  const std::size_t worldID = GetWorldOfShapeNode(this, shapeNode);
  const auto filterPtr = GetFilterPtr(this, worldID);
  return filterPtr->GetIgnoredCollision(shapeNode);
}

void EntityManagementFeatures::RemoveCollisionFilterMask(
    const Identity &_shapeID)
{
  const auto shapeNode = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  const std::size_t worldID = GetWorldOfShapeNode(this, shapeNode);
  const auto filterPtr = GetFilterPtr(this, worldID);
  filterPtr->RemoveIgnoredCollision(shapeNode);
}

}
}
}
