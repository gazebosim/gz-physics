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

#include "SDFFeatures.hh"
#include <ignition/math/eigen3/Conversions.hh>

#include <sdf/JointAxis.hh>
#include <sdf/Box.hh>
#include <sdf/Geometry.hh>

namespace ignition {
namespace physics {
namespace bullet {

inline btMatrix3x3 convertMat(Eigen::Matrix3d mat)
{
  return btMatrix3x3(mat(0, 0), mat(0, 1), mat(0, 2),
                     mat(1, 0), mat(1, 1), mat(1, 2),
                     mat(2, 0), mat(2, 1), mat(2, 2));
}

inline btVector3 convertVec(Eigen::Vector3d vec)
{
  return btVector3(vec(0), vec(1), vec(2));
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const Identity &_engine,
    const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  const WorldInfoPtr &worldInfo = this->worlds.at(worldID);

  auto gravity = _sdfWorld.Gravity();
  worldInfo->world->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
  return worldID;
}

Identity SDFFeatures::ConstructSdfModel(
  const Identity &_worldID,
  const ::sdf::Model &_sdfModel)
{
  // Read sdf params
  const std::string name = _sdfModel.Name();
  const auto pose = _sdfModel.Pose();
  const bool isStatic = _sdfModel.Static();
  const bool selfCollide = _sdfModel.SelfCollide();

  // Set multibody params
  const int numLinks = _sdfModel.LinkCount();
  const btScalar baseMass = 0;
  const btVector3 baseInertiaDiag(0, 0, 0);
  const bool canSleep = false; // To Do: experiment with it

  // Set base transform
  const auto poseIsometry = ignition::math::eigen3::convert(pose);
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // Check if floating base
  bool fixedBase = isStatic;
  const bool hasWorldLink = _sdfModel.LinkNameExists("world");
  if (!hasWorldLink)
  {
    for (std::size_t i = 0; i < _sdfModel.JointCount(); ++i)
    {
      const auto &joint = _sdfModel.JointByIndex(i);
      if (joint->ParentLinkName() == "world")
      {
        fixedBase = true;
        break;
      }
    }
  }

  // Build model
  btMultiBody* model = new btMultiBody(numLinks,
                                       baseMass,
                                       baseInertiaDiag,
                                       fixedBase,
                                       canSleep);

  model->setBaseWorldTransform(baseTransform);

  model->setHasSelfCollision(selfCollide);
  model->setUseGyroTerm(true);

  model->setLinearDamping(0);
  model->setAngularDamping(0);
  model->setMaxCoordinateVelocity(1000); // Set to a large value

  const auto &world = this->worlds.at(_worldID)->world;
  world->addMultiBody(model);
  const auto modelIdentity = this->AddModel({model, name, _worldID});

  // Build links
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i)
  {
    this->BuildSdfLink(modelIdentity, *_sdfModel.LinkByIndex(i), i);
  }

  // Buld joints
  for (std::size_t i = 0; i < _sdfModel.JointCount(); ++i)
  {
    this->BuildSdfJoint(modelIdentity, *_sdfModel.JointByIndex(i));
  }

  // Finalize model
  btAlignedObjectArray<btQuaternion> scratch_q;
  btAlignedObjectArray<btVector3> scratch_m;
  model->forwardKinematics(scratch_q, scratch_m);
  btAlignedObjectArray<btQuaternion> world_to_local;
  btAlignedObjectArray<btVector3> local_origin;
  model->updateCollisionObjectWorldTransforms(world_to_local, local_origin);

  model->finalizeMultiDof();

  return modelIdentity;
}

Identity SDFFeatures::BuildSdfLink(
  const Identity &_modelID,
  const ::sdf::Link &_sdfLink,
  const int _linkIndex)
{
  // Read sdf params
  const std::string name = _sdfLink.Name();
  const auto pose = _sdfLink.Pose();
  const auto inertial = _sdfLink.Inertial();
  const auto mass = inertial.MassMatrix().Mass();
  const auto diagonalMoments = inertial.MassMatrix().DiagonalMoments();

  // Get link properties
  const btScalar linkMass = mass;
  const btVector3 linkInertiaDiag = convertVec(ignition::math::eigen3::convert(diagonalMoments));
  const auto poseIsometry = ignition::math::eigen3::convert(pose);

  // Add default fixed joints to links unless replaced by other joints
  // Find translation
  const btVector3 parentComToCurrentCom = convertVec(
    poseIsometry.translation());
  const btVector3 currentPivotToCurrentCom(0, 0, 0);
  const btVector3 parentComToCurrentPivot = parentComToCurrentCom -
    currentPivotToCurrentCom;
  // Find rotation
  btQuaternion rotParentToThis;
  const btMatrix3x3 mat = convertMat(poseIsometry.linear());
  mat.getRotation(rotParentToThis);

  // Set up fixed joints
  const int parentIndex = -1;
  const auto &model = this->models.at(_modelID)->model;
  model->setupFixed(_linkIndex, linkMass, linkInertiaDiag, parentIndex,
                     rotParentToThis, parentComToCurrentPivot,
                     currentPivotToCurrentCom);

  const auto linkIdentity = this->AddLink({name, _linkIndex, linkMass,
                                  linkInertiaDiag, poseIsometry, _modelID});

  // Build collisions
  for (std::size_t i = 0; i < _sdfLink.CollisionCount(); ++i)
  {
    this->BuildSdfCollision(linkIdentity, *_sdfLink.CollisionByIndex(i));
  }

  return linkIdentity;
}

Identity SDFFeatures::BuildSdfJoint(
  const Identity &_modelID,
  const ::sdf::Joint &_sdfJoint)
{
  // Read properties
  const std::string name = _sdfJoint.Name();
  const auto type = _sdfJoint.Type();
  const auto pose = _sdfJoint.Pose();
  // const auto threadPitch = _sdfJoint.ThreadPitch();
  const auto parentLinkName = _sdfJoint.ParentLinkName();
  const auto childLinkName = _sdfJoint.ChildLinkName();
  const auto firstAxis = _sdfJoint.Axis(0);
  const auto secondAxis = _sdfJoint.Axis(1);

  // Find parent and child link
  LinkInfoPtr parentLinkInfo;
  LinkInfoPtr childLinkInfo;
  for (const auto &entry : this->links)
  {
    const LinkInfoPtr &linkInfo = entry.second;
    if (linkInfo->model.id == _modelID.id)
    {
      if (linkInfo->name == parentLinkName)
      {
        parentLinkInfo = linkInfo;
      }
      if (linkInfo->name == childLinkName)
      {
        childLinkInfo = linkInfo;
      }
    }
  }

  const bool worldParent = (!parentLinkInfo) && (parentLinkName == "world");
  if (!parentLinkInfo && !worldParent)
  {
    ignerr << "Parent link with name [" << parentLinkName << "] could not be "
           << "found in the model\n";

    return this->GenerateInvalidId();
  }
  if (!childLinkInfo)
  {
    ignerr << "Child link with name [" << childLinkName << "] could not be "
           << "found in the model\n";

    return this->GenerateInvalidId();
  }

  const int parentIndex = worldParent ? -1 : parentLinkInfo->linkIndex;
  const int childIndex = childLinkInfo->linkIndex;

  // Get child link properties
  btVector3 jointAxis1 = btVector3(0, 0, 0);
  if (firstAxis != nullptr)
  {
    jointAxis1 = btVector3(firstAxis->Xyz()[0],
                           firstAxis->Xyz()[1],
                           firstAxis->Xyz()[2]);
  }
  btVector3 jointAxis2 = btVector3(0, 0, 0);
  if (secondAxis != nullptr)
  {
    jointAxis2 = btVector3(secondAxis->Xyz()[0],
                           secondAxis->Xyz()[1],
                           secondAxis->Xyz()[2]);
  }

  const auto poseIsometry = ignition::math::eigen3::convert(pose);

  // Obtain translation and rotation in world frame
  const btVector3 worldComToCurrentCom = convertVec(
    childLinkInfo->poseIsometry.translation());
  btVector3 worldComToParentCom = btVector3(0, 0, 0);
  if (parentLinkInfo != nullptr)
  {
    worldComToParentCom = convertVec(
      parentLinkInfo->poseIsometry.translation());
  }
  const btMatrix3x3 matWorldToThis = convertMat(
    childLinkInfo->poseIsometry.linear());
  btMatrix3x3 matWorldToParent = btMatrix3x3().getIdentity();
  if (parentLinkInfo != nullptr)
  {
    matWorldToParent = convertMat(
      parentLinkInfo->poseIsometry.linear());
  }

  const btMatrix3x3 matParentToThis = matWorldToParent.inverse() *
                                      matWorldToThis;

  // Obtain translation from parent to child
  btVector3 parentComToCurrentCom = worldComToCurrentCom -
                                    worldComToParentCom;
  parentComToCurrentCom = matWorldToParent.inverse() *
                          parentComToCurrentCom;

  const btVector3 currentComToCurrentPivot = convertVec(
    poseIsometry.translation());
  const btVector3 parentComToCurrentPivot = parentComToCurrentCom +
          matParentToThis * currentComToCurrentPivot;
  // Expressed in child frame
  const btVector3 currentPivotToCurrentCom = -currentComToCurrentPivot;

  // Obtain rotation that expresses vectors from parent frame in child
  // frame, which is different from the rotation that rotates vectors
  // from parent to child frame, hence the inverse().
  btQuaternion rotParentToThis;
  matParentToThis.inverse().getRotation(rotParentToThis);

  // TODO: Set joint damping
  // Bullet currently does not support joint damping in multibody.

  // Set up joints
  const auto &model = this->models.at(_modelID)->model;
  if (::sdf::JointType::REVOLUTE == type)
  {
    model->setupRevolute(childIndex, childLinkInfo->linkMass,
      childLinkInfo->linkInertiaDiag, parentIndex,
      rotParentToThis, jointAxis1, parentComToCurrentPivot,
      currentPivotToCurrentCom, !model->hasSelfCollision());
  }
  else if (::sdf::JointType::BALL == type)
  {
    model->setupSpherical(childIndex,
      childLinkInfo->linkMass, childLinkInfo->linkInertiaDiag,
      parentIndex, rotParentToThis, parentComToCurrentPivot,
      currentPivotToCurrentCom, !model->hasSelfCollision());
  }
  else if (::sdf::JointType::FIXED == type)
  {
    model->setupFixed(childIndex, childLinkInfo->linkMass,
      childLinkInfo->linkInertiaDiag, parentIndex,
      rotParentToThis, parentComToCurrentPivot, currentPivotToCurrentCom, !model->hasSelfCollision());
  }

  return this->AddJoint({name, type, childIndex, parentIndex, _modelID});
}

Identity SDFFeatures::BuildSdfCollision(
      const Identity &_linkID,
      const ::sdf::Collision &_collision)
{
  if (!_collision.Geom())
  {
    ignerr << "The geometry element of collision [" << _collision.Name() << "] "
           << "was a nullptr\n";
    return this->GenerateInvalidId();
  }

  const auto &geom = _collision.Geom();
  btCollisionShape* shape = nullptr;
  bool isDynamic = true;
  btTransform transform = btTransform().getIdentity();

  if (geom->BoxShape())
  {
    const auto box = geom->BoxShape();
    const auto size = math::eigen3::convert(box->Size());
    const auto halfExtents = convertVec(size)*0.5;
    shape = new btBoxShape(halfExtents);
  }
  else if (geom->SphereShape())
  {
    const auto sphere = geom->SphereShape();
    const auto radius = sphere->Radius();
    shape = new btSphereShape(radius);
  }
  else if (geom->CylinderShape())
  {
    const auto cylinder = geom->CylinderShape();
    const auto radius = cylinder->Radius();
    const auto halfLength = cylinder->Length()*0.5;
    shape = new btCylinderShapeZ(btVector3(radius, 0, halfLength));
  }
  else if (geom->PlaneShape())
  {
    const auto plane = geom->PlaneShape();
    const auto normal = convertVec(math::eigen3::convert(plane->Normal()));
    shape = new btStaticPlaneShape(normal, 0);
    isDynamic = false;
  }

  // Get friction
  const auto &odeFriction = _collision.Element()
                                ->GetElement("surface")
                                ->GetElement("friction")
                                ->GetElement("ode");
  const auto mu = odeFriction->Get<btScalar>("mu");

  if (shape != nullptr)
  {
    const auto &linkInfo = this->links.at(_linkID);
    const auto &modelID = linkInfo->model;
    const auto &modelInfo = this->models.at(modelID);
    const auto &model = modelInfo->model;
    const auto &world = this->worlds.at(modelInfo->world)->world;

    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(model,
                                      linkInfo->linkIndex);
    col->setCollisionShape(shape);
    col->setWorldTransform(transform);
    col->setFriction(mu);
    int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter)
                                  : int(btBroadphaseProxy::StaticFilter);
    int collisionFilterMask = isDynamic ? int (btBroadphaseProxy::AllFilter) :
      int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

    world->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);
    model->getLink(linkInfo->linkIndex).m_collider = col;

    return this->AddCollision({_collision.Name(), shape, col, _linkID,
                               modelID});
  }
  return this->GenerateInvalidId();
}

}
}
}
