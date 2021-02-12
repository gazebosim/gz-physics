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

#include "SDFFeatures.hh"
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/math/Helpers.hh>

#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Plane.hh>
#include <sdf/JointAxis.hh>

namespace ignition {
namespace physics {
namespace bullet {

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

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
    const Identity &_worldID,
    const ::sdf::Model &_sdfModel)
{
  // Read sdf params
  const std::string name = _sdfModel.Name();
  const math::Pose3d pose = _sdfModel.RawPose();
  const bool isStatic = _sdfModel.Static();
  // const bool selfCollide = _sdfModel.SelfCollide();

  const auto modelIdentity = this->AddModel({name, _worldID, isStatic, pose});

  // After creating all the links, join the ones that have joints
  for (std::size_t i=0; i < _sdfModel.JointCount(); ++i)
  {
    igndbg << "Loop adding joints.\n";

    const ::sdf::Joint *sdfJoint = _sdfModel.JointByIndex(i);
    if (!sdfJoint)
    {
      ignerr << "The joint with index [" << i << "] in model ["
             << _sdfModel.Name() << "] is a nullptr. It will be skipped.\n";
      continue;
    }
  }

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfLink(
    const Identity &_modelID,
    const ::sdf::Link &_sdfLink)
{
  // Read sdf params
  const std::string name = _sdfLink.Name();
  const math::Pose3d pose = _sdfLink.RawPose();
  const ignition::math::Inertiald inertial = _sdfLink.Inertial();
  auto mass = inertial.MassMatrix().Mass();
  const auto diagonalMoments = inertial.MassMatrix().DiagonalMoments();

  // Get link properties
  btVector3 linkInertiaDiag =
    convertVec(ignition::math::eigen3::convert(diagonalMoments));

  const auto &modelInfo = this->models.at(_modelID);
  math::Pose3d base_pose = modelInfo->pose;
  const auto poseIsometry = ignition::math::eigen3::convert(base_pose * pose);
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // Create link
  // (TO-DO: do we want to use MotionState?) 2nd part: Do motion state use the same transformation?
  if (this->models.at(_modelID)->fixed)
  {
    mass = 0;
    linkInertiaDiag = btVector3(0,0,0);
  }

  btDefaultMotionState* myMotionState = new btDefaultMotionState(baseTransform);
  btCollisionShape* collisionShape = new btCompoundShape();
  btRigidBody::btRigidBodyConstructionInfo
    rbInfo(mass, myMotionState, collisionShape, linkInertiaDiag);
  btRigidBody* body = new btRigidBody(rbInfo);
  body->setActivationState(DISABLE_DEACTIVATION);

  const auto &world = this->worlds.at(modelInfo->world)->world;

  // Models collide with everything except themselves
  const int modelCollisionGroup = 1 << this->collisionGroups.at(_modelID);
  const int collisionMask = 0xFFFFFFFF & ~modelCollisionGroup;
  world->addRigidBody(body, modelCollisionGroup, collisionMask);
  //world->addRigidBody(body);

  // Generate an identity for it
  const auto linkIdentity = this->AddLink({name, body, _modelID, pose, mass, linkInertiaDiag});
  return linkIdentity;
}

Identity SDFFeatures::ConstructSdfCollision(
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
    shape = new btCylinderShapeZ(btVector3(radius, radius, halfLength));
  }
  else if (geom->PlaneShape())
  {
    const auto plane = geom->PlaneShape();
    const auto normal = convertVec(math::eigen3::convert(plane->Normal()));
    shape = new btStaticPlaneShape(normal, 0);
  }

  // TODO(lobotuerk) find if there's a  way to use friction and related
  // info in bullet dynamics

  const auto &surfaceElement = _collision.Element()->GetElement("surface");

  // Get friction
  const auto &odeFriction = surfaceElement->GetElement("friction")
                              ->GetElement("ode");
  const auto mu = odeFriction->Get<btScalar>("mu");
  const auto mu2 = odeFriction->Get<btScalar>("mu2");
  const auto mu3 = odeFriction->Get<btScalar>("mu3");

  // Get restitution
  // const auto restitution = surfaceElement->GetElement("bounce")
  //                             ->Get<btScalar>("restitution_coefficient");
  if (shape != nullptr)
  {
    const auto &linkInfo = this->links.at(_linkID);
    const auto &body = linkInfo->link;
    const auto &modelID = linkInfo->model;

    // TODO(LOBOTUERK) figure out why this was here
    // if (!modelInfo->fixed){
    //   return this->GenerateInvalidId();
    // }

    const math::Pose3d pose = _collision.RawPose();
    const Eigen::Isometry3d poseIsometry = ignition::math::eigen3::convert(pose);
    const Eigen::Vector3d poseTranslation = poseIsometry.translation();
    const auto poseLinear = poseIsometry.linear();
    btTransform baseTransform;
    baseTransform.setOrigin(convertVec(poseTranslation));
    baseTransform.setBasis(convertMat(poseLinear));

    // TODO(Blast545): Consider different approaches to set frictions
    // shape->setMargin(btScalar(0.0001));
    body->setFriction(1);
    body->setAnisotropicFriction(btVector3(mu, mu2, mu3),
    btCollisionObject::CF_ANISOTROPIC_FRICTION);

    dynamic_cast<btCompoundShape *>(body->getCollisionShape())->addChildShape(baseTransform, shape);

    auto identity = this->AddCollision({_collision.Name(), shape, _linkID,
      modelID, pose});
    return identity;
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(
    const Identity &_modelID,
    const ::sdf::Joint &_sdfJoint)
{
  // const auto &parentModelInfo = *this->ReferenceInterface<ModelInfo>(_modelID);

  // Check supported Joints
  const ::sdf::JointType type = _sdfJoint.Type();
  if( type != ::sdf::JointType::REVOLUTE && type != ::sdf::JointType::FIXED )
  {
    ignerr << "Asked to construct a joint of sdf::JointType ["
           << static_cast<int>(type) << "], but that is not supported yet.\n";
    return this->GenerateInvalidId();
  }

  // Get the parent and child ids
  const std::string parentLinkName = _sdfJoint.ParentLinkName();
  std::size_t parentId = this->FindSdfLink(_modelID, parentLinkName);

  const std::string childLinkName = _sdfJoint.ChildLinkName();
  std::size_t childId = this->FindSdfLink(_modelID, childLinkName);

  // Check if chilId and parentId are valid values
  const auto invalidEntity = this->GenerateInvalidId().id;
  if (parentId == invalidEntity || childId == invalidEntity)
  {
    ignerr << "There was a problem finding/creating parent/child links\n";
    return this->GenerateInvalidId();
  }

  // Handle the case where either child is the world, not supported
  const std::size_t worldId = this->models.at(_modelID)->world;
  if (childId == worldId)
  {
    ignwarn << "Not implemented joints using world as child\n";
    return this->GenerateInvalidId();
  }

  // Get axis unit vector (expressed in world frame).
  // IF fixed joint, use UnitZ, if revolute use the Axis given by the joint
  // Eigen::Vector3d axis;
  //ignition::math::Vector3d axis = ignition::math::Vector3d::UnitZ;
  ignition::math::Vector3d axis;
  if(type == ::sdf::JointType::FIXED )
  {
    axis = ignition::math::Vector3d::UnitZ;
  }
  else
  {
    axis = (_sdfJoint.RawPose() + this->links.at(childId)->pose).Rot() * _sdfJoint.Axis(0)->Xyz();
  }

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  math::Vector3d pivotParent, pivotChild, axisParent, axisChild;
  math::Pose3d pose;

  if (parentId != worldId)
  {
    pivotParent = (_sdfJoint.RawPose() + this->links.at(childId)->pose).Pos();
    pose = this->links.at(parentId)->pose;
    pivotParent -= pose.Pos();
    pivotParent = pose.Rot().RotateVectorReverse(pivotParent);
    axisParent = pose.Rot().RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
  }

  pivotChild = (_sdfJoint.RawPose() + this->links.at(childId)->pose).Pos();
  pose = this->links.at(childId)->pose;
  pivotChild -= pose.Pos();
  pivotChild = pose.Rot().RotateVectorReverse(pivotChild);
  axisChild = pose.Rot().RotateVectorReverse(axis);
  axisChild = axisChild.Normalize();

  btHingeAccumulatedAngleConstraint* joint;
  if (parentId != worldId)
  {
    joint = new btHingeAccumulatedAngleConstraint(
      *this->links.at(childId)->link,
      *this->links.at(parentId)->link,
      convertVec(ignition::math::eigen3::convert(pivotChild)),
      convertVec(ignition::math::eigen3::convert(pivotParent)),
      convertVec(ignition::math::eigen3::convert(axisChild)),
      convertVec(ignition::math::eigen3::convert(axisParent)));
  }
  else
  {
    joint = new btHingeAccumulatedAngleConstraint(
      *this->links.at(childId)->link,
      convertVec(ignition::math::eigen3::convert(pivotChild)),
      convertVec(ignition::math::eigen3::convert(axisChild)));
  }

  // Limit movement for fixed joints
  if(type == ::sdf::JointType::FIXED)
  {
    btScalar offset = joint->getHingeAngle();
    joint->setLimit(offset, offset);
  }

  const auto &modelInfo = this->models.at(_modelID);
  const auto &world = this->worlds.at(modelInfo->world)->world;
  world->addConstraint(joint, true);
  joint->enableFeedback(true);

  /* TO-DO(Lobotuerk): find how to implement axis friction properly for bullet*/
  if (_sdfJoint.Axis(0) != nullptr)
  {
    double friction = _sdfJoint.Axis(0)->Friction();
    joint->enableAngularMotor(true, 0.0, friction);
    joint->setLimit(_sdfJoint.Axis(0)->Lower(), _sdfJoint.Axis(0)->Upper());
  }
  else
  {
    joint->enableAngularMotor(true, 0.0, 0.0);
  }

  // Generate an identity for it and return it
  auto identity =
    this->AddJoint({_sdfJoint.Name(), joint, childId, parentId, static_cast<int>(type), axis});
  return identity;
}

/////////////////////////////////////////////////
std::size_t SDFFeatures::FindSdfLink(
    const Identity &_modelID,
    const std::string &_sdfLinkName)
{
  for (const auto &link : this->links)
  {
    const auto &linkInfo = link.second;
    if (linkInfo->name == _sdfLinkName && linkInfo->model.id == _modelID.id)
    {
      // A link was previously created with that name,
      // Return its entity value
      return link.first;
    }
  }

  // Link wasn't found, check if the requested link is "world"
  if (_sdfLinkName == "world")
  {
    // Return the ID of the parent world of the model
    return this->models.at(_modelID)->world;
  }
  else
  {
    ignerr << "Model does not contain a link named [" << _sdfLinkName << "].\n";
    return this->GenerateInvalidId();
  }
}

}
}
}
