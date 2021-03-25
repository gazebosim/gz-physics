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

#include <memory>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
static math::Pose3d ResolveSdfPose(const ::sdf::SemanticPose &_semPose)
{
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  if (!errors.empty())
  {
    if (!_semPose.RelativeTo().empty())
    {
      ignerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto &err : errors)
      {
        ignerr << err.Message() << std::endl;
      }
      ignerr << "There is no optimal fallback since the relative_to attribute["
             << _semPose.RelativeTo() << "] of the pose is not empty. "
             << "Falling back to using the raw Pose.\n";
    }
    pose = _semPose.RawPose();
  }

  return pose;
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

  for (std::size_t i=0; i < _sdfWorld.ModelCount(); ++i)
  {
    const ::sdf::Model *model = _sdfWorld.ModelByIndex(i);

    if (!model)
      continue;

    this->ConstructSdfModel(worldID, *model);
  }

  return worldID;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
    const Identity &_worldID,
    const ::sdf::Model &_sdfModel)
{
  // check if parent is a world
  if (this->worlds.find(_worldID) == this->worlds.end())
  {
    ignerr << "Unable to construct model: " << _sdfModel.Name() << ". "
           << "Parent of model is not a world. " << std::endl;
    return this->GenerateInvalidId();
  }

  // Read sdf params
  const std::string name = _sdfModel.Name();
  const math::Pose3d pose = ResolveSdfPose(_sdfModel.SemanticPose());
  const bool isStatic = _sdfModel.Static();
  // Links within a model will collide unless these are chained with a joint
  // const bool selfCollide = _sdfModel.SelfCollide();

  const auto modelIdentity =
    this->AddModel(_worldID, {name, _worldID, isStatic, pose});

  // First, construct all links
  for (std::size_t i=0; i < _sdfModel.LinkCount(); ++i)
  {
    this->FindOrConstructLink(
      modelIdentity, _sdfModel, _sdfModel.LinkByIndex(i)->Name());
  }

  // Next, join all links that have joints
  for (std::size_t i=0; i < _sdfModel.JointCount(); ++i)
  {
    const ::sdf::Joint *sdfJoint = _sdfModel.JointByIndex(i);
    if (!sdfJoint)
    {
      ignerr << "The joint with index [" << i << "] in model ["
             << _sdfModel.Name() << "] is a nullptr. It will be skipped.\n";
      continue;
    }

    const std::size_t parent = this->FindOrConstructLink(
      modelIdentity, _sdfModel, sdfJoint->ParentLinkName());

    const std::size_t child = this->FindOrConstructLink(
      modelIdentity, _sdfModel, sdfJoint->ChildLinkName());

    this->ConstructSdfJoint(modelIdentity, *sdfJoint, parent, child);
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
  const math::Pose3d pose = ResolveSdfPose(_sdfLink.SemanticPose());
  const ignition::math::Inertiald inertial = _sdfLink.Inertial();
  double mass = inertial.MassMatrix().Mass();
  math::Pose3d inertialPose = inertial.Pose();
  inertialPose.Rot() *= inertial.MassMatrix().PrincipalAxesOffset();
  const auto diagonalMoments = inertial.MassMatrix().PrincipalMoments();

  // Get link properties
  btVector3 linkInertiaDiag =
    convertVec(ignition::math::eigen3::convert(diagonalMoments));

  const auto &modelInfo = this->models.at(_modelID);
  math::Pose3d base_pose = modelInfo->pose;
  const auto poseIsometry =
    ignition::math::eigen3::convert(base_pose * pose * inertialPose);
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // Fixed links have 0 mass and inertia
  if (this->models.at(_modelID)->fixed)
  {
    mass = 0;
    linkInertiaDiag = btVector3(0, 0, 0);
  }

  auto myMotionState = std::make_shared<btDefaultMotionState>(baseTransform);
  auto collisionShape = std::make_shared<btCompoundShape>();
  btRigidBody::btRigidBodyConstructionInfo
    rbInfo(mass, myMotionState.get(), collisionShape.get(), linkInertiaDiag);

  auto body = std::make_shared<btRigidBody>(rbInfo);
  body.get()->setActivationState(DISABLE_DEACTIVATION);

  const auto &world = this->worlds.at(modelInfo->world)->world;

  // Links collide with everything except elements sharing a joint
  world->addRigidBody(body.get());

  // Generate an identity for it
  const auto linkIdentity =
    this->AddLink(_modelID, {name, _modelID, pose, inertialPose,
    mass, linkInertiaDiag, myMotionState, collisionShape, body});

  // Create associated collisions to this model
  for (std::size_t i = 0; i < _sdfLink.CollisionCount(); ++i)
  {
    const auto collision = _sdfLink.CollisionByIndex(i);
    if (collision)
      this->ConstructSdfCollision(linkIdentity, *collision);
  }

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
  std::shared_ptr<btCollisionShape> shape;

  if (geom->BoxShape())
  {
    const auto box = geom->BoxShape();
    const auto size = math::eigen3::convert(box->Size());
    const auto halfExtents = convertVec(size)*0.5;
    shape = std::make_shared<btBoxShape>(halfExtents);
  }
  else if (geom->SphereShape())
  {
    const auto sphere = geom->SphereShape();
    const auto radius = sphere->Radius();
    shape = std::make_shared<btSphereShape>(radius);
  }
  else if (geom->CylinderShape())
  {
    const auto cylinder = geom->CylinderShape();
    const auto radius = cylinder->Radius();
    const auto halfLength = cylinder->Length()*0.5;
    shape =
      std::make_shared<btCylinderShapeZ>(btVector3(radius, radius, halfLength));
  }
  else if (geom->PlaneShape())
  {
    const auto plane = geom->PlaneShape();
    const auto normal = convertVec(math::eigen3::convert(plane->Normal()));
    shape = std::make_shared<btStaticPlaneShape>(normal, 0);
  }

  // TODO(lobotuerk/blast545) Add additional friction parameters for bullet
  // Currently supporting mu and mu2
  const auto &surfaceElement = _collision.Element()->GetElement("surface");

  // Get friction
  const auto &odeFriction = surfaceElement->GetElement("friction")
                              ->GetElement("ode");
  const auto mu = odeFriction->Get<btScalar>("mu");
  const auto mu2 = odeFriction->Get<btScalar>("mu2");

  // Restitution coefficient not tested
  // const auto restitution = surfaceElement->GetElement("bounce")
  //                             ->Get<btScalar>("restitution_coefficient");
  if (shape != nullptr)
  {
    const auto &linkInfo = this->links.at(_linkID);
    const auto &body = linkInfo->link;
    const auto &modelID = linkInfo->model;

    const math::Pose3d pose =
      linkInfo->inertialPose.Inverse() *
      ResolveSdfPose(_collision.SemanticPose());
    const Eigen::Isometry3d poseIsometry =
      ignition::math::eigen3::convert(pose);
    const Eigen::Vector3d poseTranslation = poseIsometry.translation();
    const auto poseLinear = poseIsometry.linear();
    btTransform baseTransform;
    baseTransform.setOrigin(convertVec(poseTranslation));
    baseTransform.setBasis(convertMat(poseLinear));

    // shape->setMargin(btScalar(0.0001));
    // body->setRollingFriction(0.25);
    body->setFriction(1);
    body->setAnisotropicFriction(btVector3(mu, mu2, 1),
    btCollisionObject::CF_ANISOTROPIC_FRICTION);

    dynamic_cast<btCompoundShape *>(body->getCollisionShape())
      ->addChildShape(baseTransform, shape.get());

    auto identity =
      this->AddCollision(
      _linkID, {_collision.Name(), shape, _linkID, modelID, pose, false});
    return identity;
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(
    const Identity &_modelID,
    const ::sdf::Joint &_sdfJoint)
{
  // Check supported Joints
  const ::sdf::JointType type = _sdfJoint.Type();
  if( type != ::sdf::JointType::REVOLUTE && type != ::sdf::JointType::FIXED )
  {
    ignerr << "Asked to construct a joint of sdf::JointType ["
           << static_cast<int>(type) << "], but that is not supported yet.\n";
    return this->GenerateInvalidId();
  }

  // Dummy world to reuse FindOrConstructLink code
  const ::sdf::Model dummyEmptyModel;

  // Get the parent and child ids
  const std::string parentLinkName = _sdfJoint.ParentLinkName();
  std::size_t parentId =
    this->FindOrConstructLink(_modelID, dummyEmptyModel, parentLinkName);

  const std::string childLinkName = _sdfJoint.ChildLinkName();
  std::size_t childId =
    this->FindOrConstructLink(_modelID, dummyEmptyModel, childLinkName);

  return this->ConstructSdfJoint(_modelID, _sdfJoint, parentId, childId);
}

Identity SDFFeatures::ConstructSdfJoint(
    const Identity &_modelID,
    const ::sdf::Joint &_sdfJoint,
    std::size_t parentId,
    std::size_t childId)
{
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
  // ignition::math::Vector3d axis = ignition::math::Vector3d::UnitZ;
  ignition::math::Vector3d axis;
  const ::sdf::JointType type = _sdfJoint.Type();
  if(type == ::sdf::JointType::FIXED )
  {
    axis = ignition::math::Vector3d::UnitZ;
  }
  else
  {
    // Resolve Axis XYZ. If it fails, use xyz function instead
    math::Vector3d resolvedAxis;
    ::sdf::Errors errors = _sdfJoint.Axis(0)->ResolveXyz(resolvedAxis);
    if (!errors.empty())
      resolvedAxis = _sdfJoint.Axis(0)->Xyz();
    axis =
      (this->links.at(childId)->pose *
       ResolveSdfPose(_sdfJoint.SemanticPose())).Rot()
       * resolvedAxis;
  }

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  math::Vector3d pivotParent, pivotChild, axisParent, axisChild;
  math::Pose3d pose;

  if (parentId != worldId)
  {
    pivotParent =
      (ResolveSdfPose(_sdfJoint.SemanticPose()) *
       this->links.at(childId)->pose).Pos();
    pose =
      this->links.at(parentId)->pose * this->links.at(parentId)->inertialPose;
    pivotParent -= pose.Pos();
    pivotParent = pose.Rot().RotateVectorReverse(pivotParent);
    axisParent = pose.Rot().RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
  }

  pivotChild =
    (ResolveSdfPose(_sdfJoint.SemanticPose()) *
     this->links.at(childId)->pose).Pos();
  pose = this->links.at(childId)->pose * this->links.at(childId)->inertialPose;
  pivotChild -= pose.Pos();
  pivotChild = pose.Rot().RotateVectorReverse(pivotChild);
  axisChild = pose.Rot().RotateVectorReverse(axis);
  axisChild = axisChild.Normalize();

  std::shared_ptr<btHingeAccumulatedAngleConstraint> joint;
  if (parentId != worldId)
  {
    joint = std::make_shared<btHingeAccumulatedAngleConstraint>(
      *this->links.at(childId)->link.get(),
      *this->links.at(parentId)->link.get(),
      convertVec(ignition::math::eigen3::convert(pivotChild)),
      convertVec(ignition::math::eigen3::convert(pivotParent)),
      convertVec(ignition::math::eigen3::convert(axisChild)),
      convertVec(ignition::math::eigen3::convert(axisParent)));
  }
  else
  {
    joint = std::make_shared<btHingeAccumulatedAngleConstraint>(
      *this->links.at(childId)->link.get(),
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
  const auto &world = this->worlds.at(modelInfo->world)->world.get();
  world->addConstraint(joint.get(), true);
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
    this->AddJoint({_sdfJoint.Name(), joint, childId, parentId,
    static_cast<int>(type), axis});
  return identity;
}

/////////////////////////////////////////////////
std::size_t SDFFeatures::FindOrConstructLink(
    const Identity &_modelID,
    const ::sdf::Model &_sdfModel,
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

  const ::sdf::Link * const sdfLink = _sdfModel.LinkByName(_sdfLinkName);
  if (!sdfLink)
  {
    ignerr << "Model [" << _sdfModel.Name() << "] does not contain a Link "
           << "with the name [" << _sdfLinkName << "].\n";
    return this->GenerateInvalidId().id;
  }

  return this->ConstructSdfLink(_modelID, *sdfLink);
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
