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

#include "SDFFeatures.hh"

#include <BulletCollision/CollisionShapes/btCapsuleShape.h>

#include <ignition/math/Helpers.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <memory>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <utility>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
static math::Pose3d ResolveSdfPose(const ::sdf::SemanticPose& _semPose) {
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  if (!errors.empty()) {
    if (!_semPose.RelativeTo().empty()) {
      ignerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto& err : errors) {
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
Identity SDFFeatures::ConstructSdfWorld(const Identity& _engine,
                                        const ::sdf::World& _sdfWorld) {
  auto worldIdentity = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  auto g = _sdfWorld.Gravity();
  this->world->btWorld->setGravity(btVector3(g[0], g[1], g[2]));

  for (std::size_t i = 0; i < _sdfWorld.ModelCount(); ++i) {
    this->ConstructSdfModel(worldIdentity, *_sdfWorld.ModelByIndex(i));
  }

  return worldIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModelImpl(const Identity& _parentID,
                                            const ::sdf::Model& _sdfModel) {
  // Read SDF params
  std::string name = _sdfModel.Name();
  bool isStatic = _sdfModel.Static();
  auto pose = ResolveSdfPose(_sdfModel.SemanticPose());

  // Links within a model will collide unless these are chained with a joint
  // const bool selfCollide = _sdfModel.SelfCollide();

  // Create the model
  auto model =
      std::make_unique<Model>(name, pose, isStatic, this->world->btWorld.get());

  // Add the new model to our Base object, which will
  // manage its lifecycle.
  auto modelIdentity = _parentID.id == this->worldId
                           ? this->AddModel(std::move(model))
                           : this->AddNestedModel(_parentID, std::move(model));

  // Construct all the model's nested models
  for (std::size_t i = 0; i < _sdfModel.ModelCount(); ++i) {
    this->ConstructSdfModelImpl(modelIdentity, *_sdfModel.ModelByIndex(i));
  }

  // Construct all the model's links
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i) {
    this->ConstructSdfLink(modelIdentity, *_sdfModel.LinkByIndex(i));
  }

  // Construct all the model's joints
  for (std::size_t i = 0; i < _sdfModel.JointCount(); ++i) {
    this->ConstructSdfJoint(modelIdentity, *_sdfModel.JointByIndex(i));
  }

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfNestedModel(const Identity& _parentID,
                                              const ::sdf::Model& _sdfModel) {
  return ConstructSdfModelImpl(_parentID, _sdfModel);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(const Identity& /*_worldID */,
                                        const ::sdf::Model& _sdfModel) {
  return ConstructSdfModelImpl(this->GenerateIdentity(this->worldId),
                               _sdfModel);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfLink(const Identity& _modelID,
                                       const ::sdf::Link& _sdfLink) {
  // Model
  auto model = std::get<Model*>(this->entities.at(_modelID));

  // Read SDF params
  auto name = _sdfLink.Name();
  auto pose = ResolveSdfPose(_sdfLink.SemanticPose());
  auto inertial = _sdfLink.Inertial();

  // Compute pose and inertial
  double mass = inertial.MassMatrix().Mass();
  auto inertialPose = inertial.Pose();
  inertialPose.Rot() *= inertial.MassMatrix().PrincipalAxesOffset();
  auto diagonalMoments = inertial.MassMatrix().PrincipalMoments();
  btVector3 linkInertiaDiag =
      convertVec(ignition::math::eigen3::convert(diagonalMoments));
  auto poseIsometry =
      ignition::math::eigen3::convert(model->sdfPose * pose * inertialPose);
  auto poseTranslation = poseIsometry.translation();
  auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // Fixed links have zero mass and inertia
  if (model->fixed) {
    mass = 0;
    linkInertiaDiag = btVector3(0, 0, 0);
  }

  // Create a Link object
  auto link = std::make_unique<Link>(name, pose, inertialPose, model->world,
                                     baseTransform, mass, linkInertiaDiag);

  // Add the new link to our Base object, which will manage its
  // lifecycle.
  auto linkIdentity = this->AddLink(_modelID, std::move(link));

  // Create associated collisions to this model
  for (std::size_t i = 0; i < _sdfLink.CollisionCount(); ++i) {
    const auto* collision = _sdfLink.CollisionByIndex(i);
    if (collision) {
      this->ConstructSdfCollision(linkIdentity, *collision);
    }
  }

  return linkIdentity;
}

Identity SDFFeatures::ConstructSdfCollision(
    const Identity& _linkID,
    const ::sdf::Collision& _collision) {
  // Link
  auto link = std::get<Link*>(this->entities.at(_linkID));

  const auto& name = _collision.Name();

  auto* geom = _collision.Geom();
  if (geom == nullptr) {
    ignerr << "Null geometry element for collision [" << _collision.Name()
           << "]\n";
    return this->GenerateInvalidId();
  }

  std::unique_ptr<btCollisionShape> shape;

  if (geom->BoxShape()) {
    // Box
    auto box = geom->BoxShape();
    auto size = math::eigen3::convert(box->Size());
    auto halfExtents = convertVec(size) * 0.5;
    shape = std::make_unique<btBoxShape>(halfExtents);
  } else if (geom->SphereShape()) {
    // Sphere
    auto sphere = geom->SphereShape();
    auto radius = sphere->Radius();
    shape = std::make_unique<btSphereShape>(radius);
  } else if (geom->CylinderShape()) {
    // Cylinder
    auto cylinder = geom->CylinderShape();
    auto radius = cylinder->Radius();
    auto halfLength = cylinder->Length() * 0.5;
    shape = std::make_unique<btCylinderShapeZ>(
        btVector3(radius, radius, halfLength));
  } else if (geom->PlaneShape()) {
    // Plane
    auto plane = geom->PlaneShape();
    auto normal = convertVec(math::eigen3::convert(plane->Normal()));
    shape = std::make_unique<btStaticPlaneShape>(normal, 0);
  } else if (geom->CapsuleShape()) {
    // Capsule
    auto capsule = geom->CapsuleShape();
    auto radius = capsule->Radius();
    auto length = capsule->Length();
    // TODO(joxoby): What's the definition of Length() ?
    shape = std::make_unique<btCapsuleShape>(radius, length);
  } else if (geom->EllipsoidShape()) {
    // Ellipsoid
    auto ellipsoid = geom->EllipsoidShape();
    auto radii = ellipsoid->Radii();
    // TODO(joxoby): Replace with a real ellipsoid
    shape = std::make_unique<btSphereShape>(radii.Max());
  }

  // TODO(lobotuerk/blast545) Add additional friction parameters for bullet
  // Currently supporting mu and mu2
  auto surfaceElement = _collision.Element()->GetElement("surface");

  // Get friction
  auto odeFriction = surfaceElement->GetElement("friction")->GetElement("ode");
  auto mu = odeFriction->Get<double>("mu");
  auto mu2 = odeFriction->Get<double>("mu2");

  // Restitution coefficient not tested
  // const auto restitution = surfaceElement->GetElement("bounce")
  //                             ->Get<btScalar>("restitution_coefficient");
  if (shape == nullptr) {
    return this->GenerateInvalidId();
  }

  auto body = link->body.get();

  math::Pose3d pose = link->sdfInertialPose.Inverse() *
                      ResolveSdfPose(_collision.SemanticPose());
  Eigen::Isometry3d poseIsometry = ignition::math::eigen3::convert(pose);
  Eigen::Vector3d poseTranslation = poseIsometry.translation();
  auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  bool isMesh = false;
  std::unique_ptr<btTriangleMesh> nullMesh;

  auto collision =
      std::make_unique<Collision>(name, body, isMesh, std::move(shape),
                                  std::move(nullMesh), baseTransform, mu, mu2);

  // Add the new collision to our Base object, which will
  // manage its lifecycle.
  auto identity = this->AddCollision(_linkID, std::move(collision));

  return identity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(const Identity& _modelID,
                                        const ::sdf::Joint& _sdfJoint) {
  // Check supported Joints
  auto type = _sdfJoint.Type();
  if (type != ::sdf::JointType::REVOLUTE && type != ::sdf::JointType::FIXED) {
    ignerr << "Asked to construct a joint of sdf::JointType ["
           << static_cast<int>(type) << "], but that is not supported yet.\n";
    return this->GenerateInvalidId();
  }

  const auto& jointName = _sdfJoint.Name();

  auto model = std::get<Model*>(this->entities.at(_modelID));

  // Function to find a model's link given the fullname
  auto LinkWithName = [model](std::string _name) -> Link* {
    auto m = model;
    std::string delimiter = "::";
    std::size_t pos = 0;
    // If the name contains "::", we need to traverse the nested models
    // to find the link
    while ((pos = _name.find(delimiter)) != std::string::npos) {
      auto nestedModelName = _name.substr(0, pos);
      auto it = std::find_if(m->models.cbegin(), m->models.cend(),
                             [&nestedModelName](auto& nestedModel) {
                               return nestedModel->name == nestedModelName;
                             });
      if (it == m->models.cend()) {
        // Error, nested model not found
        return nullptr;
      }
      m = it->get();
      _name.erase(0, pos + delimiter.length());
    }
    // We reached the nested model containing the link, now find the link
    auto it =
        std::find_if(m->links.cbegin(), m->links.cend(),
                     [&_name](auto& link) { return link->name == _name; });
    return it != m->links.cend() ? it->get() : nullptr;
  };

  // Get the parent link
  std::string parentLinkName;
  auto errors = _sdfJoint.ResolveParentLink(parentLinkName);
  if (!errors.empty()) {
    parentLinkName = _sdfJoint.ParentLinkName();
  }

  Link* parentLink = nullptr;
  if (parentLinkName != "world") {
    parentLink = LinkWithName(parentLinkName);
    if (parentLink == nullptr) {
      ignerr << "Invalid parent link with name [" << parentLinkName
             << "] for joint [" << jointName << "]\n";
      return this->GenerateInvalidId();
    }
  }

  // Get the child link
  std::string childLinkName;
  errors = _sdfJoint.ResolveChildLink(childLinkName);
  if (!errors.empty()) {
    childLinkName = _sdfJoint.ChildLinkName();
  }

  auto childLink = LinkWithName(childLinkName);
  if (childLink == nullptr) {
    ignerr << "Invalid child link with name [" << childLinkName
           << "] for joint [" << jointName << "]\n";
    return this->GenerateInvalidId();
  }

  // Get axis unit vector (expressed in world frame).
  // If fixed joint, use UnitZ, if revolute use the Axis given by the joint.
  ignition::math::Vector3d axis;
  if (type == ::sdf::JointType::FIXED) {
    axis = ignition::math::Vector3d::UnitZ;
  } else {
    // Resolve Axis XYZ. If it fails, use xyz function instead
    math::Vector3d resolvedAxis;
    if (_sdfJoint.Axis(0)) {
      errors = _sdfJoint.Axis(0)->ResolveXyz(resolvedAxis);
      if (!errors.empty())
        resolvedAxis = _sdfJoint.Axis(0)->Xyz();
      axis = (childLink->sdfPose * ResolveSdfPose(_sdfJoint.SemanticPose()))
                 .Rot() *
             resolvedAxis;
    }
  }

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  math::Vector3d pivotParent, pivotChild, axisParent, axisChild;
  math::Pose3d pose;

  if (parentLink != nullptr) {
    pivotParent =
        (ResolveSdfPose(_sdfJoint.SemanticPose()) * childLink->sdfPose).Pos();
    pose = parentLink->sdfPose * parentLink->sdfInertialPose;
    pivotParent -= pose.Pos();
    pivotParent = pose.Rot().RotateVectorReverse(pivotParent);
    axisParent = pose.Rot().RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
  }

  pivotChild =
      (ResolveSdfPose(_sdfJoint.SemanticPose()) * childLink->sdfPose).Pos();
  pose = childLink->sdfPose * childLink->sdfInertialPose;
  pivotChild -= pose.Pos();
  pivotChild = pose.Rot().RotateVectorReverse(pivotChild);
  axisChild = pose.Rot().RotateVectorReverse(axis);
  axisChild = axisChild.Normalize();

  auto parentLinkBody =
      parentLink != nullptr ? parentLink->body.get() : nullptr;

  auto joint = std::make_unique<Joint>(
      jointName, this->world->btWorld.get(), childLink->body.get(),
      parentLinkBody, convertVec(ignition::math::eigen3::convert(pivotChild)),
      convertVec(ignition::math::eigen3::convert(pivotParent)),
      convertVec(ignition::math::eigen3::convert(axisChild)),
      convertVec(ignition::math::eigen3::convert(axisParent)), type);

  // Add the new joint to our Base object, which will
  // manage its lifecycle.
  return this->AddJoint(_modelID, std::move(joint));
}
}  // namespace bullet
}  // namespace physics
}  // namespace ignition
