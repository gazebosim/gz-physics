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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include "JointFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(
    const std::size_t _id, const std::size_t _dof) const
{
  return this->joints.at(_id)->getPosition(_dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const std::size_t _id, const std::size_t _dof) const
{
  return this->joints.at(_id)->getVelocity(_dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(
    const std::size_t _id, const std::size_t _dof) const
{
  return this->joints.at(_id)->getAcceleration(_dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(
    const std::size_t _id, const std::size_t _dof) const
{
  return this->joints.at(_id)->getForce(_dof);
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const std::size_t _id) const
{
  return this->joints.at(_id)->getRelativeTransform();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(
    const std::size_t _id, const std::size_t _dof, const double _value)
{
  this->joints.at(_id)->setPosition(_dof, _value);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
    const std::size_t _id, const std::size_t _dof, const double _value)
{
  this->joints.at(_id)->setVelocity(_dof, _value);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(
    const std::size_t _id, const std::size_t _dof, const double _value)
{
  this->joints.at(_id)->setAcceleration(_dof, _value);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
    const std::size_t _id, const std::size_t _dof, const double _value)
{
  this->joints.at(_id)->setForce(_dof, _value);
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const std::size_t _id) const
{
  return this->joints.at(_id)->getNumDofs();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const std::size_t _id) const
{
  return this->joints.at(_id)->getTransformFromParentBodyNode();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const std::size_t _id) const
{
  return this->joints.at(_id)->getTransformFromChildBodyNode().inverse();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformFromParent(
    const std::size_t _id, const Pose3d &_pose)
{
  this->joints.at(_id)->setTransformFromParentBodyNode(_pose);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformToChild(
    const std::size_t _id, const Pose3d &_pose)
{
  this->joints.at(_id)->setTransformFromChildBodyNode(_pose.inverse());
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(
    const std::size_t _jointID) const
{
  dart::dynamics::WeldJoint *weld =
      dynamic_cast<dart::dynamics::WeldJoint*>(
        this->joints.at(_jointID).get());

  if (weld)
    return this->GenerateIdentity(_jointID);

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachFixedJoint(
    const std::size_t _childID,
    const BaseLink3dPtr &_parent)
{
  DartBodyNode * const bn = this->links.at(_childID);
  dart::dynamics::WeldJoint::Properties properties;

  DartBodyNode * const parentBn = _parent?
        this->links.at(_parent->EntityID()) : nullptr;

  return this->GenerateIdentity(
        this->AddJoint(bn->moveTo<dart::dynamics::WeldJoint>(
                         parentBn, properties)));
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFreeJoint(
    const std::size_t _jointID) const
{
  dart::dynamics::FreeJoint * const freeJoint =
      dynamic_cast<dart::dynamics::FreeJoint*>(
        this->joints.at(_jointID).get());

  if (freeJoint)
    return this->GenerateIdentity(_jointID);

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void JointFeatures::SetFreeJointRelativeTransform(
    const std::size_t _jointID, const Pose3d &_pose)
{
  static_cast<dart::dynamics::FreeJoint*>(
        this->joints.at(_jointID).get())->setRelativeTransform(_pose);
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToRevoluteJoint(
    const std::size_t _jointID) const
{
  dart::dynamics::RevoluteJoint *revolute =
      dynamic_cast<dart::dynamics::RevoluteJoint*>(
        this->joints.at(_jointID).get());

  if (revolute)
    return this->GenerateIdentity(_jointID);

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const std::size_t _jointID) const
{
  return static_cast<const dart::dynamics::RevoluteJoint*>(
        this->joints.at(_jointID).get())->getAxis();
}

/////////////////////////////////////////////////
void JointFeatures::SetRevoluteJointAxis(
    const std::size_t _jointID, const AngularVector3d &_axis)
{
  static_cast<dart::dynamics::RevoluteJoint*>(
        this->joints.at(_jointID).get())->setAxis(_axis);
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachRevoluteJoint(
    const std::size_t _childID,
    const BaseLink3dPtr &_parent,
    const AngularVector3d &_axis)
{
  DartBodyNode * const bn = this->links.at(_childID);
  dart::dynamics::RevoluteJoint::Properties properties;
  properties.mAxis = _axis;

  DartBodyNode * const parentBn = _parent?
        this->links.at(_parent->EntityID()) : nullptr;

  return this->GenerateIdentity(
        this->AddJoint(bn->moveTo<dart::dynamics::RevoluteJoint>(
                         parentBn, properties)));
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToPrismaticJoint(
    const std::size_t _jointID) const
{
  dart::dynamics::PrismaticJoint *prismatic =
      dynamic_cast<dart::dynamics::PrismaticJoint*>(
        this->joints.at(_jointID).get());

  if (prismatic)
    return this->GenerateIdentity(_jointID);

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d JointFeatures::GetPrismaticJointAxis(
    const std::size_t _jointID) const
{
  return static_cast<const dart::dynamics::PrismaticJoint*>(
        this->joints.at(_jointID).get())->getAxis();
}

/////////////////////////////////////////////////
void JointFeatures::SetPrismaticJointAxis(
    const std::size_t _jointID, const LinearVector3d &_axis)
{
  static_cast<dart::dynamics::PrismaticJoint*>(
        this->joints.at(_jointID).get())->setAxis(_axis);
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachPrismaticJoint(
    const std::size_t _childID,
    const BaseLink3dPtr &_parent,
    const LinearVector3d &_axis)
{
  DartBodyNode * const bn = this->links.at(_childID);
  dart::dynamics::PrismaticJoint::Properties properties;
  properties.mAxis = _axis;

  DartBodyNode * const parentBn = _parent?
        this->links.at(_parent->EntityID()) : nullptr;

  return this->GenerateIdentity(
        this->AddJoint(bn->moveTo<dart::dynamics::PrismaticJoint>(
                         parentBn, properties)));
}

}
}
}
