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
    const Identity &_id, const std::size_t _dof) const
{
  return this->ReferenceInterface<JointInfo>(_id)->joint->getPosition(_dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, const std::size_t _dof) const
{
  return this->ReferenceInterface<JointInfo>(_id)->joint->getVelocity(_dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(
    const Identity &_id, const std::size_t _dof) const
{
  return this->ReferenceInterface<JointInfo>(_id)->joint->getAcceleration(_dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(
    const Identity &_id, const std::size_t _dof) const
{
  return this->ReferenceInterface<JointInfo>(_id)->joint->getForce(_dof);
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &_id) const
{
  return this->ReferenceInterface<JointInfo>(_id)
      ->joint->getRelativeTransform();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  this->ReferenceInterface<JointInfo>(_id)->joint->setPosition(_dof, _value);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  this->ReferenceInterface<JointInfo>(_id)->joint->setVelocity(_dof, _value);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  this->ReferenceInterface<JointInfo>(_id)->joint->setAcceleration(_dof,
                                                                   _value);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  this->ReferenceInterface<JointInfo>(_id)->joint->setForce(_dof, _value);
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const
{
  return this->ReferenceInterface<JointInfo>(_id)->joint->getNumDofs();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &_id) const
{
  return this->ReferenceInterface<JointInfo>(_id)
      ->joint->getTransformFromParentBodyNode();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const
{
  return this->ReferenceInterface<JointInfo>(_id)
      ->joint->getTransformFromChildBodyNode().inverse();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformFromParent(
    const Identity &_id, const Pose3d &_pose)
{
  this->ReferenceInterface<JointInfo>(_id)
      ->joint->setTransformFromParentBodyNode(_pose);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformToChild(
    const Identity &_id, const Pose3d &_pose)
{
  this->ReferenceInterface<JointInfo>(_id)
      ->joint->setTransformFromChildBodyNode(_pose.inverse());
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(
    const Identity &_jointID) const
{
  dart::dynamics::WeldJoint *const weld =
      dynamic_cast<dart::dynamics::WeldJoint *>(
          this->ReferenceInterface<JointInfo>(_jointID)->joint.get());

  if (weld)
    return this->GenerateIdentity(_jointID, this->Reference(_jointID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachFixedJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &_name)
{
  DartBodyNode *const bn =
      this->ReferenceInterface<LinkInfo>(_childID)->link.get();
  dart::dynamics::WeldJoint::Properties properties;
  properties.mName = _name;

  auto *const parentBn = _parent ? this->ReferenceInterface<LinkInfo>(
      _parent->FullIdentity())->link.get() : nullptr;

  const std::size_t jointID = this->AddJoint(
      bn->moveTo<dart::dynamics::WeldJoint>(parentBn, properties));
  return this->GenerateIdentity(jointID, this->joints.at(jointID));
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFreeJoint(
    const Identity &_jointID) const
{
  auto *const freeJoint =
      dynamic_cast<dart::dynamics::FreeJoint *>(
          this->ReferenceInterface<JointInfo>(_jointID)->joint.get());

  if (freeJoint)
    return this->GenerateIdentity(_jointID, this->Reference(_jointID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void JointFeatures::SetFreeJointRelativeTransform(
    const Identity &_jointID, const Pose3d &_pose)
{
  static_cast<dart::dynamics::FreeJoint *>(
      this->ReferenceInterface<JointInfo>(_jointID)->joint.get())
      ->setRelativeTransform(_pose);
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToRevoluteJoint(
    const Identity &_jointID) const
{
  dart::dynamics::RevoluteJoint *const revolute =
      dynamic_cast<dart::dynamics::RevoluteJoint *>(
          this->ReferenceInterface<JointInfo>(_jointID)->joint.get());

  if (revolute)
    return this->GenerateIdentity(_jointID, this->Reference(_jointID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const Identity &_jointID) const
{
  return static_cast<const dart::dynamics::RevoluteJoint*>(
        this->ReferenceInterface<JointInfo>(_jointID)->joint.get())->getAxis();
}

/////////////////////////////////////////////////
void JointFeatures::SetRevoluteJointAxis(
    const Identity &_jointID, const AngularVector3d &_axis)
{
  static_cast<dart::dynamics::RevoluteJoint *>(
      this->ReferenceInterface<JointInfo>(_jointID)->joint.get())
      ->setAxis(_axis);
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachRevoluteJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &_name,
    const AngularVector3d &_axis)
{
  DartBodyNode *const bn =
      this->ReferenceInterface<LinkInfo>(_childID)->link.get();
  dart::dynamics::RevoluteJoint::Properties properties;
  properties.mName = _name;
  properties.mAxis = _axis;

  auto *const parentBn = _parent ? this->ReferenceInterface<LinkInfo>(
      _parent->FullIdentity())->link.get() : nullptr;

  const std::size_t jointID = this->AddJoint(
      bn->moveTo<dart::dynamics::RevoluteJoint>(parentBn, properties));
  return this->GenerateIdentity(jointID, this->joints.at(jointID));
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToPrismaticJoint(
    const Identity &_jointID) const
{
  dart::dynamics::PrismaticJoint *prismatic =
      dynamic_cast<dart::dynamics::PrismaticJoint*>(
        this->ReferenceInterface<JointInfo>(_jointID)->joint.get());

  if (prismatic)
    return this->GenerateIdentity(_jointID, this->Reference(_jointID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d JointFeatures::GetPrismaticJointAxis(
    const Identity &_jointID) const
{
  return static_cast<const dart::dynamics::PrismaticJoint*>(
        this->ReferenceInterface<JointInfo>(_jointID)->joint.get())->getAxis();
}

/////////////////////////////////////////////////
void JointFeatures::SetPrismaticJointAxis(
    const Identity &_jointID, const LinearVector3d &_axis)
{
  static_cast<dart::dynamics::PrismaticJoint *>(
      this->ReferenceInterface<JointInfo>(_jointID)->joint.get())
      ->setAxis(_axis);
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachPrismaticJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &_name,
    const LinearVector3d &_axis)
{
  DartBodyNode *const bn =
      this->ReferenceInterface<LinkInfo>(_childID)->link.get();
  dart::dynamics::PrismaticJoint::Properties properties;
  properties.mName = _name;
  properties.mAxis = _axis;

  auto *const parentBn = _parent ? this->ReferenceInterface<LinkInfo>(
      _parent->FullIdentity())->link.get() : nullptr;

  const std::size_t jointID = this->AddJoint(
      bn->moveTo<dart::dynamics::PrismaticJoint>(parentBn, properties));
  return this->GenerateIdentity(jointID, this->joints.at(jointID));
}

}
}
}
