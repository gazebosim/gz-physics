/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "JointFeatures.hh"

#include <algorithm>
#include <sdf/Joint.hh>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(
    const Identity &_id, const std::size_t _dof) const
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return gz::math::NAN_D;

  const auto *model = this->ReferenceInterface<ModelInfo>(identifier->model);
  return *model->body->getJointPosMultiDof(_dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, const std::size_t _dof) const
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return gz::math::NAN_D;

  const auto *model = this->ReferenceInterface<ModelInfo>(identifier->model);
  return *model->body->getJointVelMultiDof(_dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(
    const Identity &/*_id*/, const std::size_t /*_dof*/) const
{
  return gz::math::NAN_D;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(
    const Identity &_id, const std::size_t _dof) const
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return gz::math::NAN_D;

  const auto *model = this->ReferenceInterface<ModelInfo>(identifier->model);
  return *model->body->getJointTorqueMultiDof(_dof);
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &_id) const
{
  (void) _id;
  gzwarn << "Dummy function GetJointTransform\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(
  const Identity &_id, const std::size_t _dof, const double _value)
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return;

  const auto *model = this->ReferenceInterface<ModelInfo>(identifier->model);
  model->body->getJointPosMultiDof(identifier->indexInModel)[_dof] = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
  const Identity &_id, const std::size_t _dof, const double _value)
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return;

  const auto *model = this->ReferenceInterface<ModelInfo>(identifier->model);
  model->body->getJointVelMultiDof(identifier->indexInModel)[_dof] = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(
  const Identity &/*_id*/, const std::size_t /*_dof*/, const double /*_value*/)
{
  // Do nothing
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return;

  const auto *model = this->ReferenceInterface<ModelInfo>(identifier->model);
  model->body->getJointVelMultiDof(identifier->indexInModel)[_dof] = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocityCommand(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  // Only support available for single DoF joints
  (void) _dof;
  const auto &jointInfo = this->joints.at(_id.id);

  // Take extra care that the value is finite
  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint velocity value [" << _value << "] set on joint ["
           << jointInfo->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }

  // Check the type of joint and act accordignly
  switch(jointInfo->constraintType)
  {
  case static_cast<int>(::sdf::JointType::REVOLUTE) :
  {
    const auto &link = this->links.at(jointInfo->childLinkId)->link;
    btTransform trans;
    link->getMotionState()->getWorldTransform(trans);
    btVector3 motion = quatRotate(trans.getRotation(),
      convertVec(gz::math::eigen3::convert(jointInfo->axis)));
    btVector3 angular_vel = motion * _value;
    link->setAngularVelocity(angular_vel);
  }
  break;
  default:
    gzwarn << "Not a valid setJointVelocityCommand type: "
            << jointInfo->constraintType << "\n";
    break;
  }
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const
{
  // Degrees of freedom may need to be saved in the JointInfo struct
  // Currently supporting 1DoF revolute joints and fixed joints
  const JointInfoPtr &jointInfo = this->joints.at(_id.id);
  if (jointInfo->constraintType == static_cast<int>(::sdf::JointType::FIXED))
  {
    return 0;
  }
  return 1;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &_id) const
{
  (void) _id;
  gzwarn << "Dummy get joint transform from parent\n";
  return Pose3d();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const
{
  (void) _id;
  gzwarn << "Dummy get joint transform to child\n";
  return Pose3d();
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(
  const Identity &_jointID) const
{
  if (this->joints.find(_jointID.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_jointID.id);
    if (jointInfo->constraintType == static_cast<int>(::sdf::JointType::FIXED))
    {
      return this->GenerateIdentity(_jointID, this->Reference(_jointID));
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToRevoluteJoint(
    const Identity &_jointID) const
{
  if (this->joints.find(_jointID.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_jointID.id);
    if (
      jointInfo->constraintType == static_cast<int>(::sdf::JointType::REVOLUTE))
    {
      return this->GenerateIdentity(_jointID, this->Reference(_jointID));
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const Identity &_jointID) const
{
  if (this->joints.find(_jointID.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_jointID.id);
    btHingeAccumulatedAngleConstraint* hinge =
      dynamic_cast<btHingeAccumulatedAngleConstraint*>(jointInfo->joint.get());
    if (hinge)
    {
      btVector3 vec =
        hinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
        hinge->getFrameOffsetA().getBasis().getColumn(2);
      // math::Vector3 globalAxis(vec[0], vec[1], vec[2]);
      return AngularVector3d(vec[0], vec[1], vec[2]);
    }
  }
  gzerr << "Error getting revolute Joint axis: " << _jointID.id << " \n";
  return AngularVector3d();
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
