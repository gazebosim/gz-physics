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
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
  {
    // Currently we only support fixed constraints for model-to-model joints,
    // and fixed constraints have zero degrees of freedom.
    return 0;
  }

  const auto *model = this->ReferenceInterface<ModelInfo>(identifier->model);
  return model->body->getLink(identifier->indexInModel).m_dofCount;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &_id) const
{
  return this->ReferenceInterface<JointInfo>(_id)->tf_from_parent;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const
{
  return this->ReferenceInterface<JointInfo>(_id)->tf_to_child;
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(const Identity &_jointID) const
{
  return this->CastToJointType(_jointID, btMultibodyLink::eFixed);
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToRevoluteJoint(const Identity &_jointID) const
{
  return this->CastToJointType(_jointID, btMultibodyLink::eRevolute);
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const Identity &_jointID) const
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_jointID);

  // In order for this function to be called, gz-physics should have already
  // validated that it is a revolute joint inside of a model.
  const auto &identifier = std::get<InternalJoint>(joint->identifier);
  const auto *model = this->ReferenceInterface<ModelInfo>(identifier.model);
  const auto &link = model->body->getLink(identifier.indexInModel);
  assert(link.m_jointType == btMultibodyLink::eRevolute);

  // According to the documentation in btMultibodyLink.h, m_axesTop[0] is the
  // joint axis for revolute joints.
  return convert(link.getAxisTop(0));
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToPrismaticJoint(const Identity &_jointID) const
{
  return this->CastToJointType(_jointID, btMultibodyLink::ePrismatic);
}

/////////////////////////////////////////////////
Eigen::Vector3d JointFeatures::GetPrismaticJointAxis(
    const Identity &_jointID) const
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_jointID);

  // In order for this function to be called, gz-physics should have already
  // validated that it is a prismatic joint inside of a model.
  const auto &identifier = std::get<InternalJoint>(joint->identifier);
  const auto *model = this->ReferenceInterface<ModelInfo>(identifier.model);
  const auto &link = model->body->getLink(identifier.indexInModel);
  assert(link.m_jointType == btMultibodyLink::ePrismatic);

  // According to the documentation in btMultibodyLink.h, m_axesBottom[0] is the
  // joint axis for prismatic joints.
  return convert(link.getAxisBottom(0));
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToJointType(
  const Identity &_jointID,
  const btMultibodyLink::eFeatherstoneJointType type) const
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_jointID);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
  {
    // Since we only support fixed constraints between models, we assume this is
    // a fixed joint.
    return _jointID;
  }

  const auto *model = this->ReferenceInterface<ModelInfo>(identifier->model);
  if (type == model->body->getLink(identifier->indexInModel).m_jointType)
    return _jointID;

  return this->GenerateInvalidId();
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
