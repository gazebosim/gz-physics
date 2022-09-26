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
  if (identifier)
  {
    const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
    return model->body->getJointPosMultiDof(identifier->indexInBtModel)[_dof];
  }

  // The base joint never really has a position. It is either a Free Joint or
  // a Fixed Joint, but it doesn't track a "joint position" for Free Joint.
  return 0.0;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, const std::size_t _dof) const
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (identifier)
  {
    const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
    return model->body->getJointVelMultiDof(identifier->indexInBtModel)[_dof];
  }

  if (std::get_if<RootJoint>(&joint->identifier))
  {
    const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);

    if (_dof < 3)
      return model->body->getBaseVel()[_dof];
    else if (_dof < 6)
      return model->body->getBaseOmega()[_dof];
  }

  return gz::math::NAN_D;
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
  if (identifier)
  {
    const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
    return model->body->getJointTorqueMultiDof(
          identifier->indexInBtModel)[_dof];
  }

  return gz::math::NAN_D;
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

  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  model->body->getJointPosMultiDof(identifier->indexInBtModel)[_dof] = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
  const Identity &_id, const std::size_t _dof, const double _value)
{
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return;

  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  model->body->getJointVelMultiDof(identifier->indexInBtModel)[_dof] = _value;
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

  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint velocity value [" << _value
           << "] commanded on joint [" << joint->name << " DOF " << _dof
           << "]. The command will be ignored\n";
    return;
  }

  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return;

  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  model->body->getJointTorqueMultiDof(
    identifier->indexInBtModel)[_dof] = _value;
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

  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  return model->body->getLink(identifier->indexInBtModel).m_dofCount;
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
  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  const auto &link = model->body->getLink(identifier.indexInBtModel);
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
  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  const auto &link = model->body->getLink(identifier.indexInBtModel);
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
    if (type == btMultibodyLink::eFixed)
      return _jointID;
    else
      return this->GenerateInvalidId();
  }

  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  if (type == model->body->getLink(identifier->indexInBtModel).m_jointType)
    return _jointID;

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocityCommand(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  // Take extra care that the value is finite. A nan can cause the Bullet
  // constraint solver to fail, which will in turn either cause a crash or
  // collisions to fail
  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint velocity value [" << _value
           << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
           << "]. The command will be ignored\n";
    return;
  }

  jointInfo->motor->setVelocityTarget(_value);
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachFixedJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &_name)
{
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_childID);
  auto modelInfo = this->ReferenceInterface<ModelInfo>(linkInfo->model);
  auto parentLinkInfo = this->ReferenceInterface<LinkInfo>(_parent->FullIdentity());
  auto parentModelInfo = this->ReferenceInterface<ModelInfo>(parentLinkInfo->model);
  auto *world = this->ReferenceInterface<WorldInfo>(modelInfo->world);

  auto jointID = this->AddJoint(
    JointInfo{
      _name + "_" + parentLinkInfo->name + "_" + linkInfo->name,
      InternalJoint{0},
      _parent->FullIdentity().id,
      _childID,
      Eigen::Isometry3d(),
      Eigen::Isometry3d(),
      linkInfo->model
    });

  auto jointInfo = this->ReferenceInterface<JointInfo>(jointID);

  jointInfo->fixedContraint = new btMultiBodyFixedConstraint(
    parentModelInfo->body.get(), -1,
    modelInfo->body.get(), -1,
    btVector3(), btVector3(),
    btMatrix3x3::getIdentity(),
    btMatrix3x3::getIdentity());
  jointInfo->fixedContraint->setMaxAppliedImpulse(0.2);

  if (world)
  {
    if (world->world)
    {
      world->world->addMultiBodyConstraint(jointInfo->fixedContraint);
      return this->GenerateIdentity(jointID, this->joints.at(jointID));
    }
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void JointFeatures::DetachJoint(const Identity &_jointId)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_jointId);
  if (jointInfo->fixedContraint)
  {
    auto modelInfo = this->ReferenceInterface<ModelInfo>(jointInfo->model);
    if (modelInfo)
    {
      auto *world = this->ReferenceInterface<WorldInfo>(modelInfo->world);
      world->world->removeMultiBodyConstraint(jointInfo->fixedContraint);
      delete jointInfo->fixedContraint;
      jointInfo->fixedContraint = 0;
    }
  }
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformFromParent(
    const Identity &_id, const Pose3d &_pose)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  if (jointInfo->fixedContraint)
  {
      jointInfo->fixedContraint->setPivotInA(
        btVector3(
          _pose.translation()[0],
          _pose.translation()[1],
          _pose.translation()[2]));
  }
}
}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
