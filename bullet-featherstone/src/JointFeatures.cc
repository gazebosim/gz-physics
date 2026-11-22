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
#include <cmath>
#include <cstddef>
#include <memory>
#include <unordered_map>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <sdf/Joint.hh>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
void recreateJointLimitConstraint(JointInfo *_jointInfo, ModelInfo *_modelInfo,
    WorldInfo *_worldInfo)
{
  GZ_PROFILE("bullet_featherstone::recreateJointLimitConstraint");
  const auto *identifier = std::get_if<InternalJoint>(&_jointInfo->identifier);
  if (!identifier)
    return;

  if (_jointInfo->jointLimits)
  {
    _worldInfo->world->removeMultiBodyConstraint(_jointInfo->jointLimits.get());
    _jointInfo->jointLimits.reset();
  }

  _jointInfo->jointLimits =
    std::make_shared<btMultiBodyJointLimitConstraint>(
      _modelInfo->body.get(), identifier->indexInBtModel,
      static_cast<btScalar>(_jointInfo->axisLower),
      static_cast<btScalar>(_jointInfo->axisUpper));

  _worldInfo->world->addMultiBodyConstraint(_jointInfo->jointLimits.get());
}

/////////////////////////////////////////////////
void updateColliderFlagsRecursive(std::size_t _linkID,
  const std::unordered_map<std::size_t, std::shared_ptr<JointInfo>> &_joints,
  const std::unordered_map<std::size_t, std::shared_ptr<LinkInfo>> &_links,
  std::function<void(LinkInfo *)> _updateColliderCb)
{
  GZ_PROFILE("bullet_featherstone::updateColliderFlagsRecursive");
  btMultiBodyFixedConstraint *fixedConstraint = nullptr;
  std::size_t childLinkID = 0u;
  for (const auto &joint : _joints)
  {
    if (!joint.second->fixedConstraint)
      continue;
    if (!joint.second->parentLinkID.has_value() ||
        joint.second->parentLinkID.value() != _linkID)
      continue;

    fixedConstraint =  joint.second->fixedConstraint.get();
    childLinkID = std::size_t(joint.second->childLinkID);
  }

  if (!fixedConstraint)
    return;

  auto childInfo = _links.at(childLinkID);
  _updateColliderCb(childInfo.get());

  updateColliderFlagsRecursive(childLinkID, _joints, _links, _updateColliderCb);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(
    const Identity &_id, const std::size_t _dof) const
{
  GZ_PROFILE("JointFeatures::GetJointPosition");
  if (!this->ValidateDofParam(_id, _dof))
    return gz::math::NAN_D;

  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  return model->body->GetJointPosForDof(
      std::get<InternalJoint>(joint->identifier).indexInBtModel, _dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, const std::size_t _dof) const
{
  GZ_PROFILE("JointFeatures::GetJointVelocity");
  if (!this->ValidateDofParam(_id, _dof))
    return gz::math::NAN_D;

  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  return model->body->getJointVelMultiDof(
      std::get<InternalJoint>(joint->identifier).indexInBtModel)[_dof];
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
  GZ_PROFILE("JointFeatures::GetJointForce");
  if (!this->ValidateDofParam(_id, _dof))
    return gz::math::NAN_D;

  double results = 0.0;
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  const int indexInBtModel =
      std::get<InternalJoint>(joint->identifier).indexInBtModel;
  auto feedback = model->body->getLink(indexInBtModel).m_jointFeedback;
  const auto &link = model->body->getLink(indexInBtModel);
  if (link.m_jointType == btMultibodyLink::eRevolute)
  {
    // According to the documentation in btMultibodyLink.h,
    // m_axesTop[0] is the joint axis for revolute joints.
    Eigen::Vector3d axis = convert(link.getAxisTop(0));
    math::Vector3d axis_converted(axis[0], axis[1], axis[2]);
    btVector3 angular = feedback->m_reactionForces.getAngular();
    math::Vector3d angularTorque(
      angular.getX(),
      angular.getY(),
      angular.getZ());
    results += axis_converted.Dot(angularTorque);
    #if BT_BULLET_VERSION < 326
      // not always true
      return results / 2.0;
    #else
      return results;
    #endif
  }
  else if (link.m_jointType == btMultibodyLink::ePrismatic)
  {
    auto axis = convert(link.getAxisBottom(0));
    math::Vector3d axis_converted(axis[0], axis[1], axis[2]);
    btVector3 linear = feedback->m_reactionForces.getLinear();
    math::Vector3d linearForce(
      linear.getX(),
      linear.getY(),
      linear.getZ());
    results += axis_converted.Dot(linearForce);
    #if BT_BULLET_VERSION < 326
      // Not always true see for reference:
      // https://github.com/bulletphysics/bullet3/discussions/3713
      // https://github.com/gazebosim/gz-physics/issues/565
      return results / 2.0;
    #else
      return results;
    #endif
  }
  return results;
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
  GZ_PROFILE("JointFeatures::SetJointPosition");
  if (!this->ValidateDofParam(_id, _dof))
    return;

  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  model->body->SetJointPosForDof(
      std::get<InternalJoint>(joint->identifier).indexInBtModel, _dof, _value);
  model->body->wakeUp();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
  const Identity &_id, const std::size_t _dof, const double _value)
{
  GZ_PROFILE("JointFeatures::SetJointVelocity");
  if (!this->ValidateDofParam(_id, _dof))
    return;

  auto *joint = this->ReferenceInterface<JointInfo>(_id);

  // A nan or inf joint velocity corrupts the state of the whole multibody.
  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint velocity value [" << _value
          << "] set on joint [" << joint->name << " DOF " << _dof
          << "]. The value will be ignored\n";
    return;
  }

  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  model->body->getJointVelMultiDof(
      std::get<InternalJoint>(joint->identifier).indexInBtModel)[_dof] =
      static_cast<btScalar>(_value);
  // Bullet does not integrate the position of a kinematic child link, so
  // WorldForwardStep does it with this velocity. Unlike a velocity command,
  // the velocity persists until it is set again. Only the first dof of a
  // kinematic child joint can currently be driven. The requested kinematic
  // state is used rather than the collider flags so that a velocity set while
  // the link is welded to a static link takes effect once it is detached.
  if (_dof == 0 &&
      this->ReferenceInterface<LinkInfo>(joint->childLinkID)->isKinematic)
  {
    joint->kinematicJointVel = _value;
    joint->kinematicJointVelCmd = std::nullopt;
  }
  model->body->wakeUp();
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
  GZ_PROFILE("JointFeatures::SetJointForce");
  if (!this->ValidateDofParam(_id, _dof))
    return;

  auto *joint = this->ReferenceInterface<JointInfo>(_id);

  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint force value [" << _value
           << "] commanded on joint [" << joint->name << " DOF " << _dof
           << "]. The command will be ignored\n";
    return;
  }

  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  auto *world = this->ReferenceInterface<WorldInfo>(model->world);

  // Disable velocity control on this DOF by removing its joint motor constraint
  auto motorIt = joint->motors.find(_dof);
  if (motorIt != joint->motors.end())
  {
    if (motorIt->second)
      world->world->removeMultiBodyConstraint(motorIt->second.get());
    joint->motors.erase(motorIt);
  }
  if (_dof == 0 && joint->motor)
  {
    world->world->removeMultiBodyConstraint(joint->motor.get());
    joint->motor.reset();
  }

  // clamp the values
  double force = std::clamp(_value,
      joint->GetMinEffort(_dof), joint->GetMaxEffort(_dof));

  model->body->getJointTorqueMultiDof(
      std::get<InternalJoint>(joint->identifier).indexInBtModel)[_dof] =
      static_cast<btScalar>(force);
  model->body->wakeUp();
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
  return static_cast<std::size_t>(
      model->body->getLink(identifier->indexInBtModel).m_dofCount);
}

/////////////////////////////////////////////////
bool JointFeatures::ValidateDofParam(
    const Identity &_id, std::size_t _dof) const
{
  // Only internal joints have degrees of freedom, so a valid dof also means
  // that the identifier of the joint is an InternalJoint.
  const std::size_t dofs = this->GetJointDegreesOfFreedom(_id);
  if (_dof >= dofs)
  {
    gzerr << "Invalid DOF [" << _dof << "] on joint ["
          << this->ReferenceInterface<JointInfo>(_id)->name << "] with ["
          << dofs << "] DOFs\n";
    return false;
  }
  return true;
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
  GZ_PROFILE("JointFeatures::GetRevoluteJointAxis");
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
  GZ_PROFILE("JointFeatures::GetPrismaticJointAxis");
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
  GZ_PROFILE("JointFeatures::CastToJointType");
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
  GZ_PROFILE("JointFeatures::SetJointVelocityCommand");
  if (!this->ValidateDofParam(_id, _dof))
    return;

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

  auto modelInfo = this->ReferenceInterface<ModelInfo>(jointInfo->model);

  // clamp the values
  double velocity = std::clamp(_value,
      jointInfo->GetMinVelocity(_dof), jointInfo->GetMaxVelocity(_dof));

  const auto *identifier = std::get_if<InternalJoint>(&jointInfo->identifier);
  if (identifier &&
      this->ReferenceInterface<LinkInfo>(jointInfo->childLinkID)->isKinematic)
  {
    // The joint of a kinematic child link is driven by WorldForwardStep, which
    // integrates the joint position with the command for one step and, if the
    // link has a dynamic ancestor, drives the joint motor itself. Only the
    // first dof can currently be driven.
    if (_dof == 0)
    {
      jointInfo->kinematicJointVel = 0.0;
      jointInfo->kinematicJointVelCmd = velocity;
    }
    modelInfo->body->wakeUp();
    return;
  }

  auto motorIt = jointInfo->motors.find(_dof);
  if (_dof == 0 && (motorIt == jointInfo->motors.end() || !motorIt->second) &&
      jointInfo->motor)
  {
    jointInfo->motors[0] = jointInfo->motor;
    motorIt = jointInfo->motors.find(0);
  }

  if (motorIt == jointInfo->motors.end() || !motorIt->second)
  {
    auto *world = this->ReferenceInterface<WorldInfo>(modelInfo->world);
    // \todo(iche033) The motor constraint is created with a max impulse
    // computed by maxEffort * stepsize. However, our API supports
    // stepping sim with varying dt. We should recompute max impulse
    // if stepSize changes.
    auto motor = std::make_shared<GzMultiBodyJointMotor>(
      modelInfo->body.get(),
      std::get<InternalJoint>(jointInfo->identifier).indexInBtModel,
      static_cast<int>(_dof),
      static_cast<btScalar>(0),
      static_cast<btScalar>(jointInfo->GetMaxEffort(_dof) * world->stepSize));
    world->world->addMultiBodyConstraint(motor.get());
    jointInfo->motors[_dof] = motor;
    if (_dof == 0)
    {
      jointInfo->motor = motor;
    }
    motorIt = jointInfo->motors.find(_dof);
  }

  motorIt->second->setVelocityTarget(static_cast<btScalar>(velocity));
  modelInfo->body->wakeUp();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointMinPosition(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto *jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (std::isnan(_value))
  {
    gzerr << "Invalid minimum joint position value [" << _value
           << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
           << "]. The command will be ignored\n";
    return;
  }
  jointInfo->axisLower = _value;

  auto *modelInfo = this->ReferenceInterface<ModelInfo>(jointInfo->model);
  auto *worldInfo = this->ReferenceInterface<WorldInfo>(modelInfo->world);
  recreateJointLimitConstraint(jointInfo, modelInfo, worldInfo);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointMaxPosition(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto *jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (std::isnan(_value))
  {
    gzerr << "Invalid maximum joint position value [" << _value
           << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
           << "]. The command will be ignored\n";
    return;
  }

  jointInfo->axisUpper = _value;

  auto *modelInfo = this->ReferenceInterface<ModelInfo>(jointInfo->model);
  auto *worldInfo = this->ReferenceInterface<WorldInfo>(modelInfo->world);
  recreateJointLimitConstraint(jointInfo, modelInfo, worldInfo);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointMinVelocity(
    const Identity &_id, std::size_t _dof, double _value)
{
  if (!this->ValidateDofParam(_id, _dof))
    return;

  auto *jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (std::isnan(_value))
  {
    gzerr << "Invalid minimum joint velocity value [" << _value
          << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
          << "]. The command will be ignored\n";
    return;
  }

  jointInfo->minVelocityMap[_dof] = _value;
  if (_dof == 0)
    jointInfo->minVelocity = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointMaxVelocity(
    const Identity &_id, std::size_t _dof, double _value)
{
  if (!this->ValidateDofParam(_id, _dof))
    return;

  auto *jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (std::isnan(_value))
  {
    gzerr << "Invalid maximum joint velocity value [" << _value
          << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
          << "]. The command will be ignored\n";
    return;
  }

  jointInfo->maxVelocityMap[_dof] = _value;
  if (_dof == 0)
    jointInfo->maxVelocity = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointMinEffort(
    const Identity &_id, std::size_t _dof, double _value)
{
  if (!this->ValidateDofParam(_id, _dof))
    return;

  auto *jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (std::isnan(_value))
  {
    gzerr << "Invalid minimum joint effort value [" << _value
          << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
          << "]. The command will be ignored\n";

    return;
  }

  jointInfo->minEffortMap[_dof] = _value;
  if (_dof == 0)
    jointInfo->minEffort = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointMaxEffort(
    const Identity &_id, std::size_t _dof, double _value)
{
  if (!this->ValidateDofParam(_id, _dof))
    return;

  auto *jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (std::isnan(_value))
  {
    gzerr << "Invalid maximum joint effort value [" << _value
          << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
          << "]. The command will be ignored\n";
    return;
  }

  const auto *identifier = std::get_if<InternalJoint>(&jointInfo->identifier);
  if (!identifier)
    return;

  jointInfo->maxEffortMap[_dof] = _value;
  if (_dof == 0)
    jointInfo->maxEffort = _value;

  auto *modelInfo = this->ReferenceInterface<ModelInfo>(jointInfo->model);
  auto *world = this->ReferenceInterface<WorldInfo>(modelInfo->world);

  auto motorIt = jointInfo->motors.find(_dof);
  if (_dof == 0 && (motorIt == jointInfo->motors.end() || !motorIt->second) &&
      jointInfo->motor)
  {
    jointInfo->motors[0] = jointInfo->motor;
    motorIt = jointInfo->motors.find(0);
  }

  if (motorIt != jointInfo->motors.end() && motorIt->second)
  {
    world->world->removeMultiBodyConstraint(motorIt->second.get());
    motorIt->second.reset();
  }

  auto motor = std::make_shared<GzMultiBodyJointMotor>(
    modelInfo->body.get(),
    std::get<InternalJoint>(jointInfo->identifier).indexInBtModel,
    static_cast<int>(_dof),
    static_cast<btScalar>(0),
    static_cast<btScalar>(_value * world->stepSize));
  world->world->addMultiBodyConstraint(motor.get());
  jointInfo->motors[_dof] = motor;
  if (_dof == 0)
  {
    jointInfo->motor = motor;
  }
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachFixedJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &_name)
{
  GZ_PROFILE("JointFeatures::AttachFixedJoint");
  auto *linkInfo = this->ReferenceInterface<LinkInfo>(_childID);
  auto *modelInfo = this->ReferenceInterface<ModelInfo>(linkInfo->model);
  auto *parentLinkInfo = this->ReferenceInterface<LinkInfo>(
    _parent->FullIdentity());
  auto *parentModelInfo = this->ReferenceInterface<ModelInfo>(
    parentLinkInfo->model);
  auto *world = this->ReferenceInterface<WorldInfo>(modelInfo->world);

  auto jointID = this->addConstraint(
    JointInfo{
      _name + "_" + parentLinkInfo->name + "_" + linkInfo->name,
      FixedConstraintJoint{},
      _parent->FullIdentity().id,
      _childID,
      Eigen::Isometry3d(),
      Eigen::Isometry3d(),
      linkInfo->model
    });

  auto parentLinkIdx = parentLinkInfo->indexInModel.value_or(-1);
  auto childLinkIdx = linkInfo->indexInModel.value_or(-1);
  auto *jointInfo = this->ReferenceInterface<JointInfo>(jointID);

  jointInfo->fixedConstraint = std::make_shared<btMultiBodyFixedConstraint>(
    parentModelInfo->body.get(), parentLinkIdx,
    modelInfo->body.get(), childLinkIdx,
    btVector3(0, 0, 0), btVector3(0, 0, 0),
    btMatrix3x3::getIdentity(),
    btMatrix3x3::getIdentity());

  if (world != nullptr && world->world)
  {
    world->world->addMultiBodyConstraint(jointInfo->fixedConstraint.get());
    jointInfo->fixedConstraint->setMaxAppliedImpulse(btScalar(1e9));

    // Make child link static if parent is static
    // This is done by updating collision flags
    btMultiBodyLinkCollider *parentCollider = parentLinkInfo->collider.get();
    btMultiBodyLinkCollider *childCollider = linkInfo->collider.get();
    if (parentCollider && childCollider)
    {
      // disable collision between parent and child collider
      // \todo(iche033) if self collide is true, extend this to
      // disable collisions between all the links in the parent's model with
      // all the links in the child's model.
      parentCollider->setIgnoreCollisionCheck(childCollider, true);
      childCollider->setIgnoreCollisionCheck(parentCollider, true);

      // If parent link is static or fixed, recusively update child colliders
      // collision flags to be static.
      if (parentLinkInfo->isStaticOrFixed && !linkInfo->isStaticOrFixed)
      {
        makeColliderStatic(linkInfo);
        updateColliderFlagsRecursive(std::size_t(_childID),
            this->joints, this->links, makeColliderStatic);
      }
    }

    return this->GenerateIdentity(jointID, this->joints.at(jointID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void JointFeatures::DetachJoint(const Identity &_jointId)
{
  GZ_PROFILE("JointFeatures::DetachJoint");
  auto jointInfo = this->ReferenceInterface<JointInfo>(_jointId);
  if (jointInfo->fixedConstraint)
  {
    // Make links dynamic again as they were originally not static
    // This is done by reverting any collision flag changes
    // made in AttachJoint
    auto *linkInfo = this->ReferenceInterface<LinkInfo>(jointInfo->childLinkID);
    if (jointInfo->parentLinkID.has_value())
    {
      auto parentLinkInfo = this->links.at(jointInfo->parentLinkID.value());

      btMultiBodyLinkCollider *parentCollider = parentLinkInfo->collider.get();
      btMultiBodyLinkCollider *childCollider = linkInfo->collider.get();
      if (parentCollider && childCollider)
      {
        parentCollider->setIgnoreCollisionCheck(childCollider, false);
        childCollider->setIgnoreCollisionCheck(parentCollider, false);
        // Recursively make child colliders dynamic if they were originally
        // not static
        if (!linkInfo->isStaticOrFixed && isColliderStatic(linkInfo))
        {
          makeColliderDynamic(linkInfo);
          updateColliderFlagsRecursive(std::size_t(jointInfo->childLinkID),
              this->joints, this->links, makeColliderDynamic);
        }
      }
    }

    auto modelInfo = this->ReferenceInterface<ModelInfo>(jointInfo->model);
    if (modelInfo)
    {
      auto *world = this->ReferenceInterface<WorldInfo>(modelInfo->world);
      world->world->removeMultiBodyConstraint(jointInfo->fixedConstraint.get());
      jointInfo->fixedConstraint.reset();
      jointInfo->fixedConstraint = nullptr;
      modelInfo->body->wakeUp();
    }
  }
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformFromParent(
    const Identity &_id, const Pose3d &_pose)
{
  GZ_PROFILE("JointFeatures::SetJointTransformFromParent");
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  if (jointInfo->fixedConstraint)
  {
    Eigen::Isometry3d parentInertiaToLinkFrame = Eigen::Isometry3d::Identity();
    if (jointInfo->parentLinkID.has_value())
    {
      auto parentLinkInfo = this->links.at(jointInfo->parentLinkID.value());
      parentInertiaToLinkFrame = parentLinkInfo->inertiaToLinkFrame;
    }
    auto *linkInfo = this->ReferenceInterface<LinkInfo>(jointInfo->childLinkID);
    auto tf = convertTf(parentInertiaToLinkFrame * _pose *
                        linkInfo->inertiaToLinkFrame.inverse());
    jointInfo->fixedConstraint->setPivotInA(
      tf.getOrigin());
    jointInfo->fixedConstraint->setFrameInA(
      tf.getBasis());
  }
}

/////////////////////////////////////////////////
Wrench3d JointFeatures::GetJointTransmittedWrenchInJointFrame(
    const Identity &_id) const
{
  GZ_PROFILE("JointFeatures::GetJointTransmittedWrenchInJointFrame");
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  Wrench3d wrenchOut;

  // Convert the force and torque into the joint's frame of reference.
  wrenchOut.force = jointInfo->tf_to_child.rotation() * convert(
    jointInfo->jointFeedback->m_reactionForces.getLinear());
  wrenchOut.torque = jointInfo->tf_to_child.rotation() * convert(
    jointInfo->jointFeedback->m_reactionForces.getAngular());

  // If a constraint is used to move the joint, e.g motor constraint,
  // account for the applied constraint forces and torques.
  // \todo(iche033) Check whether this is also needed for gearbox constraint
  if (jointInfo->motor || !jointInfo->motors.empty())
  {
    auto linkInfo = this->ReferenceInterface<LinkInfo>(
        jointInfo->childLinkID);

    // link index in model should be >=0 because we expect the base link in
    // btMultibody to always be a parent link of a joint
    // (except when it's fixed to world)
    int linkIndexInModel = -1;
    if (linkInfo->indexInModel.has_value())
      linkIndexInModel = *linkInfo->indexInModel;
    if (linkIndexInModel >= 0)
    {
      auto *modelInfo = this->ReferenceInterface<ModelInfo>(linkInfo->model);
      btMultibodyLink &link = modelInfo->body->getLink(linkIndexInModel);

      wrenchOut.force +=
          jointInfo->tf_to_child.rotation().inverse() *
          convert(link.m_cachedWorldTransform.getBasis().inverse() *
          link.m_appliedConstraintForce);
      wrenchOut.torque +=
          jointInfo->tf_to_child.rotation().inverse() *
          convert(link.m_cachedWorldTransform.getBasis().inverse() *
          link.m_appliedConstraintTorque);
    }
  }

  return wrenchOut;
}

/////////////////////////////////////////////////
bool JointFeatures::SetJointMimicConstraint(
    const Identity &_id,
    std::size_t _dof,
    const BaseJoint3dPtr &_leaderJoint,
    std::size_t _leaderAxisDof,
    double _multiplier,
    double _offset,
    double _reference)
{
  GZ_PROFILE("JointFeatures::SetJointMimicConstraint");
  if (_dof != 0 || _leaderAxisDof != 0)
  {
    gzerr << "Failed to set mimic constraint for follower axis " << _dof
          << " and leader axis " << _leaderAxisDof
          << " because bullet-featherstone doesn't yet support mimic "
          << " constraints for multi-axis joints."
          << std::endl;
    return false;
  }

  auto followerJoint = this->ReferenceInterface<JointInfo>(_id);
  auto followerChild = this->ReferenceInterface<LinkInfo>(
      followerJoint->childLinkID);
  auto followerChildIndexInModel = followerChild->indexInModel.value_or(-1);
  const auto *model =
      this->ReferenceInterface<ModelInfo>(followerJoint->model);

  auto leaderJointId = _leaderJoint->FullIdentity();
  auto leaderJoint = this->ReferenceInterface<JointInfo>(leaderJointId);
  auto leaderChild = this->ReferenceInterface<LinkInfo>(
      leaderJoint->childLinkID);
  auto leaderChildIndexInModel = leaderChild->indexInModel.value_or(-1);

  // Get world pointer and remove an existing mimic / gear constraint
  // after follower and leader entities have been found.
  auto *world = this->ReferenceInterface<WorldInfo>(model->world);
  if (followerJoint->gearConstraint)
  {
    world->world->removeMultiBodyConstraint(
        followerJoint->gearConstraint.get());
    followerJoint->gearConstraint.reset();
  }

  followerJoint->gearConstraint = std::make_shared<btMultiBodyGearConstraint>(
      model->body.get(),
      followerChildIndexInModel,
      model->body.get(),
      leaderChildIndexInModel,
      btVector3(0, 0, 0),
      btVector3(0, 0, 0),
      btMatrix3x3::getIdentity(),
      btMatrix3x3::getIdentity());
  // btMultiBodyGearConstraint isn't clearly documented, but from trial and
  // and error, I found that the Gear Ratio should have the opposite sign
  // of the multiplier parameter.
  followerJoint->gearConstraint->setGearRatio(btScalar(-_multiplier));
  // Recall the linear equation from the definition of the mimic constraint:
  // follower_position = multiplier * (leader_position - reference) + offset
  //
  // The equation can be rewritten to collect the constant terms involving the
  // reference and offset parameters together:
  // follower_position = multiplier*leader_position
  //                   + offset - multiplier * reference
  // The RelativePositionTarget is then set as (offset - multiplier * reference)
  followerJoint->gearConstraint->setRelativePositionTarget(
      btScalar(_offset - _multiplier * _reference));
  // TODO(scpeters): figure out what is a good value for this
  followerJoint->gearConstraint->setMaxAppliedImpulse(btScalar(1e8));
  // setErp is needed to correct position constraint errors
  // this is especially relevant to the offset and reference parameters
  followerJoint->gearConstraint->setErp(btScalar(0.3));
  world->world->addMultiBodyConstraint(followerJoint->gearConstraint.get());
  return true;
}

/////////////////////////////////////////////////
void JointFeatures::SetFixedJointWeldChildToParent(
    const Identity &_id, bool _weldChildToParent)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  if (jointInfo->fixedConstraint)
  {
    jointInfo->fixedConstraintWeldChildToParent = _weldChildToParent;
  }
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
