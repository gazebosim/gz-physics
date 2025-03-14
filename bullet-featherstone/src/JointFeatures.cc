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
#include <sdf/Joint.hh>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
void recreateJointLimitConstraint(JointInfo *_jointInfo, ModelInfo *_modelInfo,
    WorldInfo *_worldInfo)
{
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
void makeColliderStatic(LinkInfo *_linkInfo)
{
  btMultiBodyLinkCollider *childCollider = _linkInfo->collider.get();
  if (!childCollider)
    return;

  // if link is already static or fixed, we do not need to change its
  // collision flags
  if (_linkInfo->isStaticOrFixed)
    return;

  btBroadphaseProxy *childProxy = childCollider->getBroadphaseHandle();
  if (!childProxy)
    return;

  childProxy->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
  childProxy->m_collisionFilterMask =
      btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter;
#if BT_BULLET_VERSION >= 307
  childCollider->setDynamicType(btCollisionObject::CF_STATIC_OBJECT);
#endif
}

/////////////////////////////////////////////////
void makeColliderDynamic(LinkInfo *_linkInfo)
{
  btMultiBodyLinkCollider *childCollider = _linkInfo->collider.get();
  if (!childCollider)
    return;

  btBroadphaseProxy *childProxy = childCollider->getBroadphaseHandle();
  if (!childProxy)
    return;

  // If broadphase and collision object flags do not agree, the
  // link was originally non-static but made static by AttachJoint
  if (!_linkInfo->isStaticOrFixed &&
      ((childProxy->m_collisionFilterGroup &
      btBroadphaseProxy::StaticFilter) > 0))
  {
    childProxy->m_collisionFilterGroup =
        btBroadphaseProxy::DefaultFilter;
    childProxy->m_collisionFilterMask = btBroadphaseProxy::AllFilter;
#if BT_BULLET_VERSION >= 307
    childCollider->setDynamicType(btCollisionObject::CF_DYNAMIC_OBJECT);
#endif
  }
}

/////////////////////////////////////////////////
void updateColliderFlagsRecursive(std::size_t _linkID,
  const std::unordered_map<std::size_t, std::shared_ptr<JointInfo>> &_joints,
  const std::unordered_map<std::size_t, std::shared_ptr<LinkInfo>> &_links,
  std::function<void(LinkInfo *)> _updateColliderCb)
{
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
    const Identity &_id, const std::size_t /*_dof*/) const
{
  double results = gz::math::NAN_D;
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);

  if (identifier)
  {
    const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
    auto feedback = model->body->getLink(
      identifier->indexInBtModel).m_jointFeedback;
    const auto &link = model->body->getLink(identifier->indexInBtModel);
    results = 0.0;
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
  const auto *joint = this->ReferenceInterface<JointInfo>(_id);
  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return;

  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  model->body->getJointPosMultiDof(identifier->indexInBtModel)[_dof] =
      static_cast<btScalar>(_value);
  model->body->wakeUp();
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
  model->body->getJointVelMultiDof(identifier->indexInBtModel)[_dof] =
      static_cast<btScalar>(_value);
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
  auto *joint = this->ReferenceInterface<JointInfo>(_id);

  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint force value [" << _value
           << "] commanded on joint [" << joint->name << " DOF " << _dof
           << "]. The command will be ignored\n";
    return;
  }

  const auto *identifier = std::get_if<InternalJoint>(&joint->identifier);
  if (!identifier)
    return;

  const auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
  auto *world = this->ReferenceInterface<WorldInfo>(model->world);

  // Disable velocity control by removing joint motor constraint
  if (joint->motor)
  {
    world->world->removeMultiBodyConstraint(joint->motor.get());
    joint->motor.reset();
  }

  // clamp the values
  double force = std::clamp(_value,
      joint->minEffort, joint->maxEffort);

  model->body->getJointTorqueMultiDof(
    identifier->indexInBtModel)[_dof] = static_cast<btScalar>(force);
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

  auto modelInfo = this->ReferenceInterface<ModelInfo>(jointInfo->model);
  if (!jointInfo->motor)
  {
    auto *world = this->ReferenceInterface<WorldInfo>(modelInfo->world);
    // \todo(iche033) The motor constraint is created with a max impulse
    // computed by maxEffort * stepsize. However, our API supports
    // stepping sim with varying dt. We should recompute max impulse
    // if stepSize changes.
    jointInfo->motor = std::make_shared<btMultiBodyJointMotor>(
      modelInfo->body.get(),
      std::get<InternalJoint>(jointInfo->identifier).indexInBtModel,
      0,
      static_cast<btScalar>(0),
      static_cast<btScalar>(jointInfo->maxEffort * world->stepSize));
    world->world->addMultiBodyConstraint(jointInfo->motor.get());
  }

  // clamp the values
  double velocity = std::clamp(_value,
      jointInfo->minVelocity, jointInfo->maxVelocity);

  jointInfo->motor->setVelocityTarget(static_cast<btScalar>(velocity));
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
  auto *jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (std::isnan(_value))
  {
    gzerr << "Invalid minimum joint velocity value [" << _value
          << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
          << "]. The command will be ignored\n";
    return;
  }

  jointInfo->minVelocity = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointMaxVelocity(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto *jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (std::isnan(_value))
  {
    gzerr << "Invalid maximum joint velocity value [" << _value
          << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
          << "]. The command will be ignored\n";
    return;
  }

  jointInfo->maxVelocity = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointMinEffort(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto *jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (std::isnan(_value))
  {
    gzerr << "Invalid minimum joint effort value [" << _value
          << "] commanded on joint [" << jointInfo->name << " DOF " << _dof
          << "]. The command will be ignored\n";

    return;
  }

  jointInfo->minEffort = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointMaxEffort(
    const Identity &_id, std::size_t _dof, double _value)
{
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

  jointInfo->maxEffort = _value;

  auto *modelInfo = this->ReferenceInterface<ModelInfo>(jointInfo->model);
  auto *world = this->ReferenceInterface<WorldInfo>(modelInfo->world);

  if (jointInfo->motor)
  {
    world->world->removeMultiBodyConstraint(jointInfo->motor.get());
    jointInfo->motor.reset();
  }

  jointInfo->motor = std::make_shared<btMultiBodyJointMotor>(
    modelInfo->body.get(),
    std::get<InternalJoint>(jointInfo->identifier).indexInBtModel,
    0,
    static_cast<btScalar>(0),
    static_cast<btScalar>(jointInfo->maxEffort * world->stepSize));
  world->world->addMultiBodyConstraint(jointInfo->motor.get());
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachFixedJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &_name)
{
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
      InternalJoint{0},
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
        btBroadphaseProxy *childProxy = childCollider->getBroadphaseHandle();
        if (childProxy)
        {
          // Recursively make child colliders dynamic if they were originally
          // not static
          if (!linkInfo->isStaticOrFixed &&
              ((childProxy->m_collisionFilterGroup &
              btBroadphaseProxy::StaticFilter) > 0))
          {
            makeColliderDynamic(linkInfo);
            updateColliderFlagsRecursive(std::size_t(jointInfo->childLinkID),
                this->joints, this->links, makeColliderDynamic);
          }
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
  if (jointInfo->motor)
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
}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
