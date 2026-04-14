/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <cmath>
#include <cstddef>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <gz/math/Helpers.hh>
#include "Base.hh"
#include "gz/physics/Geometry.hh"
#include "mujoco/mujoco.h"

#include "JointFeatures.hh"

namespace gz {
namespace physics {
namespace mujoco {

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    gzerr << "Cannot get position for joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return math::NAN_D;
  }

  if (jointInfo->nq_index < 0)
    return math::NAN_D;

  if (_dof > 0 && _dof > this->GetJointDegreesOfFreedom(_id) - 1)
  {
    gzerr << "Trying to access an invalid DOF [" << _dof << "] on joint [ "
          << jointInfo->name << "]\n";
    return math::NAN_D;
  }
  return jointInfo->worldInfo->mjDataObj->qpos[jointInfo->nq_index + _dof];
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    gzerr << "Cannot get velocity for joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return math::NAN_D;
  }

  if (jointInfo->nv_index < 0)
    return math::NAN_D;

  if (_dof > 0 && _dof > this->GetJointDegreesOfFreedom(_id) - 1)
  {
    gzerr << "Trying to access an invalid DOF [" << _dof << "] on joint [ "
          << jointInfo->name << "]\n";
    return math::NAN_D;
  }
  return jointInfo->worldInfo->mjDataObj->qvel[jointInfo->nv_index + _dof];
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    gzerr << "Cannot get acceleration for joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return math::NAN_D;
  }

  if (jointInfo->nv_index < 0)
    return math::NAN_D;

  if (_dof > 0 && _dof > this->GetJointDegreesOfFreedom(_id) - 1)
  {
    gzerr << "Trying to access an invalid DOF [" << _dof << "] on joint [ "
          << jointInfo->name << "]\n";
    return math::NAN_D;
  }
  return jointInfo->worldInfo->mjDataObj->qacc[jointInfo->nv_index + _dof];
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  if (!jointInfo->joint)
  {
    gzerr << "Cannot get force for joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return gz::math::NAN_D;
  }

  if (jointInfo->nv_index < 0)
    return math::NAN_D;

  if (_dof > 0 && _dof > this->GetJointDegreesOfFreedom(_id) - 1)
  {
    gzerr << "Trying to access an invalid DOF [" << _dof << "] on joint [ "
          << jointInfo->name << "]\n";
    return math::NAN_D;
  }
  return jointInfo->worldInfo->mjDataObj
      ->qfrc_actuator[jointInfo->nv_index + _dof];
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &_id) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    return this->GetJointTransformFromParent(_id) *
           this->GetJointTransformToChild(_id);
  }
  auto m = jointInfo->worldInfo->mjModelObj;
  int childBodyId = mjs_getId(jointInfo->childBody->element);
  if (childBodyId < 0 || childBodyId > m->nbody)
    return {};

  auto *parentBody = mjs_getParent(jointInfo->childBody->element);
  int parentBodyId = mjs_getId(parentBody ->element);

  auto d = jointInfo->worldInfo->mjDataObj;
  Eigen::Isometry3d parentPose =
      getBodyWorldPoseFromMjDataEigen(d, parentBodyId);
  Eigen::Isometry3d childPose = getBodyWorldPoseFromMjDataEigen(d, childBodyId);
  return parentPose.inverse() * childPose;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  // Take extra care that the value is finite. A nan might cause the physics
  // engine to fail
  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint position value [" << _value << "] set on joint ["
           << jointInfo->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }
  if (!jointInfo->joint)
  {
    gzerr << "Cannot set position on joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return;
  }

  if (jointInfo->nq_index < 0)
    return;

  if (_dof > 0 && _dof > this->GetJointDegreesOfFreedom(_id) - 1)
  {
    gzerr << "Trying to access an invalid DOF [" << _dof << "] on joint [ "
          << jointInfo->name << "]\n";
    return;
  }
  jointInfo->worldInfo->mjDataObj->qpos[jointInfo->nq_index + _dof] = _value;
  mj_forward(jointInfo->worldInfo->mjModelObj, jointInfo->worldInfo->mjDataObj);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  // Take extra care that the value is finite. A nan might cause the physics
  // engine to fail
  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint velocity value [" << _value << "] set on joint ["
           << jointInfo->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }
  if (!jointInfo->joint)
  {
    gzerr << "Cannot set velocity on joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return;
  }

  if (jointInfo->nv_index < 0)
    return;

  if (_dof > 0 && _dof > this->GetJointDegreesOfFreedom(_id) - 1)
  {
    gzerr << "Trying to access an invalid DOF [" << _dof << "] on joint [ "
          << jointInfo->name << "]\n";
    return;
  }
  jointInfo->worldInfo->mjDataObj->qvel[jointInfo->nv_index + _dof] = _value;
  mj_forward(jointInfo->worldInfo->mjModelObj, jointInfo->worldInfo->mjDataObj);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(
    const Identity &_id, std::size_t _dof, double _value)
{
  gzerr << "Setting joint acceleration is not supported by the MuJoCo physics "
           "plugin.\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  // Take extra care that the value is finite. A nan might cause the physics
  // engine to fail
  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint force value [" << _value << "] set on joint ["
           << jointInfo->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }
  if (!jointInfo->joint)
  {
    gzerr << "Cannot set force on joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return;
  }

  if (!jointInfo->actuator)
  {
    gzerr << "No actuator set up for this joint\n";
    return;
  }
  int ctrlIndex = mjs_getId(jointInfo->actuator->element);
  if (ctrlIndex < 0)
    return;

  if (_dof > 0 && _dof > this->GetJointDegreesOfFreedom(_id) - 1)
  {
    gzerr << "Trying to access an invalid DOF [" << _dof << "] on joint [ "
          << jointInfo->name << "]\n";
    return;
  }
  jointInfo->worldInfo->mjDataObj->ctrl[ctrlIndex + _dof] = _value;
  mj_forward(jointInfo->worldInfo->mjModelObj, jointInfo->worldInfo->mjDataObj);
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  if (!jointInfo->joint)
  {
    return 0;
  }
  auto m = jointInfo->worldInfo->mjModelObj;
  int bodyId = mjs_getId(jointInfo->childBody->element);
  if (bodyId < 0 || bodyId > m->nbody)
    return 0;
  return m->body_dofnum[bodyId];
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &_id) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  auto it = this->frames.find(_id);
  if (it != this->frames.end())
  {
    Pose3d childInParent =
        convertPose(jointInfo->childBody->pos, jointInfo->childBody->quat);
    Pose3d jointInChild =
        convertPose(it->second->site->pos, it->second->site->quat);

    return childInParent * jointInChild;
  }
  return {};
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const
{
  auto it = this->frames.find(_id.id);
  if (it != this->frames.end())
  {
    Pose3d jointInChild =
        convertPose(it->second->site->pos, it->second->site->quat);
    return jointInChild.inverse();
  }
  return {};
}
}
}
}

