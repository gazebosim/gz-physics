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
    gzerr << "Cannot set position on joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return math::NAN_D;
  }

  if (jointInfo->qposAddr < 0)
    return math::NAN_D;

  // TODO(azeey) Check that _dof is equal to the DOF of the body associated with
  // this joint
  return jointInfo->worldInfo->mjDataObj->qpos[jointInfo->qposAddr];
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    gzerr << "Cannot set position on joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return math::NAN_D;
  }

  if (jointInfo->qvelAccelAddr < 0)
    return math::NAN_D;

  // TODO(azeey) Check that _dof is equal to the DOF of the body associated with
  // this joint
  return jointInfo->worldInfo->mjDataObj->qvel[jointInfo->qvelAccelAddr];
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    gzerr << "Cannot set position on joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return math::NAN_D;
  }

  if (jointInfo->qvelAccelAddr < 0)
    return math::NAN_D;

  // TODO(azeey) Check that _dof is equal to the DOF of the body associated with
  // this joint
  return jointInfo->worldInfo->mjDataObj->qacc[jointInfo->qvelAccelAddr];
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(
    const Identity &_id, std::size_t _dof) const
{
  return 0;
  // return this->ReferenceInterface<JointInfo>(_id)->joint->getForce(_dof);
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &_id) const
{
  return {};
  // return this->ReferenceInterface<JointInfo>(_id)
  //     ->joint->getRelativeTransform();
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

  if (jointInfo->qposAddr < 0)
    return;

  // TODO(azeey) Check that _dof is equal to the DOF of the body associated with
  // this joint
  jointInfo->worldInfo->mjDataObj->qpos[jointInfo->qposAddr] = _value;
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

  if (jointInfo->qvelAccelAddr < 0)
    return;

  // TODO(azeey) Check that _dof is equal to the DOF of the body associated with
  // this joint
  jointInfo->worldInfo->mjDataObj->qvel[jointInfo->qvelAccelAddr] = _value;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto joint = this->ReferenceInterface<JointInfo>(_id)->joint;

  // Take extra care that the value is finite. A nan can cause the DART
  // constraint solver to fail, which will in turn either cause a crash or
  // collisions to fail
  // if (!std::isfinite(_value))
  // {
  //   gzerr << "Invalid joint acceleration value [" << _value
  //          << "] set on joint [" << joint->getName() << " DOF " << _dof
  //          << "]. The value will be ignored\n";
  //   return;
  // }
  // TODO(azeey): Implement
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto joint = this->ReferenceInterface<JointInfo>(_id)->joint;

  // Take extra care that the value is finite. A nan can cause the DART
  // constraint solver to fail, which will in turn either cause a crash or
  // collisions to fail
  if (!std::isfinite(_value))
  {
    // gzerr << "Invalid joint force value [" << _value << "] set on joint ["
    //        << joint->name << " DOF " << _dof
    //        << "]. The value will be ignored\n";
    return;
  }
  // TODO(azeey): Implement
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const
{
  // return this->ReferenceInterface<JointInfo>(_id)->joint->getNumDofs();
  return 0;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &_id) const
{
  // return this->ReferenceInterface<JointInfo>(_id)
  //     ->joint->getTransformFromParentBodyNode();
  return {};
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const
{
  // return this->ReferenceInterface<JointInfo>(_id)
  //     ->joint->getTransformFromChildBodyNode().inverse();
  return {};
}
}
}
}

