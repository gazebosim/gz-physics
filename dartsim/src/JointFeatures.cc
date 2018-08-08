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

#include "JointFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

double JointFeatures::GetJointPosition(
    const std::size_t _id, const std::size_t _dof) const
{
  return this->joints.at(_id)->getPosition(_dof);
}

double JointFeatures::GetJointVelocity(
    const std::size_t _id, const std::size_t _dof) const
{
  return this->joints.at(_id)->getVelocity(_dof);
}

double JointFeatures::GetJointAcceleration(
    const std::size_t _id, const std::size_t _dof) const
{
  return this->joints.at(_id)->getAcceleration(_dof);
}

double JointFeatures::GetJointForce(
    const std::size_t _id, const std::size_t _dof) const
{
  return this->joints.at(_id)->getForce(_dof);
}

Pose3d JointFeatures::GetJointTransform(const std::size_t _id) const
{
  return this->joints.at(_id)->getRelativeTransform();
}

void JointFeatures::SetJointPosition(
    const std::size_t _id, const std::size_t _dof, const double _value)
{
  this->joints.at(_id)->setPosition(_dof, _value);
}

void JointFeatures::SetJointVelocity(
    const std::size_t _id, const std::size_t _dof, const double _value)
{
  this->joints.at(_id)->setVelocity(_dof, _value);
}

void JointFeatures::SetJointAcceleration(
    const std::size_t _id, const std::size_t _dof, const double _value)
{
  this->joints.at(_id)->setAcceleration(_dof, _value);
}

void JointFeatures::SetJointForce(
    const std::size_t _id, const std::size_t _dof, const double _value)
{
  this->joints.at(_id)->setForce(_dof, _value);
}

std::size_t JointFeatures::GetJointDegreesOfFreedom(const std::size_t _id) const
{
  return this->joints.at(_id)->getNumDofs();
}

Pose3d JointFeatures::GetJointTransformFromParent(const std::size_t _id) const
{
  return this->joints.at(_id)->getTransformFromParentBodyNode();
}

Pose3d JointFeatures::GetJointTransformToChild(const std::size_t _id) const
{
  return this->joints.at(_id)->getTransformFromChildBodyNode().inverse();
}

void JointFeatures::SetJointTransformFromParent(
    const std::size_t _id, const Pose3d &_pose)
{
  this->joints.at(_id)->setTransformFromParentBodyNode(_pose);
}

void JointFeatures::SetJointTransformToChild(
    const std::size_t _id, const Pose3d &_pose)
{
  this->joints.at(_id)->setTransformFromChildBodyNode(_pose.inverse());
}

}
}
}
