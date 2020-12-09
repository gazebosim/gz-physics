/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(
    const Identity &_id, const std::size_t _dof) const
{
  ignwrn << "Dummy function GetJointPosition\n";
  return 0.0;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, const std::size_t _dof) const
{
  ignwrn << "Dummy function GetJointVelocity\n";
  return 0.0;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(
    const Identity &_id, const std::size_t _dof) const
{
  ignwrn << "Dummy function GetJointAcceleration\n";
  return 0.0;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(
    const Identity &_id, const std::size_t _dof) const
{
  ignwrn << "Dummy function GetJointForce\n";
  return 0.0;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &_id) const
{
  ignwrn << "Dummy function GetJointTransform\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  ignwrn << "Dummy function SetJointPosition\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  ignwrn << "Dummy SetJointVelocity\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  ignwrn << "Dummy SetJointAcceleration\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  ignwrn << "Dummy SetJointForce\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocityCommand(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  ignwrn << "Dummy SetJointVelocityCommand\n";
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const
{
  ignwrn << "Dummy GetJointDegreesOfFreedom\n";
  return 0;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &_id) const
{
  ignwrn << "Dummy get joint transform from parent\n";
  return Pose3d();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const
{
  ignwrn << "Dummy get joint transform to child\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformFromParent(
    const Identity &_id, const Pose3d &_pose)
{
  ignwrn << "Dummy set joint transform from parent\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformToChild(
    const Identity &_id, const Pose3d &_pose)
{
  ignwrn << "Dummy set joint transform to child\n";
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachFixedJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &_name)
{
  ignwrn << "Dummy AttachFixedJoint\n";
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const Identity &_jointID) const
{
  ignwrn << "Dummy GetRevoluteJointAxis\n";
  return AngularVector3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetRevoluteJointAxis(
    const Identity &_jointID, const AngularVector3d &_axis)
{
  ignwrn << "Dummy SetRevoluteJointAxis\n";
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachRevoluteJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &_name,
    const AngularVector3d &_axis)
{
  ignwrn << "Dummy Attach RevoluteJoint\n";
  return this->GenerateInvalidId();
}

}
}
}
