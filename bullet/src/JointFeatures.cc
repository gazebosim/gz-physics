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

#include <sdf/Joint.hh>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(
    const Identity &_id, const std::size_t _dof) const
{
  (void) _id;
  (void) _dof;
  // igndbg << "Dummy function GetJointPosition\n";
  return 0.0;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, const std::size_t _dof) const
{
  (void) _id;
  (void) _dof;
  // igndbg << "Dummy function GetJointVelocity\n";
  return 0.0;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(
    const Identity &_id, const std::size_t _dof) const
{
  (void) _id;
  (void) _dof;
  // ignwarn << "Dummy function GetJointAcceleration\n";
  return 0.0;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(
    const Identity &_id, const std::size_t _dof) const
{
  (void) _id;
  (void) _dof;
  // ignwarn << "Dummy function GetJointForce\n";
  return 0.0;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &_id) const
{
  (void) _id;
  // ignwarn << "Dummy function GetJointTransform\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  (void) _id;
  (void) _dof;
  (void) _value;
  // ignwarn << "Dummy function SetJointPosition\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  (void) _id;
  (void) _dof;
  (void) _value;
  // ignwarn << "Dummy SetJointVelocity\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  (void) _id;
  (void) _dof;
  (void) _value;
  // ignwarn << "Dummy SetJointAcceleration\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  (void) _id;
  (void) _dof;
  (void) _value;
  // ignwarn << "Dummy SetJointForce\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocityCommand(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  // (void) _id;
  // (void) _dof;
  // (void) _value;
 //ignwarn << "Dummy SetJointVelocityCommand\n";

  // Only support available for single DoF joints
  (void) _dof;
  const auto &jointInfo = this->joints.at(_id);

  // Take extra care that the value is finite. A nan can cause the DART
  // constraint solver to fail, which will in turn either cause a crash or
  // collisions to fail
  if (!std::isfinite(_value))
  {
    ignerr << "Invalid joint velocity value [" << _value << "] set on joint ["
           << jointInfo->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }

  // Check the type of joint and act accordignly
  if(jointInfo->constraintType == static_cast<int>(::sdf::JointType::REVOLUTE)) {
    btHingeConstraint * hinge = dynamic_cast<btHingeConstraint *> (jointInfo->joint);

    // // This value was set arbitrarily
    // const float maxMotorImpulse = 1000.0f;
    // const float targetVelocity = _value;
    // hinge->enableAngularMotor(true, targetVelocity, maxMotorImpulse);

    btVector3 axis(0,1,0);
    this->links.at(jointInfo->childLinkId)->link->setAngularVelocity(_value * axis);

    //ignerr << "MOTOR ENABLED : " << targetVelocity << std::endl;

  }
  else {
    // // igndbg << "Sending command to not revolute joint\n";
  }

}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const
{
  (void) _id;
  // TO-DO: Degrees of freedom may need to be saved in the JointInfo struct
  // As bullet's constraints do not save this info
  // igndbg << "Dummy GetJointDegreesOfFreedom\n";
  return 1;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &_id) const
{
  (void) _id;
  // ignwarn << "Dummy get joint transform from parent\n";
  return Pose3d();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const
{
  (void) _id;
  // ignwarn << "Dummy get joint transform to child\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformFromParent(
    const Identity &_id, const Pose3d &_pose)
{
  (void) _id;
  (void) _pose;
  // ignwarn << "Dummy set joint transform from parent\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformToChild(
    const Identity &_id, const Pose3d &_pose)
{
  (void) _id;
  (void) _pose;
  // ignwarn << "Dummy set joint transform to child\n";
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(
  const Identity &_jointID) const
{
  (void) _jointID;
  // ignwarn << "Dummy CastToFixedJoint\n";
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachFixedJoint(
  const Identity &_childID,
  const BaseLink3dPtr &_parent,
  const std::string &_name)
{
  (void) _childID;
  (void) _parent;
  (void) _name;
  // ignwarn << "Dummy AttachFixedJoint\n";
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToRevoluteJoint(
    const Identity &_jointID) const
{
  (void) _jointID;
  // ignwarn << "Dummy CastToRevoluteJoint\n";
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const Identity &_jointID) const
{
  (void) _jointID;
  // ignwarn << "Dummy GetRevoluteJointAxis\n";
  return AngularVector3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetRevoluteJointAxis(
    const Identity &_jointID, const AngularVector3d &_axis)
{
  (void) _jointID;
  (void) _axis;
  // ignwarn << "Dummy SetRevoluteJointAxis\n";
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachRevoluteJoint(
  const Identity &_childID,
  const BaseLink3dPtr &_parent,
  const std::string &_name,
  const AngularVector3d &_axis)
{
  (void) _childID;
  (void) _parent;
  (void) _name;
  (void) _axis;
  // ignwarn << "Dummy Attach RevoluteJoint\n";
  return this->GenerateInvalidId();
}

}
}
}
