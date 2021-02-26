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

#include <algorithm>
#include <sdf/Joint.hh>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(
    const Identity &_id, const std::size_t _dof) const
{
  (void) _dof;
  double result = ignition::math::NAN_D;
  if (this->joints.find(_id.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_id.id);
    const int jointType = jointInfo->constraintType;
    // Check the type of joint and act accordignly
    if (jointInfo->constraintType ==
        static_cast<int>(::sdf::JointType::REVOLUTE))
    {
      btHingeAccumulatedAngleConstraint* hinge =
        static_cast<btHingeAccumulatedAngleConstraint*>(jointInfo->joint);
      if (hinge)
      {
        result = hinge->getAccumulatedHingeAngle();
        // result -= this->angleOffset;
      }
      else
      {
        ignerr << "Corrupted joint at index:" << _id.id << "\n";
      }
    }
    else
    {
      ignerr << "Not a valid constrating type: " << jointType << "\n";
    }
  }
  return result;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, const std::size_t _dof) const
{
  (void) _dof;
  double result = ignition::math::NAN_D;
  if (this->joints.find(_id.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_id.id);
    const int jointType = jointInfo->constraintType;
    if (jointInfo->constraintType ==
        static_cast<int>(::sdf::JointType::REVOLUTE))
    {
      btHingeAccumulatedAngleConstraint* hinge =
        static_cast<btHingeAccumulatedAngleConstraint*>(jointInfo->joint);
      if (hinge)
      {
        result = 0.0;
        // Get the axis of the joint
        btVector3 vec =
          hinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
          hinge->getFrameOffsetA().getBasis().getColumn(2);

        math::Vector3 globalAxis(vec[0], vec[1], vec[2]);

        if (this->links.find(jointInfo->childLinkId) != this->links.end())
        {
          btRigidBody *childLink = this->links.at(jointInfo->childLinkId)->link;
          btVector3 aux = childLink->getAngularVelocity();
          math::Vector3 angularVelocity(aux[0], aux[1], aux[2]);
          // result +=
          // globalAxis.Dot(convertVec(childLink->getAngularVelocity()));
          result += globalAxis.Dot(angularVelocity);
        }
        if (this->links.find(jointInfo->parentLinkId) != this->links.end())
        {
          btRigidBody *parentLink =
            this->links.at(jointInfo->parentLinkId)->link;
          btVector3 aux = parentLink->getAngularVelocity();
          math::Vector3 angularVelocity(aux[0], aux[1], aux[2]);
          // result -=
          // globalAxis.Dot(convertVec(parentLink->getAngularVelocity()));
          result -= globalAxis.Dot(angularVelocity);
        }
      }
      else
      {
        ignerr << "Corrupted joint at index:" << _id.id << "\n";
      }
    }
    else
    {
      ignerr << "Not a valid constrating type: " << jointType << "\n";
    }
  }
  igndbg << "Joint Velocity: " << _id.id << " -> " << result << std::endl;
  return result;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(
    const Identity &_id, const std::size_t _dof) const
{
  (void) _dof;
  double result = ignition::math::NAN_D;
  if (this->joints.find(_id.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_id.id);
    const int jointType = jointInfo->constraintType;
    if (jointInfo->constraintType ==
        static_cast<int>(::sdf::JointType::REVOLUTE))
    {
      btHingeAccumulatedAngleConstraint* hinge =
        static_cast<btHingeAccumulatedAngleConstraint*>(jointInfo->joint);
      if (hinge)
      {
        result = 0.0;
        // Get the axis of the joint
        btVector3 vec =
          hinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
          hinge->getFrameOffsetA().getBasis().getColumn(2);

        math::Vector3 globalAxis(vec[0], vec[1], vec[2]);

        if (this->links.find(jointInfo->childLinkId) != this->links.end())
        {
          btRigidBody *childLink = this->links.at(jointInfo->childLinkId)->link;
          btVector3 aux = childLink->getTotalTorque();
          math::Vector3 angularTorque(aux[0], aux[1], aux[2]);

          // TODO(blast545): divide inertia
          result += globalAxis.Dot(angularTorque);
        }
        if (this->links.find(jointInfo->parentLinkId) != this->links.end())
        {
          btRigidBody *parentLink =
            this->links.at(jointInfo->parentLinkId)->link;
          btVector3 aux = parentLink->getTotalTorque();
          math::Vector3 angularTorque(aux[0], aux[1], aux[2]);
          result -= globalAxis.Dot(angularTorque);
        }
      }
      else
      {
        ignerr << "Corrupted joint at index:" << _id.id << "\n";
      }
    }
    else
    {
      ignerr << "Not a valid constrating type: " << jointType << "\n";
    }
  }
  return result;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(
    const Identity &_id, const std::size_t _dof) const
{
  (void) _dof;
  double result = ignition::math::NAN_D;
  if (this->joints.find(_id.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_id.id);
    const int jointType = jointInfo->constraintType;
    if (jointInfo->constraintType ==
        static_cast<int>(::sdf::JointType::REVOLUTE))
    {
      btHingeAccumulatedAngleConstraint* hinge =
        static_cast<btHingeAccumulatedAngleConstraint*>(jointInfo->joint);
      if (hinge)
      {
        result = 0.0;
        // Get the axis of the joint
        btVector3 vec =
          hinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
          hinge->getFrameOffsetA().getBasis().getColumn(2);

        math::Vector3 globalAxis(vec[0], vec[1], vec[2]);

        if (this->links.find(jointInfo->childLinkId) != this->links.end())
        {
          btRigidBody *childLink = this->links.at(jointInfo->childLinkId)->link;
          btVector3 aux = childLink->getTotalTorque();
          math::Vector3 angularTorque(aux[0], aux[1], aux[2]);
          result += globalAxis.Dot(angularTorque);
        }
        if (this->links.find(jointInfo->parentLinkId) != this->links.end())
        {
          btRigidBody *parentLink =
            this->links.at(jointInfo->parentLinkId)->link;
          btVector3 aux = parentLink->getTotalTorque();
          math::Vector3 angularTorque(aux[0], aux[1], aux[2]);
          result -= globalAxis.Dot(angularTorque);
        }
      }
      else
      {
        ignerr << "Corrupted joint at index:" << _id.id << "\n";
      }
    }
    else
    {
      ignerr << "Not a valid constrating type: " << jointType << "\n";
    }
  }
  return result;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &_id) const
{
  (void) _id;
  ignwarn << "Dummy function GetJointTransform\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  (void) _id;
  (void) _dof;
  (void) _value;
  ignwarn << "Dummy function SetJointPosition\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  (void) _id;
  (void) _dof;
  (void) _value;
  ignwarn << "Dummy SetJointVelocity\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  (void) _id;
  (void) _dof;
  (void) _value;
  ignwarn << "Dummy SetJointAcceleration\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  (void) _dof;

  if (this->joints.find(_id.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_id.id);
    const int jointType = jointInfo->constraintType;
    if (jointType == static_cast<int>(::sdf::JointType::REVOLUTE))
    {
      btHingeAccumulatedAngleConstraint* hinge =
        static_cast<btHingeAccumulatedAngleConstraint*>(jointInfo->joint);
      if (hinge)
      {
        // Limit the max torque applied to avoid abrupt changes in the
        // angular position of the joint and losing the angle reference
        // TO-DO (blast545): this limitation should be based on angular speed
        // as this breaks the PID controller when setting high values
        const double thresholdValue = std::max(std::min(_value, 0.1), -0.1);

        // z-axis of constraint frame
        btVector3 hingeAxisLocalA =
          hinge->getFrameOffsetA().getBasis().getColumn(2);
        btVector3 hingeAxisLocalB =
          hinge->getFrameOffsetB().getBasis().getColumn(2);

        btVector3 hingeAxisWorldA =
          hinge->getRigidBodyA().getWorldTransform().getBasis() *
          hingeAxisLocalA;
        btVector3 hingeAxisWorldB =
          hinge->getRigidBodyB().getWorldTransform().getBasis() *
          hingeAxisLocalB;

        btVector3 hingeTorqueA = thresholdValue * hingeAxisWorldA;
        btVector3 hingeTorqueB = thresholdValue * hingeAxisWorldB;

        hinge->getRigidBodyA().applyTorque(hingeTorqueA);
        hinge->getRigidBodyB().applyTorque(-hingeTorqueB);
      }
    }
  }
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocityCommand(
    const Identity &_id, const std::size_t _dof, const double _value)
{
  // Only support available for single DoF joints
  (void) _dof;
  const auto &jointInfo = this->joints.at(_id.id);

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
  if (jointInfo->constraintType ==
      static_cast<int>(::sdf::JointType::REVOLUTE)) {
    const auto &link = this->links.at(jointInfo->childLinkId)->link;
    btTransform trans;
    link->getMotionState()->getWorldTransform(trans);
    btVector3 motion = quatRotate(trans.getRotation(),
      convertVec(ignition::math::eigen3::convert(jointInfo->axis)));
    btVector3 angular_vel = motion * _value;
    link->setAngularVelocity(angular_vel);
  } else {
    // igndbg << "Sending command to not revolute joint\n";
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
  ignwarn << "Dummy get joint transform from parent\n";
  return Pose3d();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const
{
  (void) _id;
  ignwarn << "Dummy get joint transform to child\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformFromParent(
    const Identity &_id, const Pose3d &_pose)
{
  (void) _id;
  (void) _pose;
  ignwarn << "Dummy set joint transform from parent\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformToChild(
    const Identity &_id, const Pose3d &_pose)
{
  (void) _id;
  (void) _pose;
  ignwarn << "Dummy set joint transform to child\n";
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(
  const Identity &_jointID) const
{
  (void) _jointID;
  ignwarn << "Dummy CastToFixedJoint\n";
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
  ignwarn << "Dummy AttachFixedJoint\n";
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToRevoluteJoint(
    const Identity &_jointID) const
{
  (void) _jointID;
  ignwarn << "Dummy CastToRevoluteJoint\n";
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const Identity &_jointID) const
{
  (void) _jointID;
  ignwarn << "Dummy GetRevoluteJointAxis\n";
  return AngularVector3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetRevoluteJointAxis(
    const Identity &_jointID, const AngularVector3d &_axis)
{
  (void) _jointID;
  (void) _axis;
  ignwarn << "Dummy SetRevoluteJointAxis\n";
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
  ignwarn << "Dummy Attach RevoluteJoint\n";
  return this->GenerateInvalidId();
}

}
}
}
