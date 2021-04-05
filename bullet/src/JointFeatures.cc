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
    switch(jointInfo->constraintType)
    {
      case static_cast<int>(::sdf::JointType::REVOLUTE) :
      {
        btHingeAccumulatedAngleConstraint* hinge =
          dynamic_cast<
          btHingeAccumulatedAngleConstraint*>(jointInfo->joint.get());
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
      break;
    default:
      ignwarn << "Not a valid getJointPosition type: " << jointType << "\n";
      break;
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
    switch(jointInfo->constraintType)
    {
      case static_cast<int>(::sdf::JointType::REVOLUTE) :
      {
        btHingeAccumulatedAngleConstraint* hinge =
          dynamic_cast<
          btHingeAccumulatedAngleConstraint*>(jointInfo->joint.get());
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
            btRigidBody *childLink =
              this->links.at(jointInfo->childLinkId)->link.get();
            btVector3 aux = childLink->getAngularVelocity();
            math::Vector3 angularVelocity(aux[0], aux[1], aux[2]);
            // result +=
            // globalAxis.Dot(convertVec(childLink->getAngularVelocity()));
            result += globalAxis.Dot(angularVelocity);
          }
          if (this->links.find(jointInfo->parentLinkId) != this->links.end())
          {
            btRigidBody *parentLink =
              this->links.at(jointInfo->parentLinkId)->link.get();
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
      break;
      default:
        ignwarn << "Not a valid getJointVelocity type: " << jointType << "\n";
        break;
    }
  }
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
    switch(jointInfo->constraintType)
    {
      case static_cast<int>(::sdf::JointType::REVOLUTE) :
      {
        btHingeAccumulatedAngleConstraint* hinge =
          static_cast<
          btHingeAccumulatedAngleConstraint*>(jointInfo->joint.get());
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
            btRigidBody *childLink =
              this->links.at(jointInfo->childLinkId)->link.get();
            btVector3 aux = childLink->getTotalTorque();
            math::Vector3 angularTorque(aux[0], aux[1], aux[2]);
            const btVector3 localInertia = childLink->getLocalInertia();
            math::Vector3 angularAcceleration(
              angularTorque[0]/localInertia[0],
              angularTorque[1]/localInertia[1],
              angularTorque[2]/localInertia[2]);
            result += globalAxis.Dot(angularTorque);
          }
          if (this->links.find(jointInfo->parentLinkId) != this->links.end())
          {
            btRigidBody *parentLink =
              this->links.at(jointInfo->parentLinkId)->link.get();
            btVector3 aux = parentLink->getTotalTorque();
            math::Vector3 angularTorque(aux[0], aux[1], aux[2]);
            const btVector3 localInertia = parentLink->getLocalInertia();
            math::Vector3 angularAcceleration(
              angularTorque[0]/localInertia[0],
              angularTorque[1]/localInertia[1],
              angularTorque[2]/localInertia[2]);
            result -= globalAxis.Dot(angularTorque);
          }
        }
        else
        {
          ignerr << "Corrupted joint at index:" << _id.id << "\n";
        }
      }
      break;
    default:
      ignwarn << "Not a valid getJointAcceleration type: " << jointType << "\n";
      break;
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
    switch(jointInfo->constraintType)
    {
      case static_cast<int>(::sdf::JointType::REVOLUTE) :
      {
        btHingeAccumulatedAngleConstraint* hinge =
          static_cast<
          btHingeAccumulatedAngleConstraint*>(jointInfo->joint.get());
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
            btRigidBody *childLink =
              this->links.at(jointInfo->childLinkId)->link.get();
            btVector3 aux = childLink->getTotalTorque();
            math::Vector3 angularTorque(aux[0], aux[1], aux[2]);
            result += globalAxis.Dot(angularTorque);
          }
          if (this->links.find(jointInfo->parentLinkId) != this->links.end())
          {
            btRigidBody *parentLink =
              this->links.at(jointInfo->parentLinkId)->link.get();
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
      break;
    default:
      ignwarn << "Not a valid getJointForce type: " << jointType << "\n";
      break;
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
    switch(jointInfo->constraintType)
    {
      case static_cast<int>(::sdf::JointType::REVOLUTE) :
      {
        btHingeAccumulatedAngleConstraint* hinge =
          static_cast<
          btHingeAccumulatedAngleConstraint*>(jointInfo->joint.get());
        if (hinge)
        {
          // TO-DO (blast545): Find how to address limitation caused by
	  // https://pybullet.org/Bullet/BulletFull/btHingeConstraint_8cpp_source.html#l00318
          //const double thresholdValue = std::max(std::min(_value, 0.1), -0.1);

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

          btVector3 hingeTorqueA = _value * hingeAxisWorldA;
          btVector3 hingeTorqueB = _value * hingeAxisWorldB;

          hinge->getRigidBodyA().applyTorque(hingeTorqueA);
          hinge->getRigidBodyB().applyTorque(-hingeTorqueB);
        }
      }
      break;
    default:
      ignwarn << "Not a valid setJointForce type: " << jointType << "\n";
      break;
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

  // Take extra care that the value is finite
  if (!std::isfinite(_value))
  {
    ignerr << "Invalid joint velocity value [" << _value << "] set on joint ["
           << jointInfo->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }

  // Check the type of joint and act accordignly
  switch(jointInfo->constraintType)
  {
  case static_cast<int>(::sdf::JointType::REVOLUTE) :
  {
    const auto &link = this->links.at(jointInfo->childLinkId)->link;
    btTransform trans;
    link->getMotionState()->getWorldTransform(trans);
    btVector3 motion = quatRotate(trans.getRotation(),
      convertVec(ignition::math::eigen3::convert(jointInfo->axis)));
    btVector3 angular_vel = motion * _value;
    link->setAngularVelocity(angular_vel);
  }
  break;
  default:
    ignwarn << "Not a valid setJointVelocityCommand type: "
            << jointInfo->constraintType << "\n";
    break;
  }
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const
{
  // Degrees of freedom may need to be saved in the JointInfo struct
  // Currently supporting 1DoF revolute joints and fixed joints
  const JointInfoPtr &jointInfo = this->joints.at(_id.id);
  if (jointInfo->constraintType == static_cast<int>(::sdf::JointType::FIXED))
  {
    return 0;
  }
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
Identity JointFeatures::CastToFixedJoint(
  const Identity &_jointID) const
{
  if (this->joints.find(_jointID.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_jointID.id);
    if (jointInfo->constraintType == static_cast<int>(::sdf::JointType::FIXED))
    {
      return this->GenerateIdentity(_jointID, this->Reference(_jointID));
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToRevoluteJoint(
    const Identity &_jointID) const
{
  if (this->joints.find(_jointID.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_jointID.id);
    if (
      jointInfo->constraintType == static_cast<int>(::sdf::JointType::REVOLUTE))
    {
      return this->GenerateIdentity(_jointID, this->Reference(_jointID));
    }
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const Identity &_jointID) const
{
  if (this->joints.find(_jointID.id) != this->joints.end())
  {
    const JointInfoPtr &jointInfo = this->joints.at(_jointID.id);
    btHingeAccumulatedAngleConstraint* hinge =
      dynamic_cast<btHingeAccumulatedAngleConstraint*>(jointInfo->joint.get());
    if (hinge)
    {
      btVector3 vec =
        hinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
        hinge->getFrameOffsetA().getBasis().getColumn(2);
      // math::Vector3 globalAxis(vec[0], vec[1], vec[2]);
      return AngularVector3d(vec[0], vec[1], vec[2]);
    }
  }
  ignerr << "Error getting revolute Joint axis: " << _jointID.id << " \n";
  return AngularVector3d();
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
