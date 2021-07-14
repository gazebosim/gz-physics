/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include "ignition/physics/detail/Identity.hh"

#include <algorithm>
#include <regex>
#include <sdf/Joint.hh>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(const Identity &_id,
                                       const std::size_t /* _dof */) const {
  double result = ignition::math::NAN_D;
  auto joint = std::get<Joint *>(this->entities.at(_id));

  auto jointType = joint->type;

  switch (joint->type) {
    case (::sdf::JointType::REVOLUTE): {
      btHingeAccumulatedAngleConstraint *hinge =
          dynamic_cast<btHingeAccumulatedAngleConstraint *>(
              joint->constraint.get());
      if (hinge != nullptr) {
        result = hinge->getAccumulatedHingeAngle();
        // result -= this->angleOffset;
      } else {
        ignerr << "Corrupted joint at index:" << _id.id << "\n";
      }
    } break;
    default:
      ignwarn << "Not a valid getJointPosition type: " << (int)jointType
              << "\n";
      break;
  }
  return result;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(const Identity &_id,
                                       const std::size_t /*_dof */) const {
  double result = ignition::math::NAN_D;

  auto joint = std::get<Joint *>(this->entities.at(_id.id));

  switch (joint->type) {
    case (::sdf::JointType::REVOLUTE): {
      auto *hinge = dynamic_cast<btHingeAccumulatedAngleConstraint *>(
          joint->constraint.get());
      if (hinge) {
        result = 0.0;
        // Get the axis of the joint
        auto axis =
            hinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
            hinge->getFrameOffsetA().getBasis().getColumn(2);

        math::Vector3 globalAxis(axis[0], axis[1], axis[2]);

        auto aux = joint->childBody->getAngularVelocity();
        math::Vector3 angularVelocityChild(aux[0], aux[1], aux[2]);
        // result +=
        // globalAxis.Dot(convertVec(childLink->getAngularVelocity()));
        result += globalAxis.Dot(angularVelocityChild);
        if (joint->parentBody != nullptr) {
          aux = joint->parentBody->getAngularVelocity();
          math::Vector3 angularVelocityParent(aux[0], aux[1], aux[2]);
          // result -=
          // globalAxis.Dot(convertVec(parentLink->getAngularVelocity()));
          result -= globalAxis.Dot(angularVelocityParent);
        }
      } else {
        ignerr << "Corrupted joint at index:" << _id.id << "\n";
      }
    } break;
    default:
      ignwarn << "Not a valid joint type: " << static_cast<int>(joint->type)
              << "\n";
      break;
  }

  return result;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(const Identity &_id,
                                           const std::size_t /*_dof */) const {
  double result = ignition::math::NAN_D;

  auto joint = std::get<Joint *>(this->entities.at(_id.id));

  switch (joint->type) {
    case (::sdf::JointType::REVOLUTE): {
      auto *hinge = static_cast<btHingeAccumulatedAngleConstraint *>(
          joint->constraint.get());
      if (hinge) {
        result = 0.0;
        // Get the axis of the joint
        auto axis =
            hinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
            hinge->getFrameOffsetA().getBasis().getColumn(2);

        math::Vector3 globalAxis(axis[0], axis[1], axis[2]);

        auto aux = joint->childBody->getTotalTorque();
        math::Vector3 childAngularTorque(aux[0], aux[1], aux[2]);
        const auto childLocalInertia = joint->childBody->getLocalInertia();
        math::Vector3 angularAcceleration(childAngularTorque[0] / childLocalInertia[0],
                                          childAngularTorque[1] / childLocalInertia[1],
                                          childAngularTorque[2] / childLocalInertia[2]);
        result += globalAxis.Dot(childAngularTorque);

        if (joint->parentBody != nullptr) {
          auto parentTorque = joint->parentBody->getTotalTorque();
          math::Vector3 parentAngularTorque(parentTorque[0], parentTorque[1],
                                            parentTorque[2]);
          const auto parentLocalInertia = joint->parentBody->getLocalInertia();
          math::Vector3 parentAngularAcceleration(
              parentAngularTorque[0] / parentLocalInertia[0],
              parentAngularTorque[1] / parentLocalInertia[1],
              parentAngularTorque[2] / parentLocalInertia[2]);
          result -= globalAxis.Dot(parentAngularAcceleration);
        }
      } else {
        ignerr << "Corrupted joint at index:" << _id.id << "\n";
      }
    } break;
    default:
      ignwarn << "Not a valid joint type: " << static_cast<int>(joint->type)
              << "\n";
      break;
  }

  return result;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(const Identity &_id,
                                    const std::size_t /*_dof*/) const {
  double result = ignition::math::NAN_D;

  auto joint = std::get<Joint *>(this->entities.at(_id.id));

  switch(joint->type)
  {
    case (::sdf::JointType::REVOLUTE):
    {
      auto* hinge =
        static_cast<
        btHingeAccumulatedAngleConstraint*>(joint->constraint.get());
      if (hinge)
      {
        result = 0.0;
        // Get the axis of the joint
        btVector3 vec =
          hinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
          hinge->getFrameOffsetA().getBasis().getColumn(2);

        math::Vector3 globalAxis(vec[0], vec[1], vec[2]);

        btVector3 aux = joint->childBody->getTotalTorque();
        math::Vector3 childAngularTorque(aux[0], aux[1], aux[2]);
        result += globalAxis.Dot(childAngularTorque);

        if (joint->parentBody != nullptr)
        {
          aux = joint->parentBody->getTotalTorque();
          math::Vector3 parentAngularTorque(aux[0], aux[1], aux[2]);
          result -= globalAxis.Dot(parentAngularTorque);
        }
      }
      else
      {
        ignerr << "Corrupted joint at index:" << _id.id << "\n";
      }
    }
    break;
  default:
    ignwarn << "Not a valid getJointForce type: " << static_cast<int>(joint->type) << "\n";
    break;
  }
  return result;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &_id) const {
  (void)_id;
  ignwarn << "Dummy function GetJointTransform\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(const Identity &_id,
                                     const std::size_t _dof,
                                     const double _value) {
  (void)_id;
  (void)_dof;
  (void)_value;
  ignwarn << "Dummy function SetJointPosition\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(const Identity &_id,
                                     const std::size_t _dof,
                                     const double _value) {
  (void)_id;
  (void)_dof;
  (void)_value;
  ignwarn << "Dummy SetJointVelocity\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(const Identity &_id,
                                         const std::size_t _dof,
                                         const double _value) {
  (void)_id;
  (void)_dof;
  (void)_value;
  ignwarn << "Dummy SetJointAcceleration\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(const Identity &_id, const std::size_t _dof,
                                  const double _value) {
  (void)_dof;
  (void)_id;

  auto joint = std::get<Joint *>(this->entities.at(_id.id));

  switch(joint->type)
  {
    case (::sdf::JointType::REVOLUTE) :
    {
      auto* hinge =
        static_cast<
        btHingeAccumulatedAngleConstraint*>(joint->constraint.get());
      if (hinge)
      {
        /* TO-DO (blast545): Find how to address limitation caused by:
         https://pybullet.org/Bullet/BulletFull/btHingeConstraint_8cpp_source.html#l00318
         double thresholdValue = std::max(std::min(_value, 0.1), -0.1);
        */

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
    ignwarn << "Not a valid setJointForce type: " << static_cast<int>(joint->type) << "\n";
    break;
  }
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocityCommand(const Identity &_id,
                                            const std::size_t _dof,
                                            const double _value) {
  // Only support available for single DoF joints
  auto joint = std::get<Joint *>(this->entities.at(_id.id));

  // Take extra care that the value is finite
  if (!std::isfinite(_value))
  {
    ignerr << "Invalid joint velocity value [" << _value << "] set on joint ["
           << joint->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }

  // Check the type of joint and act accordignly
  switch(joint->type)
  {
  case (::sdf::JointType::REVOLUTE):
  {
    auto child = joint->childBody;
    btTransform trans;
    child->getMotionState()->getWorldTransform(trans);
    auto motion = quatRotate(trans.getRotation(),
      convertVec(ignition::math::eigen3::convert(joint->axis)));
    auto angularVel = motion * _value;
    child->setAngularVelocity(angularVel);
  }
  break;
  default:
    ignwarn << "Not a valid setJointVelocityCommand type: "
            << static_cast<int>(joint->type) << "\n";
    break;
  }
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const {
  // Degrees of freedom may need to be saved in the JointInfo struct
  // Currently supporting 1DoF revolute joints and fixed joints
  auto joint = std::get<Joint *>(this->entities.at(_id));
  if (joint->type == ::sdf::JointType::FIXED) {
    return 0;
  }
  return 1;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &_id) const {
  (void)_id;
  ignwarn << "Dummy get joint transform from parent\n";
  return Pose3d();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const {
  (void)_id;
  ignwarn << "Dummy get joint transform to child\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::DetachJoint(const Identity &_jointID) { (void)_jointID; }

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(const Identity &_jointID) const {
  return _jointID;
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToRevoluteJoint(const Identity &_jointID) const {
  return _jointID;
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const Identity &/*_jointID*/) const {
  return AngularVector3d();
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
