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

#include <algorithm>
#include <ostream>
#include <regex>
#include <sdf/Joint.hh>

#include "ignition/physics/detail/Identity.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(const Identity& _id,
                                       const std::size_t /* _dof */) const {
  auto joint = std::get<Joint*>(this->entities.at(_id));
  auto jointIndex = joint->rootModel->edgeIdToJointIndex.at(joint->edgeId);
  return joint->rootModel->multibody->getJointPos(jointIndex);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(const Identity& _id,
                                       const std::size_t /*_dof */) const {
  auto joint = std::get<Joint*>(this->entities.at(_id));
  auto jointIndex = joint->rootModel->edgeIdToJointIndex.at(joint->edgeId);
  return joint->rootModel->multibody->getJointVel(jointIndex);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(const Identity& _id,
                                           const std::size_t /*_dof */) const {
  return ignition::math::NAN_D;
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(const Identity& _id,
                                    const std::size_t /*_dof*/) const {
  auto joint = std::get<Joint*>(this->entities.at(_id));
  auto jointIndex = joint->rootModel->edgeIdToJointIndex.at(joint->edgeId);
  return joint->rootModel->multibody->getJointTorque(jointIndex);
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity& _id) const {
  (void)_id;
  ignwarn << "Dummy function GetJointTransform\n";
  return Pose3d();
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(const Identity& _id,
                                     const std::size_t _dof,
                                     const double _value) {
  (void) _dof;
  auto joint = std::get<Joint*>(this->entities.at(_id));
  auto jointIndex = joint->rootModel->edgeIdToJointIndex.at(joint->edgeId);
  // TODO(joxoby): negative value?
  joint->rootModel->multibody->setJointPos(jointIndex, _value);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(const Identity& _id,
                                     const std::size_t _dof,
                                     const double _value) {
  (void) _dof;
  auto joint = std::get<Joint*>(this->entities.at(_id));
  auto jointIndex = joint->rootModel->edgeIdToJointIndex.at(joint->edgeId);
  joint->rootModel->multibody->setJointVel(jointIndex, _value);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(const Identity& _id,
                                         const std::size_t _dof,
                                         const double _value) {
  (void)_id;
  (void)_dof;
  (void)_value;
  ignwarn << "Dummy SetJointAcceleration\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(const Identity& _id,
                                  const std::size_t _dof,
                                  const double _value) {
  (void) _dof;
  auto joint = std::get<Joint*>(this->entities.at(_id));
  auto jointIndex = joint->rootModel->edgeIdToJointIndex.at(joint->edgeId);
  joint->rootModel->multibody->addJointTorque(jointIndex, _value);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocityCommand(const Identity& _id,
                                            const std::size_t _dof,
                                            const double _value) {
  (void) _dof;
  auto joint = std::get<Joint*>(this->entities.at(_id));
  auto jointIndex = joint->rootModel->edgeIdToJointIndex.at(joint->edgeId);
  joint->rootModel->multibody->setJointVel(jointIndex, _value);
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity& _id) const {
  // Degrees of freedom may need to be saved in the JointInfo struct
  // Currently supporting 1DoF revolute joints and fixed joints
  auto joint = std::get<Joint*>(this->entities.at(_id));
  if (joint->type == ::sdf::JointType::FIXED) {
    return 0;
  }
  return 1;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity& _id) const {
  (void)_id;
  ignwarn << "Dummy get joint transform from parent\n";
  return Pose3d();
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity& _id) const {
  (void)_id;
  ignwarn << "Dummy get joint transform to child\n";
  return Pose3d();
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(const Identity& _jointID) const {
  return _jointID;
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToRevoluteJoint(const Identity& _jointID) const {
  return _jointID;
}

/////////////////////////////////////////////////
AngularVector3d JointFeatures::GetRevoluteJointAxis(
    const Identity& /*_jointID*/) const {
  return AngularVector3d();
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
