/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <cstddef>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/math/pose3.hh>

#include "JointFeatures.hh"

using namespace gz;
using namespace physics;
using namespace tpeplugin;

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointCount(const Identity &) const
{ return 0; }

/////////////////////////////////////////////////
Identity JointFeatures::GetJoint(const Identity &, std::size_t) const
{ return this->GenerateInvalidId(); }

/////////////////////////////////////////////////
Identity JointFeatures::GetJoint(const Identity &, const std::string &) const
{ return this->GenerateInvalidId(); }

/////////////////////////////////////////////////
const std::string &JointFeatures::GetJointName(const Identity &) const
{
  static const std::string kEmpty;
  return kEmpty;
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointIndex(const Identity &) const
{ return 0; }

/////////////////////////////////////////////////
Identity JointFeatures::GetModelOfJoint(const Identity &) const
{ return this->GenerateInvalidId(); }

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &) const
{ return 0; }

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &) const
{ return Pose3d::Identity(); }

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &) const
{ return Pose3d::Identity(); }

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(const Identity &, std::size_t) const
{ return 0.0; }

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(const Identity &, std::size_t) const
{ return 0.0; }

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(const Identity &, std::size_t) const
{ return 0.0; }

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(const Identity &, std::size_t) const
{ return 0.0; }

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &) const
{ return Pose3d::Identity(); }

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(const Identity &, std::size_t, double) {}
void JointFeatures::SetJointVelocity(const Identity &, std::size_t, double) {}
void JointFeatures::SetJointAcceleration(const Identity &, std::size_t, double) {}
void JointFeatures::SetJointForce(const Identity &, std::size_t, double) {}

/////////////////////////////////////////////////
void JointFeatures::SetJointTransformFromParent(
    const Identity &_id, const Pose3d &_pose)
{
  auto jointIt = this->joints.find(_id.id);
  if (jointIt == this->joints.end() || !jointIt->second)
    return;

  auto &jointInfo = jointIt->second;
  jointInfo->poseFromParent = math::eigen3::convert(_pose);

  // Get parent world pose
  math::Pose3d parentWorldPose;
  if (jointInfo->parentLinkId.has_value())
  {
    auto parentLinkIt = this->links.find(jointInfo->parentLinkId.value());
    if (parentLinkIt != this->links.end() && parentLinkIt->second)
      parentWorldPose = parentLinkIt->second->link->GetWorldPose();
  }

  // Compute child link target world pose
  math::Pose3d childLinkTargetWorldPose =
      parentWorldPose * jointInfo->poseFromParent;

  // Get child link and model
  auto childLinkIt = this->links.find(jointInfo->childLinkId);
  auto childModelIt = this->models.find(jointInfo->childModelId);
  if (childLinkIt == this->links.end() || !childLinkIt->second ||
      childModelIt == this->models.end() || !childModelIt->second)
    return;

  // Compute and set model world pose
  math::Pose3d linkLocalPose = childLinkIt->second->link->GetPose();
  math::Pose3d modelWorldPose = childLinkTargetWorldPose * linkLocalPose.Inverse();
  childModelIt->second->model->SetPose(modelWorldPose);
}

/////////////////////////////////////////////////
void JointFeatures::DetachJoint(const Identity &_jointId)
{
  auto it = this->joints.find(_jointId.id);
  if (it != this->joints.end())
    this->joints.erase(it);
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(const Identity &_jointID) const
{
  auto it = this->joints.find(_jointID.id);
  if (it != this->joints.end() && it->second)
    return this->GenerateIdentity(_jointID.id, it->second);
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity JointFeatures::AttachFixedJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &/*_name*/)
{
  auto childLinkIt = this->links.find(_childID.id);
  if (childLinkIt == this->links.end() || !childLinkIt->second)
  {
    gzerr << "Child link [" << _childID.id << "] not found" << std::endl;
    return this->GenerateInvalidId();
  }

  auto modelIt = this->childIdToParentId.find(_childID.id);
  if (modelIt == this->childIdToParentId.end())
  {
    gzerr << "Could not find model for child link [" << _childID.id << "]"
          << std::endl;
    return this->GenerateInvalidId();
  }

  std::optional<std::size_t> parentLinkId;
  if (_parent)
  {
    parentLinkId = _parent->FullIdentity().id;
    auto parentLinkIt = this->links.find(parentLinkId.value());
    if (parentLinkIt == this->links.end() || !parentLinkIt->second)
    {
      gzerr << "Parent link [" << parentLinkId.value() << "] not found"
            << std::endl;
      return this->GenerateInvalidId();
    }
  }

  static std::size_t nextJointId = 1000000;
  std::size_t jointId = nextJointId++;
  auto jointPtr = std::make_shared<JointInfo>(
      JointInfo{_childID.id, parentLinkId, math::Pose3d::Zero, modelIt->second});
  this->joints.insert({jointId, jointPtr});
  return this->GenerateIdentity(jointId, jointPtr);
}
