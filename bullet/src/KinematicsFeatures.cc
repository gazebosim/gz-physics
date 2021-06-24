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

#include <ignition/common/Console.hh>
#include "KinematicsFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  FrameData3d data;

  // The feature system should never send us the world ID.
  if (_id.IsWorld())
  {
    ignerr << "Given a FrameID belonging to the world. This should not be "
           << "possible! Please report this bug!\n";
    assert(false);
    return data;
  }

  const auto linkID = _id.ID();

  if (this->links.find(linkID) == this->links.end())
  {
    ignerr << "Given a FrameID not belonging to a link.\n";
    return data;
  }
  const auto &linkInfo = this->links.at(linkID);
  const auto &rigidBody = linkInfo->link;

  btTransform trans;
  trans = rigidBody->getCenterOfMassTransform();
  const btVector3 pos = trans.getOrigin();
  const btMatrix3x3 mat = trans.getBasis();

  const Eigen::Isometry3d poseIsometry =
    ignition::math::eigen3::convert(linkInfo->inertialPose.Inverse());
  Eigen::Isometry3d poseIsometryBase;
  poseIsometryBase.linear() = convert(mat);
  poseIsometryBase.translation() = convert(pos);
  poseIsometryBase =  poseIsometryBase * poseIsometry;
  data.pose.linear() = poseIsometryBase.linear();
  data.pose.translation() = poseIsometryBase.translation();

  // Add base velocities
  btVector3 omega = rigidBody->getAngularVelocity();
  btVector3 vel = rigidBody->getLinearVelocity();

  data.linearVelocity = convert(vel) + ignition::math::eigen3::convert(
    ignition::math::eigen3::convert(convert(omega)).Cross(
    -ignition::math::eigen3::convert(data.pose).Rot() *
    linkInfo->inertialPose.Pos()));
  data.angularVelocity = convert(omega);

  // \todo(anyone) compute frame accelerations

  return data;
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
