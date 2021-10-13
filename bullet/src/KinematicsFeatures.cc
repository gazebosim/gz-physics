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

#include "KinematicsFeatures.hh"

#include <LinearMath/btVector3.h>

#include <ignition/common/Console.hh>
#include <stdexcept>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID& _id) const {
  FrameData3d data;

  // The feature system should never send us the world ID.
  if (_id.IsWorld()) {
    throw std::runtime_error(
        "Given a FrameID belonging to the world. This should not be possible! "
        "Please report this bug!");
    return data;
  }

  auto linkID = _id.ID();
  auto link = std::get<Link*>(this->entities.at(linkID));

  if (link->rootModel->multibody) {
    auto& multibody = link->rootModel->multibody;
    auto linkNumber = link->rootModel->vertexIdToLinkIndex.at(link->vertexId);

    auto localPose =
        link->rootModel->vertexIdToLinkPoseFromPivot.at(link->vertexId);

    auto localPos =
        convertVec(ignition::math::eigen3::convert(localPose.Pos()));
    auto localRot = convertQuat(localPose.Rot());

    auto pos = multibody->localPosToWorld(linkNumber, localPos);
    auto mat = multibody->localFrameToWorld(linkNumber, btMatrix3x3(localRot));
    data.pose.translation() = convert(pos);
    data.pose.linear() = convert(mat);

  } else {
    auto& body = link->rootModel->body;
    auto pose = body->getWorldTransform();
    data.pose.translation() = convert(pose.getOrigin());
    data.pose.linear() = convert(pose.getBasis());
  }

  return data;
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
