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

#include "KinematicsFeatures.hh"

#include <mujoco/mujoco.h>

#include <gz/common/Console.hh>

#include "gz/physics/FrameData.hh"
namespace gz
{
namespace physics
{
namespace mujoco
{
/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  FrameData3d data;
  auto it = this->frames.find(_id.ID());
  if (it == this->frames.end())
  {
    gzerr << "Frame [" << _id.ID() << "] not found\n";
    return data;
  }
  auto worldInfo = it->second->worldInfo;
  this->RecompileSpec(*worldInfo);

  auto *m = worldInfo->mjModelObj;
  auto *d = worldInfo->mjDataObj;
  const auto &frame = *it->second;

  if (!frame.body)
  {
    gzerr << "Frame [" << _id.ID() << "] has a null body pointer\n";
    return data;
  }

  const int bodyId = mjs_getId(frame.body->element);
  if (bodyId < 0 || bodyId >= m->nbody)
  {
    gzerr << "Frame [" << _id.ID() << "] has an invalid body id\n";
    return data;
  }

  // World pose of the owning body, then apply the constant frame offset.
  // Read the orientation from xmat rather than xquat to avoid a
  // quaternion -> matrix conversion in this hot path.
  Eigen::Isometry3d bodyPose = Eigen::Isometry3d::Identity();
  // Eigen defaults to column-major, so we first create a map with row-major
  bodyPose.linear() =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          &d->xmat[9 * bodyId]);
  bodyPose.translation() =
      Eigen::Map<const Eigen::Vector3d>(&d->xpos[3 * bodyId]);

  data.pose = bodyPose * frame.offset;

  if (m->body_weldid[bodyId] == 0)
  {
    // Static bodies have zero velocity.
    data.linearVelocity.setZero();
    data.angularVelocity.setZero();
  }
  else
  {
    mjtNum velocity[6];
    mju_transformSpatial(velocity, &d->cvel[6 * bodyId], 0,
                         data.pose.translation().data(),
                         &d->subtree_com[3 * m->body_rootid[bodyId]], nullptr);
    mju_copy3(data.angularVelocity.data(), velocity);
    mju_copy3(data.linearVelocity.data(), velocity + 3);
  }
  return data;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
