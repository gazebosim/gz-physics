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

  const Eigen::Isometry3d bodyPose = getBodyWorldPoseFromMjDataEigen(d, bodyId);
  data.pose = bodyPose * frame.offset;

  if (m->body_weldid[bodyId] == 0)
  {
    // Static bodies have zero velocity and acceleration.
    data.linearVelocity.setZero();
    data.angularVelocity.setZero();
    data.linearAcceleration.setZero();
    data.angularAcceleration.setZero();
  }
  else
  {
    const mjtNum *subtreeCom = &d->subtree_com[3 * m->body_rootid[bodyId]];
    mjtNum velocity[6];
    mju_transformSpatial(velocity, &d->cvel[6 * bodyId], 0,
                         data.pose.translation().data(), subtreeCom, nullptr);
    mju_copy3(data.angularVelocity.data(), velocity);
    mju_copy3(data.linearVelocity.data(), velocity + 3);

    // cacc is only valid after mj_rnePostConstraint, which WorldForwardStep
    // calls after every step. mj_forward clears the flag, so before the first
    // step the accelerations stay zero.
    if (d->flg_rnepost)
    {
      mjtNum acceleration[6];
      mju_transformSpatial(acceleration, &d->cacc[6 * bodyId], 0,
                           data.pose.translation().data(), subtreeCom,
                           nullptr);
      mju_copy3(data.angularAcceleration.data(), acceleration);

      // cacc is a spatial acceleration. The acceleration of the frame's
      // origin adds the angular velocity crossed with its linear velocity.
      data.linearAcceleration =
          Eigen::Vector3d(acceleration[3], acceleration[4], acceleration[5]) +
          data.angularVelocity.cross(data.linearVelocity);

      // MuJoCo offsets cacc by -gravity, which is what an accelerometer
      // measures. Add gravity back to report the acceleration of the frame.
      if (!(m->opt.disableflags & mjDSBL_GRAVITY))
      {
        data.linearAcceleration +=
            Eigen::Vector3d(m->opt.gravity[0], m->opt.gravity[1],
                            m->opt.gravity[2]);
      }
    }
  }
  return data;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
