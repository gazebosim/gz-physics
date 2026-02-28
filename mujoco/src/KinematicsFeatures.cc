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

#include "gz/physics/FrameData.hh"
#include "mujoco/mujoco.h"
#include <gz/common/Console.hh>
#include <iostream>
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
    std::cerr << "Frame not found error\n";
    // TODO (azeey): Frame not found error
    return data;
  }
  auto worldInfo = it->second->worldInfo.lock();
  if (!worldInfo)
  {
    // TODO(azeey): WorldInfo weak ptr could not be locked
    std::cerr << "WorldInfo weak ptr could not be locked\n";
    return data;
  }
  auto * body = it->second->body;
  auto bodyId = mjs_getId(body->element);

  auto d = worldInfo->mjDataObj;
  // mju_printMat(&d->xpos[3 * bodyId], 1, 3);
  data.pose.translation() = Eigen::Map<Eigen::Vector3d>(&d->xpos[3 * bodyId]);
  // Eigen defaults to column-major, so we first create a map
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotMatMap(&d->xmat[9 * bodyId]);
  data.pose.linear() = rotMatMap;
  data.pose = data.pose * it->second->offset;
  // std::cout << "Pose:\n" << data.pose.matrix() << "\n";

  mjtNum velocity[6];
  mj_objectVelocity(worldInfo->mjModelObj, d, mjOBJ_XBODY, bodyId, velocity, 0);
  mju_copy3(data.angularVelocity.data(), velocity);
  mju_copy3(data.linearVelocity.data(), velocity+3);
  return data;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
