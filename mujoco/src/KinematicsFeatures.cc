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
  auto *linkInfo = this->FrameInterface<LinkInfo>(_id);
  if (linkInfo)
  {
    auto worldInfo = linkInfo->modelInfo.lock()->worldInfo.lock();
    this->RecompileSpec(*worldInfo);
    mjData *d = worldInfo->mjDataObj;

    auto bodyId = mjs_getId(linkInfo->body->element);
    data.pose.translation() << d->xpos[3 * bodyId], d->xpos[3 * bodyId + 1],
        d->xpos[3 * bodyId + 2];

    Eigen::Quaterniond quat{d->xquat[4 * bodyId], d->xquat[4 * bodyId + 1],
                            d->xquat[4 * bodyId + 2], d->xquat[4 * bodyId + 3]};
    data.pose.linear() = quat.matrix();
    return data;
  }

  auto *modelInfo = this->FrameInterface<ModelInfo>(_id);
  if (modelInfo)
  {
    auto worldInfo = modelInfo->worldInfo.lock();
    this->RecompileSpec(*worldInfo);
    mjData *d = worldInfo->mjDataObj;

    auto bodyId = mjs_getId(modelInfo->body->element);
    data.pose.translation() << d->xpos[3 * bodyId], d->xpos[3 * bodyId + 1],
        d->xpos[3 * bodyId + 2];
    Eigen::Quaterniond quat{d->xquat[4 * bodyId], d->xquat[4 * bodyId + 1],
                            d->xquat[4 * bodyId + 2], d->xquat[4 * bodyId + 3]};
    data.pose.linear() = quat.matrix();
    return data;
  }
  return data;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
