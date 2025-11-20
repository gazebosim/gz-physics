
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

#include "SimulationFeatures.hh"
#include <mujoco/mjdata.h>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace gz
{
namespace physics
{
namespace mujoco
{

/////////////////////////////////////////////////
void SimulationFeatures::WorldForwardStep(
    const Identity &_worldID,
    ForwardStep::Output & _h,
    ForwardStep::State & /*_x*/,
    const ForwardStep::Input & _u)
{
  const auto worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  auto *dtDur =
    _u.Query<std::chrono::steady_clock::duration>();
  double stepSize = 0.001;
  if (dtDur)
  {
    std::chrono::duration<double> dt = *dtDur;
    stepSize = dt.count();
  }

  mj_step(worldInfo->mjModelObj, worldInfo->mjDataObj);

  auto &worldPoses = _h.Get<WorldPoses>();
  const mjData* d = worldInfo->mjDataObj;
  for (const auto &model: worldInfo->models)
  {
    for (const auto &link: model->links)
    {
      auto bodyId = mjs_getId(link->body->element);
      WorldPose wp;
      math::Vector3d pos{d->xpos[0], d->xpos[1], d->xpos[2]};
      math::Quaterniond quat{d->xquat[0], d->xquat[1], d->xquat[2], d->xquat[3]};
      wp.pose.Set(pos, quat);
      wp.body = bodyId;
      worldPoses.entries.push_back(wp);
    }
  }
}

/////////////////////////////////////////////////
void SimulationFeatures::Write(WorldPoses &_worldPoses) const
{
  // remove link poses from the previous iteration
  // _worldPoses.entries.clear();
  // _worldPoses.entries.reserve(this->links.size());
  //
  // for (const auto &[id, info] : this->links)
  // {
  //   const auto &model = this->ReferenceInterface<ModelInfo>(info->model);
  //   WorldPose wp;
  //   wp.pose = gz::math::eigen3::convert(GetWorldTransformOfLink(*model, *info));
  //   wp.body = id;
  //   _worldPoses.entries.push_back(wp);
  // }
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
