/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gz/math/eigen3/Conversions.hh>

#include <unordered_map>
#include <utility>

namespace gz {
namespace physics {
namespace bullet_featherstone {

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
  if (dtDur)
  {
    std::chrono::duration<double> dt = *dtDur;
    stepSize = dt.count();
  }

  worldInfo->world->stepSimulation(this->stepSize, 1, this->stepSize);

  gzdbg << "SimulationFeatures::WorldForwardStep\n";
  gzdbg << " - getGravity: " << worldInfo->world->getGravity().z() << std::endl;
  gzdbg << " - getNumConstraints: " << worldInfo->world->getNumConstraints() << std::endl;
  gzdbg << " - getNumMultibodies: " << worldInfo->world->getNumMultibodies() << std::endl;
  gzdbg << " - getNumMultiBodyConstraints: " << worldInfo->world->getNumMultiBodyConstraints() << std::endl;

  this->Write(_h.Get<ChangedWorldPoses>());
}

/////////////////////////////////////////////////
void SimulationFeatures::Write(ChangedWorldPoses &_changedPoses) const
{
  // remove link poses from the previous iteration
  _changedPoses.entries.clear();
  _changedPoses.entries.reserve(this->links.size());

  std::unordered_map<std::size_t, math::Pose3d> newPoses;

  std::stringstream ss;
  ss << "SimulationFEatures::Write\n";
  size_t ii = 0;
  for (const auto &[id, info] : this->links)
  {
    const auto &model = this->ReferenceInterface<ModelInfo>(info->model);
    WorldPose wp;
    wp.pose = gz::math::eigen3::convert(GetWorldTransformOfLink(*model, *info));
    wp.body = id;

    auto iter = this->prevLinkPoses.find(id);
    if ((iter == this->prevLinkPoses.end()) ||
        !iter->second.Pos().Equal(wp.pose.Pos(), 1e-6) ||
        !iter->second.Rot().Equal(wp.pose.Rot(), 1e-6))
    {
      ii++;
      _changedPoses.entries.push_back(wp);
      newPoses[id] = wp.pose;
    }
    else
      newPoses[id] = iter->second;
  }

  ss << " - changedPoses: (" << ii << "/" << this->links.size() << ")\n";
  gzdbg << ss.str();

  // Save the new poses so that they can be used to check for updates in the
  // next iteration. Re-setting this->prevLinkPoses with the contents of
  // newPoses ensures that we aren't caching data for links that were removed
  this->prevLinkPoses = std::move(newPoses);
}
}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
