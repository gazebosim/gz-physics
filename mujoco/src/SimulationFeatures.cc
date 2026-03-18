
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

#include <unordered_map>
#include <utility>

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace
{

gz::math::Pose3d getBodyWorldPoseFromMjData(mjData *_d, int _bodyId)
{
  return gz::math::Pose3d(_d->xpos[3 * _bodyId],
                          _d->xpos[3 * _bodyId + 1],
                          _d->xpos[3 * _bodyId + 2],
                          _d->xquat[4 * _bodyId],
                          _d->xquat[4 * _bodyId + 1],
                          _d->xquat[4 * _bodyId + 2],
                          _d->xquat[4 * _bodyId + 3]);
}

}  // namespace

namespace gz
{
namespace physics
{
namespace mujoco
{

/////////////////////////////////////////////////
void SimulationFeatures::WorldForwardStep(const Identity &_worldID,
                                          ForwardStep::Output &_h,
                                          ForwardStep::State & /*_x*/,
                                          const ForwardStep::Input &_u)
{
  GZ_PROFILE("SimulationFeatures::WorldForwardStep");
  GZ_PROFILE_BEGIN("Recompile");
  auto worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  this->RecompileSpec(*worldInfo);
  auto *dtDur = _u.Query<std::chrono::steady_clock::duration>();
  double stepSize = 0.001;
  if (dtDur)
  {
    std::chrono::duration<double> dt = *dtDur;
    stepSize = dt.count();
  }
  GZ_PROFILE_END();

  worldInfo->mjModelObj->opt.timestep = stepSize;

  mj_step(worldInfo->mjModelObj, worldInfo->mjDataObj);

  this->WriteRequiredData(_h);
  this->Write(_h.Get<ChangedWorldPoses>());
}

/////////////////////////////////////////////////
void SimulationFeatures::Write(WorldPoses &_worldPoses) const
{
  _worldPoses.entries.clear();
  for (const auto &[worldId, worldInfo] : this->worlds.idToObject)
  {
    mjData *d = worldInfo->mjDataObj;
    for (const auto &[modelId, model] : worldInfo->models.idToObject)
    {
      for (const auto &[linkId, link] : model->links.idToObject)
      {
        int bodyId = mjs_getId(link->body->element);
        auto &wp = _worldPoses.entries.emplace_back();
        wp.pose = getBodyWorldPoseFromMjData(d, bodyId);
        wp.body = link->entityId;
      }
    }
  }
}

/////////////////////////////////////////////////
void SimulationFeatures::Write(ChangedWorldPoses &_changedPoses) const
{
  _changedPoses.entries.clear();
  for (const auto &[worldId, worldInfo] : this->worlds.idToObject)
  {
    mjData *d = worldInfo->mjDataObj;

    for (const auto &[modelId, model] : worldInfo->models.idToObject)
    {
      for (const auto &[linkId, link] : model->links.idToObject)
      {
        int bodyId = mjs_getId(link->body->element);
        if (bodyId < 0 || static_cast<std::size_t>(bodyId) >= worldInfo->prevBodyPoses.size())
          continue;

        WorldPose wp;
        wp.pose = getBodyWorldPoseFromMjData(d, bodyId);
        wp.body = linkId;

        // If the body's pose is new or has changed, save this new pose and
        // add it to the output poses. Otherwise, keep the existing body pose
        auto &prevPose = worldInfo->prevBodyPoses[bodyId];
        if (!prevPose.has_value() ||
            !prevPose->Pos().Equal(wp.pose.Pos(), 1e-6) ||
            !prevPose->Rot().Equal(wp.pose.Rot(), 1e-6))
        {
          _changedPoses.entries.push_back(wp);
          prevPose = wp.pose;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
std::vector<SimulationFeatures::ContactInternal>
SimulationFeatures::GetContactsFromLastStep(const Identity &_worldID) const
{
  std::vector<SimulationFeatures::ContactInternal> outContacts;
  auto *const worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);

  if (!worldInfo || !worldInfo->mjDataObj || !worldInfo->mjModelObj)
    return outContacts;

  const mjModel *m = worldInfo->mjModelObj;
  const mjData *d = worldInfo->mjDataObj;

  for (int i = 0; i < d->ncon; ++i)
  {
    const mjContact *con = d->contact + i;

    auto it1 = worldInfo->geomIdToShapeInfo.find(con->geom1);
    auto it2 = worldInfo->geomIdToShapeInfo.find(con->geom2);

    if (it1 != worldInfo->geomIdToShapeInfo.end() &&
        it2 != worldInfo->geomIdToShapeInfo.end())
    {
      const auto &shape1 = it1->second;
      const auto &shape2 = it2->second;

      CompositeData extraData;
      auto &extraContactData =
          extraData.Get<SimulationFeatures::ExtraContactData>();

      mjtNum contactForce[6];
      mj_contactForce(m, d, i, contactForce);

      mjtNum forceInWorldFrame[3];
      for (int j = 0; j < 3; ++j)
      {
        forceInWorldFrame[j] = con->frame[j] * contactForce[0] +
                               con->frame[3 + j] * contactForce[1] +
                               con->frame[6 + j] * contactForce[2];
      }

      extraContactData.force =
          Eigen::Vector3d(forceInWorldFrame[0], forceInWorldFrame[1],
                          forceInWorldFrame[2]);
      extraContactData.normal = Eigen::Vector3d(con->frame[0], con->frame[1],
                                                con->frame[2]);
      extraContactData.depth = -con->dist;

      outContacts.push_back(SimulationFeatures::ContactInternal {
          this->GenerateIdentity(shape1->entityId, shape1),
          this->GenerateIdentity(shape2->entityId, shape2),
          Eigen::Vector3d(con->pos[0], con->pos[1], con->pos[2]),
          extraData
      });
    }
  }

  return outContacts;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
