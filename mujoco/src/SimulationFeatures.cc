
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

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

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

  auto &worldPoses = _h.Get<WorldPoses>();
  worldPoses.entries.clear();
  mjData *d = worldInfo->mjDataObj;
  for (const auto &[modelId, model] : worldInfo->models.idToObject)
  {
    for (const auto &[linkId, link] : model->links.idToObject)
    {
      auto bodyId = mjs_getId(link->body->element);
      auto &wp = worldPoses.entries.emplace_back();
      wp.pose.Pos().Set(d->xpos[3 * bodyId], d->xpos[3 * bodyId + 1],
                        d->xpos[3 * bodyId + 2]);
      wp.pose.Rot().Set(d->xquat[4 * bodyId], d->xquat[4 * bodyId + 1],
                        d->xquat[4 * bodyId + 2], d->xquat[4 * bodyId + 3]);
      wp.body = link->entityId;
    }
  }
  // TODO(azeey) This simply copies all links instead of only the ones with
  // changed poses.
  auto &changedPoses = _h.Get<ChangedWorldPoses>();
  changedPoses.entries = worldPoses.entries;
}

/////////////////////////////////////////////////
void SimulationFeatures::Write(WorldPoses &/* _worldPoses */) const
{
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

      mjtNum f_contact[6];
      mj_contactForce(m, d, i, f_contact);

      mjtNum f_world[3];
      for (int j = 0; j < 3; ++j)
      {
        f_world[j] = con->frame[j] * f_contact[0] +
                     con->frame[3 + j] * f_contact[1] +
                     con->frame[6 + j] * f_contact[2];
      }

      extraContactData.force =
          Eigen::Vector3d(f_world[0], f_world[1], f_world[2]);
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
