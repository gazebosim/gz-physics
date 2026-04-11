
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

#include <memory>

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

  auto *m = worldInfo->mjModelObj;
  auto *d = worldInfo->mjDataObj;
  // gzdbg << "qfrc_applied: ";
  // mju_printMat(d->qfrc_applied, 1, m->nv);
  // gzdbg << "ctrl: ";
  // mju_printMat(d->ctrl, 1, m->nu);
  // gzdbg << "actuator_force: ";
  // mju_printMat(d->actuator_force, 1, m->nu);
  // gzdbg << "qfrc_actuator: ";
  // mju_printMat(d->qfrc_actuator, 1, m->nv);

  mj_step(m, d);
  // Clear joint applied forces so that they are not applied in the next
  // timestep, which is the expected behavior in Gazebo.
  // gzmsg << "qfrc_applied: ";
  // mju_printMat(d->qfrc_applied, 1, m->nv);
  // gzmsg << "ctrl: ";
  // mju_printMat(d->ctrl, 1, m->nu);
  // gzmsg << "actuator_force: ";
  // mju_printMat(d->actuator_force, 1, m->nu);
  // gzmsg << "qfrc_actuator: ";
  // mju_printMat(d->qfrc_actuator, 1, m->nv);

  std::fill(d->ctrl, d->ctrl + m->nu, 0.0);

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
        if (bodyId < 0 ||
            static_cast<std::size_t>(bodyId) >= worldInfo->prevBodyPoses.size())
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

    std::shared_ptr<ShapeInfo> shape1 = nullptr;
    std::shared_ptr<ShapeInfo> shape2 = nullptr;

    if (con->geom[0] >= 0 &&
        static_cast<std::size_t>(con->geom[0]) <
            worldInfo->geomIdToShapeInfo.size())
      shape1 = worldInfo->geomIdToShapeInfo[con->geom[0]];
    if (con->geom[1] >= 0 &&
        static_cast<std::size_t>(con->geom[1]) <
            worldInfo->geomIdToShapeInfo.size())
      shape2 = worldInfo->geomIdToShapeInfo[con->geom[1]];

    if (shape1 && shape2)
    {
      CompositeData extraData;
      auto &extraContactData =
          extraData.Get<SimulationFeatures::ExtraContactData>();

      mjtNum contactForce[6];
      mj_contactForce(m, d, i, contactForce);

      // In mujoco, con->frame is a rotation matrix that transforms vectors
      // from the world frame to the contact frame (storing local axes as rows).
      // We multiply the local contact force by its transpose (inverse) to
      // transform the force back into the world frame.
      using Matrix3RowMajor = Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>;
      Eigen::Map<const Matrix3RowMajor> contactFrame(con->frame);
      Eigen::Map<const Eigen::Vector<mjtNum, 3>> localForce(contactForce);

      extraContactData.force =
          (contactFrame.transpose() * localForce).cast<double>();
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
