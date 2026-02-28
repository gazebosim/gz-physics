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

#include "FreeGroupFeatures.hh"
#include <unistd.h>

#include <gz/physics/FreeGroup.hh>

#include "Base.hh"
#include "mujoco/mujoco.h"
namespace gz
{
namespace physics
{
namespace mujoco
{
/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForModel(
    const Identity &_modelID) const
{
  // TODO(azeey): Implement checks to see if this model has a freegroup.

  return _modelID;
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForLink(
    const Identity &_linkID) const
{
  // TODO(azeey) This assumes the freegroup encompasses the entire model. Handle
  // the case where there could be multiple free groups within a model
  const auto *linkInfo = this->ReferenceInterface<LinkInfo>(_linkID);
  auto modelInfo = linkInfo->modelInfo.lock();
  if (!modelInfo)
  {
    return this->GenerateInvalidId();
  }

  return this->GenerateIdentity(modelInfo->entityId, modelInfo);
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupRootLink(const Identity &_groupID) const
{
  // TODO(azeey) This assumes the freegroup encompasses the entire model. Handle
  // the case where there could be multiple free groups within a model
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_groupID);
  auto linkInfo = modelInfo->LinkFromBody(modelInfo->body);
  if (!linkInfo)
  {
    return this->GenerateInvalidId();
  }
  return this->GenerateIdentity(linkInfo->entityId, linkInfo);
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldAngularVelocity(
    const Identity &_groupID, const AngularVelocity &_angularVelocity)
{
  // TODO(azeey) This assumes the freegroup encompasses the entire model. Handle
  // the case where there could be multiple free groups within a model
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_groupID);
  auto worldInfo = modelInfo->worldInfo;
  auto *d = worldInfo->mjDataObj;
  auto *m = worldInfo->mjModelObj;
  const auto bodyId = mjs_getId(modelInfo->body->element);
  const auto jntadr = m->body_jntadr[bodyId];
  const auto qveladr = m->jnt_dofadr[jntadr];
  mju_copy3(&d->qvel[qveladr] + 3, _angularVelocity.data());
  mj_forward(m, d);
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldLinearVelocity(
    const Identity &_groupID, const LinearVelocity &_linearVelocity)
{
  // TODO(azeey) This assumes the freegroup encompasses the entire model. Handle
  // the case where there could be multiple free groups within a model
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_groupID);
  auto worldInfo = modelInfo->worldInfo;
  auto *d = worldInfo->mjDataObj;
  auto *m = worldInfo->mjModelObj;
  const auto bodyId = mjs_getId(modelInfo->body->element);
  const auto jntadr = m->body_jntadr[bodyId];
  const auto qveladr = m->jnt_dofadr[jntadr];
  mju_copy3(&d->qvel[qveladr], _linearVelocity.data());
  mj_forward(m, d);
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
    const Identity &_groupID,
    const PoseType &_pose)
{
  // TODO(azeey) This assumes the freegroup encompasses the entire model. Handle
  // the case where there could be multiple free groups within a model
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_groupID);
  auto worldInfo = modelInfo->worldInfo;
  auto *d = worldInfo->mjDataObj;
  auto *m = worldInfo->mjModelObj;
  const auto bodyId = mjs_getId(modelInfo->body->element);
  const auto jntadr = m->body_jntadr[bodyId];
  const auto qposadr = m->jnt_qposadr[jntadr];
  const Eigen::Quaterniond quat(_pose.rotation());
  const double quatCoeffs[] = {quat.w(), quat.x(), quat.y(), quat.z()};
  mju_copy3(&d->qpos[qposadr], _pose.translation().data());
  mju_copy4(&d->qpos[qposadr]+3, quatCoeffs);
  mj_forward(m, d);
}
}  // namespace mujoco
}  // namespace physics
}  // namespace gz
