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
  // Free groups in bullet-featherstone are always represented by ModelInfo
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_groupID);
  auto linkInfo = modelInfo->LinkFromBody(modelInfo->body);
  if (!linkInfo) {
    return this->GenerateInvalidId();
  }
  return this->GenerateIdentity(linkInfo->entityId, linkInfo);
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldAngularVelocity(
    const Identity &_groupID, const AngularVelocity &_angularVelocity)
{
  // TODO(azeey): Implement SetFreeGroupWorldAngularVelocity
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldLinearVelocity(
    const Identity &_groupID, const LinearVelocity &_linearVelocity)
{
  // TODO(azeey): Implement SetFreeGroupLinearAngularVelocity
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
    const Identity &_groupID,
    const PoseType &_pose)
{
  // TODO(azeey): Implement SetFreeGroupWorldPose
}
}  // namespace mujoco
}  // namespace physics
}  // namespace gz
