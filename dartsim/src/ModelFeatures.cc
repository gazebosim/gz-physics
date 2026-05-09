/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "ModelFeatures.hh"

namespace gz {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
void ModelFeatures::SetModelStatic(const Identity &_id, bool _static)
{
  auto modelInfo = this->GetModelInfo(_id);
  if (!modelInfo || !modelInfo->model)
    return;
  auto skeleton = modelInfo->model;

  // In DART, isMobile=false means the skeleton is fixed (static).
  skeleton->setMobile(!_static);

  // When transitioning between states, zero out any
  // velocities and accelerations that DART may have stored on the skeleton
  skeleton->resetVelocities();
  skeleton->resetAccelerations();
  skeleton->clearExternalForces();
  skeleton->clearInternalForces();

  // Also apply to nested models recursively.
  for (const std::size_t nestedID : modelInfo->nestedModels)
  {
    this->SetModelStatic(this->GenerateIdentity(
        nestedID, this->GetModelInfo(nestedID)), _static);
  }
}

/////////////////////////////////////////////////
bool ModelFeatures::GetModelStatic(const Identity &_id) const
{
  const auto modelInfo = this->GetModelInfo(_id);
  if (!modelInfo || !modelInfo->model)
    return false;
  return !modelInfo->model->isMobile();
}

/////////////////////////////////////////////////
void ModelFeatures::SetModelGravityEnabled(
    const Identity &_id, bool _enabled)
{
  auto modelInfo = this->GetModelInfo(_id);
  if (!modelInfo || !modelInfo->model)
    return;

  for (const auto &linkInfo : modelInfo->links)
  {
    this->SetLinkGravityMode(linkInfo.get(), _enabled);
  }

  // Also apply to nested models recursively.
  for (const std::size_t nestedID : modelInfo->nestedModels)
  {
    this->SetModelGravityEnabled(this->GenerateIdentity(
        nestedID, this->GetModelInfo(nestedID)), _enabled);
  }
}

/////////////////////////////////////////////////
bool ModelFeatures::GetModelGravityEnabled(const Identity &_id) const
{
  const auto modelInfo = this->GetModelInfo(_id);
  if (!modelInfo || !modelInfo->model)
    return true;

  for (const auto &linkInfo : modelInfo->links)
  {
    if (!this->GetLinkGravityMode(linkInfo.get()))
      return false;
  }

  // Also check nested models.
  for (const std::size_t nestedID : modelInfo->nestedModels)
  {
    if (!this->GetModelGravityEnabled(this->GenerateIdentity(
        nestedID, this->GetModelInfo(nestedID))))
    {
      return false;
    }
  }

  return true;
}

}
}
}
