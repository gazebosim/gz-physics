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
  const bool wasStatic = !skeleton->isMobile();

  // In DART, isMobile=false means the skeleton is fixed (static).
  skeleton->setMobile(!_static);

  // When transitioning from static (kinematic) to dynamic, zero out any
  // velocities and accelerations that DART may have stored on the skeleton
  // while it was kinematic. Without this the stored state is released as
  // momentum when the body becomes dynamic.
  if (wasStatic && !_static)
  {
    skeleton->resetVelocities();
    skeleton->resetAccelerations();
    skeleton->clearExternalForces();
    skeleton->clearInternalForces();
  }

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
void ModelFeatures::SetModelGravityEnabled(const Identity &_id, bool _enabled)
{
  auto modelInfo = this->GetModelInfo(_id);
  if (!modelInfo || !modelInfo->model)
    return;
  for (const auto &linkInfo : modelInfo->links)
  {
    if (!linkInfo || !linkInfo->link)
      continue;
    // Added-mass links must keep DART gravity disabled: gravity is applied
    // manually as F=ma each step. Let SetLinkAddedMass own that flag.
    if (linkInfo->inertial.has_value() &&
        linkInfo->inertial->FluidAddedMass().has_value())
      continue;
    linkInfo->link->setGravityMode(_enabled);
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
    if (!linkInfo || !linkInfo->link)
      continue;
    // Added-mass links have DART gravity forcibly disabled; exclude them so
    // their internal state doesn't pollute the user-visible gravity flag.
    if (linkInfo->inertial.has_value() &&
        linkInfo->inertial->FluidAddedMass().has_value())
      continue;
    if (!linkInfo->link->getGravityMode())
      return false;
  }
  return true;
}

}
}
}
