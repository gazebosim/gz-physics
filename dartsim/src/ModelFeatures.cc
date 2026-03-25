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
  return !this->GetModelInfo(_id)->model->isMobile();
}

/////////////////////////////////////////////////
void ModelFeatures::SetModelGravityEnabled(const Identity &_id, bool _enabled)
{
  auto modelInfo = this->GetModelInfo(_id);
  const auto &skeleton = modelInfo->model;

  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i)
  {
    skeleton->getBodyNode(i)->setGravityMode(_enabled);
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
  const auto &skeleton = this->GetModelInfo(_id)->model;
  if (skeleton->getNumBodyNodes() == 0)
    return true;
  return skeleton->getBodyNode(0)->getGravityMode();
}

}
}
}
