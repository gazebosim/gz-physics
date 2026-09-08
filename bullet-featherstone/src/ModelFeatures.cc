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

#include <gz/common/Profiler.hh>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
void ModelFeatures::SetModelStatic(const Identity &_id, bool _static)
{
  GZ_PROFILE("ModelFeatures::SetModelStatic");
  auto *modelInfo = this->ReferenceInterface<ModelInfo>(_id);
  if (!modelInfo)
    return;

  if (!modelInfo->body)
  {
    gzerr << "Error setting model static state. Model is missing a btMultiBody."
          << std::endl;
  }

  // Zero out velocities and forces
  modelInfo->body->setBaseVel(btVector3(0, 0, 0));
  modelInfo->body->setBaseOmega(btVector3(0, 0, 0));
  modelInfo->body->clearForcesAndTorques();
  modelInfo->body->clearConstraintForces();
  modelInfo->body->clearVelocities();

  for (const auto linkID : modelInfo->linkEntityIds)
  {
    auto linkIt = this->links.find(linkID);
    if (linkIt == this->links.end())
      continue;

    auto *linkInfo = linkIt->second.get();
    if (_static)
    {
      // Make dynamic links static collision objects
      makeColliderStatic(linkInfo);
      // Explicitly promote this link to become permanent static
      linkInfo->isStaticOrFixed = true;
    }
    else
    {
      // Reset flag to allow restore back to dynamic status
      linkInfo->isStaticOrFixed = false;
      makeColliderDynamic(linkInfo);
    }
  }

  if (!_static)
  {
    modelInfo->body->wakeUp();
  }

  // Apply recursively to nested models
  for (const auto nestedID : modelInfo->nestedModelEntityIds)
  {
    auto nestedIt = this->models.find(nestedID);
    if (nestedIt != this->models.end())
    {
      this->SetModelStatic(
          this->GenerateIdentity(nestedID, nestedIt->second), _static);
    }
  }
}

/////////////////////////////////////////////////
bool ModelFeatures::GetModelStatic(const Identity &_id) const
{
  GZ_PROFILE("ModelFeatures::GetModelStatic");
  const auto *modelInfo = this->ReferenceInterface<ModelInfo>(_id);
  if (!modelInfo)
    return false;

  // Check if first link is static as typical representation of whole
  // model state
  if (!modelInfo->linkEntityIds.empty())
  {
    const auto linkID = modelInfo->linkEntityIds.front();
    auto linkIt = this->links.find(linkID);
    if (linkIt != this->links.end())
    {
      return linkIt->second->isStaticOrFixed;
    }
  }

  return false;
}

}
}
}
