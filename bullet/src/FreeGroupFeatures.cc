/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <unordered_map>

namespace gz {
namespace physics {
namespace bullet {

namespace
{
std::unordered_map<std::size_t, bool> gravityStates;
}  // namespace

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForModel(
    const Identity &_modelID) const
{
  const auto &model = this->models.at(_modelID);

  // If there are no links at all in this model, then the FreeGroup functions
  // will not work properly, so we'll just reject these cases.
  if (model->links.size() == 0)
    return this->GenerateInvalidId();

  // Reject also if the model has fixed base
  if (model->fixed)
    return this->GenerateInvalidId();

  return _modelID;
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForLink(
    const Identity &_linkID) const
{
  const auto &link_it = this->links.find(_linkID);

  if (link_it != this->links.end() && link_it->second != nullptr)
    return this->GenerateIdentity(_linkID.id, link_it->second);
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupRootLink(
    const Identity &_groupID) const
{
  const auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  if (model == nullptr || model->links.empty())
    return this->GenerateInvalidId();

  const auto rootLinkID = model->links.front();
  auto linkIt = this->links.find(rootLinkID);
  if (linkIt == this->links.end() || linkIt->second == nullptr)
    return this->GenerateInvalidId();

  return this->GenerateIdentity(rootLinkID, linkIt->second);
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupGravityEnabled(
    const Identity &_groupID, bool _enabled)
{
  auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  if (model == nullptr)
    return;

  btVector3 gravity(0, 0, 0);
  if (_enabled)
  {
    auto worldIt = this->worlds.find(static_cast<std::size_t>(model->world));
    if (worldIt != this->worlds.end() && worldIt->second &&
        worldIt->second->world)
    {
      gravity = worldIt->second->world->getGravity();
    }
  }

  for (const auto linkID : model->links)
  {
    auto linkIt = this->links.find(linkID);
    if (linkIt == this->links.end() || !linkIt->second || !linkIt->second->link)
      continue;

    auto body = linkIt->second->link.get();
    body->setGravity(gravity);
    if (!_enabled)
      body->setActivationState(ACTIVE_TAG);
  }

  gravityStates[static_cast<std::size_t>(_groupID)] = _enabled;
}

/////////////////////////////////////////////////
bool FreeGroupFeatures::GetFreeGroupStaticState(
    const Identity &_groupID)
{
  auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  if (model == nullptr)
    return false;

  return model->fixed;
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupStaticState(
    const Identity &_groupID,
    bool _enabled)
{
  auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  if (model == nullptr)
    return;

  model->fixed = _enabled;
  for (const auto linkID : model->links)
  {
    auto linkIt = this->links.find(linkID);
    if (linkIt == this->links.end() || !linkIt->second || !linkIt->second->link)
      continue;

    auto body = linkIt->second->link.get();
    if (_enabled)
    {
      body->setMassProps(0, btVector3(0, 0, 0));
      body->setCollisionFlags(
          body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
      body->setLinearVelocity(btVector3(0, 0, 0));
      body->setAngularVelocity(btVector3(0, 0, 0));
      body->setActivationState(DISABLE_SIMULATION);
    }
    else
    {
      body->setCollisionFlags(
          body->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
      auto *linkInfo = this->ReferenceInterface<LinkInfo>(_groupID);
      if (linkInfo != nullptr)
        body->setMassProps(linkInfo->mass, linkInfo->inertia);
      body->setActivationState(ACTIVE_TAG);
    }
  }
}

/////////////////////////////////////////////////
bool FreeGroupFeatures::GetFreeGroupGravityEnabled(
    const Identity &_groupID)
{
  auto it = gravityStates.find(static_cast<std::size_t>(_groupID));
  if (it != gravityStates.end())
    return it->second;

  return true;
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
    const Identity &_groupID,
    const PoseType &_pose)
{
  // Convert pose
  const auto poseTranslation = _pose.translation();
  const auto poseLinear = _pose.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // Set base transform
  const auto &model = this->models.at(_groupID);
  for (auto link : model->links)
  {
    this->links.at(link)->link->setCenterOfMassTransform(baseTransform);
  }
}

void FreeGroupFeatures::SetFreeGroupWorldLinearVelocity(
    const Identity &_groupID,
    const LinearVelocity &_linearVelocity)
{
  auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  if (model == nullptr)
    return;

  const btVector3 velocity = convertVec(_linearVelocity);
  for (const auto linkID : model->links)
  {
    auto linkIt = this->links.find(linkID);
    if (linkIt == this->links.end() || !linkIt->second || !linkIt->second->link)
      continue;

    auto body = linkIt->second->link.get();
    body->setLinearVelocity(velocity);
    body->setActivationState(ACTIVE_TAG);
  }
}

void FreeGroupFeatures::SetFreeGroupWorldAngularVelocity(
    const Identity &_groupID,
    const AngularVelocity &_angularVelocity)
{
  auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  if (model == nullptr)
    return;

  const btVector3 angular = convertVec(_angularVelocity);
  for (const auto linkID : model->links)
  {
    auto linkIt = this->links.find(linkID);
    if (linkIt == this->links.end() || !linkIt->second || !linkIt->second->link)
      continue;

    auto body = linkIt->second->link.get();
    body->setAngularVelocity(angular);
    body->setActivationState(ACTIVE_TAG);
  }
}

}  // namespace bullet
}  // namespace physics
}  // namespace gz
