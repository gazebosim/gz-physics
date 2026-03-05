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
    {
      // Zero out velocities and forces so the body doesn't drift
      // from residual momentum when gravity is disabled
      body->setLinearVelocity(btVector3(0, 0, 0));
      body->setAngularVelocity(btVector3(0, 0, 0));
      body->clearForces();
    }
    body->setActivationState(ACTIVE_TAG);
  }

  gravityStates[static_cast<std::size_t>(_groupID)] = _enabled;
}

/////////////////////////////////////////////////
bool FreeGroupFeatures::GetFreeGroupStaticState(
    const Identity &_groupID) const
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
      body->clearForces();
      body->setActivationState(DISABLE_SIMULATION);
    }
    else
    {
      body->setCollisionFlags(
          body->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
      // Restore mass/inertia from this link's own LinkInfo
      body->setMassProps(linkIt->second->mass, linkIt->second->inertia);
      body->updateInertiaTensor();
      body->setActivationState(ACTIVE_TAG);
    }
  }
}

/////////////////////////////////////////////////
bool FreeGroupFeatures::GetFreeGroupGravityEnabled(
    const Identity &_groupID) const
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
  const auto &model = this->models.at(_groupID);
  if (model->links.empty())
    return;

  // Get the current world pose of the root link (link frame, not COM frame)
  const auto rootLinkID = model->links.front();
  const auto &rootLinkInfo = this->links.at(rootLinkID);
  btTransform currentRootCOM =
    rootLinkInfo->link->getCenterOfMassTransform();

  // Root link world pose = currentRootCOM * inertialPose.Inverse()
  const auto rootInertialPose =
    gz::math::eigen3::convert(rootLinkInfo->inertialPose);
  btTransform rootInertialBt;
  rootInertialBt.setOrigin(convertVec(rootInertialPose.translation()));
  rootInertialBt.setBasis(convertMat(rootInertialPose.linear()));

  btTransform currentRootWorldPose = currentRootCOM * rootInertialBt.inverse();

  // Desired new world pose for the root link
  btTransform newRootWorldPose;
  newRootWorldPose.setOrigin(convertVec(_pose.translation()));
  newRootWorldPose.setBasis(convertMat(_pose.linear()));

  // Compute the relative transform change
  btTransform tfChange = newRootWorldPose * currentRootWorldPose.inverse();

  // Apply the transform change to all links
  for (const auto linkID : model->links)
  {
    const auto &linkInfo = this->links.at(linkID);
    auto body = linkInfo->link.get();

    // Current link world pose (link frame)
    btTransform linkInertialBt;
    const auto linkInertialPose =
      gz::math::eigen3::convert(linkInfo->inertialPose);
    linkInertialBt.setOrigin(convertVec(linkInertialPose.translation()));
    linkInertialBt.setBasis(convertMat(linkInertialPose.linear()));

    btTransform currentLinkCOM = body->getCenterOfMassTransform();
    btTransform currentLinkWorld = currentLinkCOM * linkInertialBt.inverse();

    // Apply the change to get new link world pose, then convert back to COM
    btTransform newLinkWorld = tfChange * currentLinkWorld;
    btTransform newLinkCOM = newLinkWorld * linkInertialBt;

    body->setCenterOfMassTransform(newLinkCOM);
    // Also update the motion state so Bullet doesn't reset the pose
    if (body->getMotionState())
    {
      body->getMotionState()->setWorldTransform(newLinkCOM);
    }
    body->setLinearVelocity(btVector3(0, 0, 0));
    body->setAngularVelocity(btVector3(0, 0, 0));
    body->clearForces();
    body->setActivationState(ACTIVE_TAG);
  }
}

void FreeGroupFeatures::SetFreeGroupWorldLinearVelocity(
    const Identity &_groupID,
    const LinearVelocity &_linearVelocity)
{
  auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  if (model == nullptr)
    return;

  // Do not set velocity on a static model
  if (model->fixed)
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

  // Do not set velocity on a static model
  if (model->fixed)
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
