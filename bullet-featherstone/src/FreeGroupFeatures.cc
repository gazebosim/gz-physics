/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <memory>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForModel(
    const Identity &_modelID) const
{
  const auto *model = this->ReferenceInterface<ModelInfo>(_modelID);

  // Reject if the model has fixed base
  if (model->body->hasFixedBase())
    return this->GenerateInvalidId();

  return _modelID;
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForLink(
    const Identity &_linkID) const
{
  const auto *link = this->ReferenceInterface<LinkInfo>(_linkID);
  const auto *model = this->ReferenceInterface<ModelInfo>(link->model);
  if (model->body->hasFixedBase())
    return this->GenerateInvalidId();

  return link->model;
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupRootLink(const Identity &_groupID) const
{
  // Free groups in bullet-featherstone are always represented by ModelInfo
  const auto *model = this->ReferenceInterface<ModelInfo>(_groupID);

  // The first link entity in the model is always the root link
  const std::size_t rootID = model->linkEntityIds.front();
  return this->GenerateIdentity(rootID, this->links.at(rootID));
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldAngularVelocity(
    const Identity &_groupID, const AngularVelocity &_angularVelocity)
{
  // Free groups in bullet-featherstone are always represented by ModelInfo
  const auto *model = this->ReferenceInterface<ModelInfo>(_groupID);

  if (model != nullptr)
  {
    // Set angular velocity the each one of the joints of the model
    for (const auto& jointID : model->jointEntityIds)
    {
      auto jointInfo = this->joints[jointID];
      if (!jointInfo->motor)
      {
        auto *modelInfo = this->ReferenceInterface<ModelInfo>(jointInfo->model);
        jointInfo->motor = std::make_shared<btMultiBodyJointMotor>(
          modelInfo->body.get(),
          std::get<InternalJoint>(jointInfo->identifier).indexInBtModel,
          0,
          static_cast<btScalar>(0),
          static_cast<btScalar>(jointInfo->effort));
        auto *world = this->ReferenceInterface<WorldInfo>(modelInfo->world);
        world->world->addMultiBodyConstraint(jointInfo->motor.get());
      }

      if (jointInfo->motor)
      {
        jointInfo->motor->setVelocityTarget(
            static_cast<btScalar>(_angularVelocity[2]));
      }
    }
  }
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldLinearVelocity(
    const Identity &_groupID, const LinearVelocity &_linearVelocity)
{
  // Free groups in bullet-featherstone are always represented by ModelInfo
  const auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  // Set Base Vel
  if(model)
  {
    model->body->setBaseVel(convertVec(_linearVelocity));
  }
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
    const Identity &_groupID,
    const PoseType &_pose)
{
  const auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  model->body->setBaseWorldTransform(convertTf(_pose));
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
