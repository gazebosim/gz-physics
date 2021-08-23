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

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForModel(
    const Identity &_modelID) const
{
  auto model = std::get<Model*>(this->entities.at(_modelID));

  // If there are no links at all in this model, then the FreeGroup functions
  // will not work properly, so we'll just reject these cases.
  if (model->links.empty())
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
  (void) _linkID;
  /*
  const auto &link_it = this->links.find(_linkID);

  if (link_it != this->links.end() && link_it->second != nullptr)
    return this->GenerateIdentity(_linkID.id, link_it->second);
    */
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupRootLink(
    const Identity &_groupID) const
{
  (void) _groupID;
  // Todo(Lobotuerk) implement canonical links
  return this->GenerateInvalidId();
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
    auto model = std::get<Model*>(this->entities.at(_groupID));
  for (auto& link : model->links)
  {
    link->body->setCenterOfMassTransform(baseTransform);
  }
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
