/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <Eigen/Geometry>

#include <ignition/common/Console.hh>

#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/math/Pose3.hh>

#include "EntityManagementFeatures.hh"
#include "FreeGroupFeatures.hh"

using namespace ignition;
using namespace physics;
using namespace tpeplugin;

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForModel(
  const Identity &_modelID) const
{
  auto it = this->models.find(_modelID.id);
  if (it == this->models.end() || it->second == nullptr)
    return this->GenerateInvalidId();
  auto modelPtr = it->second;
  // if there are no links in this model, then the FreeGroup functions
  // will not work properly; need to reject this case.
  if (modelPtr->model->GetChildCount() == 0)
    return this->GenerateInvalidId();
  return _modelID;
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForLink(
  const Identity &_linkID) const
{
  auto it = this->links.find(_linkID.id);
  if (it != this->links.end() && it->second != nullptr)
    return this->GenerateIdentity(_linkID.id, it->second);
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupCanonicalLink(
  const Identity &_groupID) const
{
  // assume no canonical link for now
  // assume groupID ~= modelID
  const auto modelIt = this->models.find(_groupID.id);
  if (modelIt != this->models.end() && modelIt->second != nullptr)
  {
    // assume canonical link is the first link in model
    tpelib::Entity &link = modelIt->second->model->GetCanonicalLink();
    auto linkPtr = std::make_shared<LinkInfo>();
    linkPtr->link = static_cast<tpelib::Link *>(&link);
    return this->GenerateIdentity(link.GetId(), linkPtr);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
  const Identity &_groupID,
  const PoseType &_pose)
{
  auto modelIt = this->models.find(_groupID.id);
  auto linkIt = this->links.find(_groupID.id);
  if (modelIt != this->models.end())
  {
    if (modelIt->second != nullptr)
      modelIt->second->model->SetPose(math::eigen3::convert(_pose));
  }
  else if (linkIt != this->links.end())
  {
    if (linkIt->second != nullptr)
      linkIt->second->link->SetPose(math::eigen3::convert(_pose));
  }
  else
  {
    ignwarn << "No free group with id [" << _groupID.id << "] found."
      << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldLinearVelocity(
  const Identity &_groupID,
  const LinearVelocity &_linearVelocity)
{
  // assume no canonical link for now
  // assume groupID ~= modelID
  auto it = this->models.find(_groupID.id);
  // set model linear velocity
  if (it != this->models.end() && it->second != nullptr)
    it->second->model->SetLinearVelocity(
      math::eigen3::convert(_linearVelocity));
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldAngularVelocity(
  const Identity &_groupID, const AngularVelocity &_angularVelocity)
{
  // assume no canonical link for now
  // assume groupID ~= modelID
  auto it = this->models.find(_groupID.id);
  // set model angular velocity
  if (it != this->models.end() && it->second != nullptr)
    it->second->model->SetAngularVelocity(
      math::eigen3::convert(_angularVelocity));
}
