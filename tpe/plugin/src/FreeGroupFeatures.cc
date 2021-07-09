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
  // if there are no links or nested models in this model, then the FreeGroup
  // functions will not work properly; need to reject this case.
  if (it->second->model->GetChildCount() == 0)
    return this->GenerateInvalidId();
  return this->GenerateIdentity(_modelID.id, it->second);
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
/// Helper function to find a model's root link by recursively searching for it
/// in nested models if the model has no links.
static tpelib::Link *FindModelRootLink(tpelib::Model *_model)
{
  if (nullptr == _model)
    return nullptr;

  // assume no canonical link for now
  if (_model->GetLinkCount() > 0)
  {
    // assume canonical link is the first link in model
    // note the canonical link of a free group is renamed to root link in
    // ign-physics4. The canonical link / root link of a free group can be
    // different from the canonical link of a model.
    // Here we treat them the same and return the model's canonical link
    return static_cast<tpelib::Link *>(&_model->GetCanonicalLink());
  }
  else
  {
    // If the model doesn't have any links, we recursively search for the root
    // link in the nested models.
    for (size_t i = 0; i < _model->GetChildCount(); ++i)
    {
      auto *rootLink = FindModelRootLink(
          static_cast<tpelib::Model *>(&_model->GetChildByIndex(i)));
      if (nullptr != rootLink)
        return rootLink;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupRootLink(const Identity &_groupID) const
{
  // assume no canonical link for now
  // assume groupID ~= modelID
  const auto modelIt = this->models.find(_groupID.id);
  if (modelIt != this->models.end() && modelIt->second != nullptr)
  {
    auto *rootLink = FindModelRootLink(modelIt->second->model);
    if (nullptr != rootLink)
    {
      auto linkPtr = std::make_shared<LinkInfo>();
      linkPtr->link = rootLink;
      return this->GenerateIdentity(rootLink->GetId(), linkPtr);
    }
    else {
      return this->GenerateInvalidId();
    }
  }
  auto linkIt = this->links.find(_groupID.id);
  if (linkIt != this->links.end())
    return this->GenerateIdentity(_groupID.id, linkIt->second);
  else
    return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
  const Identity &_groupID,
  const PoseType &_pose)
{
  // The input _pose is the target world pose for the canonical link
  // in the model! So we need to compute the world pose to set the model to
  // so that the canonical link is placed at the specified _pose
  tpelib::Link *link = nullptr;
  auto modelIt = this->models.find(_groupID.id);
  if (modelIt != this->models.end())
  {
    if (modelIt->second != nullptr)
    {
      link = FindModelRootLink(modelIt->second->model);
    }
  }
  else
  {
    auto linkIt = this->links.find(_groupID.id);
    if (linkIt != this->links.end())
    {
      // assume canonical link
      link = linkIt->second->link;
    }
  }

  if (!link)
  {
    ignwarn << "No free group with id [" << _groupID.id << "] found."
      << std::endl;
    return;
  }

  math::Pose3d targetWorldPose = math::eigen3::convert(_pose);
  math::Pose3d linkWorldPose = link->GetWorldPose();
  math::Pose3d tfChange = targetWorldPose * linkWorldPose.Inverse();

  // get top level model
  tpelib::Entity *parent = link->GetParent();
  tpelib::Entity *model = nullptr;
  while (parent && dynamic_cast<tpelib::Model *>(parent))
  {
    model = parent;
    parent = model->GetParent();
  }
  if (!model)
  {
    ignerr << "No model for free group with [" << _groupID.id << "] found."
      << std::endl;
    return;
  }

  // compute model world pose base that moves the link to its target world pose
  math::Pose3d modelWorldPose = model->GetWorldPose();
  math::Pose3d targetModelWorldPose;
  targetModelWorldPose.Pos() = targetWorldPose.Pos() - tfChange.Rot() *
     (linkWorldPose.Pos() - modelWorldPose.Pos());
  targetModelWorldPose.Rot() = tfChange.Rot() * modelWorldPose.Rot();

  // set the model world pose
  model->SetPose(targetModelWorldPose);
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldLinearVelocity(
  const Identity &_groupID,
  const LinearVelocity &_linearVelocity)
{
  auto it = this->models.find(_groupID.id);
  // set model linear velocity
  if (it != this->models.end() && it->second != nullptr)
  {
    it->second->model->SetLinearVelocity(
      math::eigen3::convert(_linearVelocity));
  }
  else
  {
    auto linkIt = this->links.find(_groupID.id);
    if (linkIt != this->links.end() && linkIt->second != nullptr)
    {
      math::Pose3d linkWorldPose = linkIt->second->link->GetWorldPose();
      linkIt->second->link->SetLinearVelocity(
        linkWorldPose.Rot().Inverse() *
        math::eigen3::convert( _linearVelocity));
    }
  }
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldAngularVelocity(
  const Identity &_groupID, const AngularVelocity &_angularVelocity)
{
  auto it = this->models.find(_groupID.id);
  // set model angular velocity
  if (it != this->models.end() && it->second != nullptr)
  {
    it->second->model->SetAngularVelocity(
      math::eigen3::convert(_angularVelocity));
  }
  else
  {
    auto linkIt = this->links.find(_groupID.id);
    if (linkIt != this->links.end() && linkIt->second != nullptr)
    {
      math::Pose3d linkWorldPose = linkIt->second->link->GetWorldPose();
      linkIt->second->link->SetAngularVelocity(
        linkWorldPose.Rot().Inverse() *
        math::eigen3::convert(_angularVelocity));
    }
  }
}
