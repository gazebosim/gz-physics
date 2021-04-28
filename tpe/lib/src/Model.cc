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

#include <set>
#include <string>

#include <ignition/common/Profiler.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "Link.hh"
#include "Model.hh"

/// \brief Private data class for Model
class ignition::physics::tpelib::ModelPrivate
{
  /// \brief Canonical link id;
  public: std::size_t canonicalLinkId = kNullEntityId;

  /// \brief First inserted link id;
  public: std::size_t firstLinkId = kNullEntityId;
};

using namespace ignition;
using namespace physics;
using namespace tpelib;

//////////////////////////////////////////////////
Model::Model()
    : Entity(), dataPtr(new ModelPrivate)
{
}

//////////////////////////////////////////////////
Model::Model(std::size_t _id)
    : Entity(_id), dataPtr(new ModelPrivate)
{
}

//////////////////////////////////////////////////
Model::~Model()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Entity &Model::AddLink()
{
  std::size_t linkId = Entity::GetNextId();

  if (this->GetChildren().empty())
  {
    this->dataPtr->firstLinkId = linkId;
    this->dataPtr->canonicalLinkId = linkId;
  }

  const auto[it, success]  = this->GetChildren().insert(
      {linkId, std::make_shared<Link>(linkId)});

  it->second->SetParent(this);
  this->ChildrenChanged();
  return *it->second.get();
}

//////////////////////////////////////////////////
Entity &Model::AddModel()
{
  std::size_t modelId = Entity::GetNextId();
  const auto[it, success]  = this->GetChildren().insert(
      {modelId, std::make_shared<Model>(modelId)});

  it->second->SetParent(this);
  this->ChildrenChanged();
  return *it->second.get();
}

//////////////////////////////////////////////////
void Model::SetCanonicalLink(std::size_t linkId)
{
  this->dataPtr->canonicalLinkId = linkId;
  if (this->dataPtr->canonicalLinkId == kNullEntityId)
  {
    this->dataPtr->canonicalLinkId = this->dataPtr->firstLinkId;
  }
}

//////////////////////////////////////////////////
Entity &Model::GetCanonicalLink()
{
  // return canonical link but make sure it exists
  // todo(anyone) Prevent removal of canonical link in a model?
  Entity &linkEnt = this->GetChildById(this->dataPtr->canonicalLinkId);
  if (linkEnt.GetId() != kNullEntityId)
  {
    return linkEnt;
  }
  else
  {
    for (auto &child : this->GetChildren())
    {
      Entity childEnt = *(child.second);
      Model *nestedModel = static_cast<Model *>(&childEnt);
      if (nestedModel != nullptr)
      {
        Entity &ent = nestedModel->GetChildById(this->dataPtr->canonicalLinkId);
        if (ent.GetId() != kNullEntityId)
        {
          return ent;
        }
      }
    }
  }
  return kNullEntity;
}

//////////////////////////////////////////////////
void Model::SetLinearVelocity(const math::Vector3d &_velocity)
{
  this->linearVelocity = _velocity;
}

//////////////////////////////////////////////////
math::Vector3d Model::GetLinearVelocity() const
{
  IGN_PROFILE("tpelib::Model::GetLinearVelocity");
  return this->linearVelocity;
}

//////////////////////////////////////////////////
void Model::SetAngularVelocity(const math::Vector3d &_velocity)
{
  this->angularVelocity = _velocity;
}

//////////////////////////////////////////////////
math::Vector3d Model::GetAngularVelocity() const
{
  IGN_PROFILE("tpelib::Model::GetAngularVelocity");
  return this->angularVelocity;
}

//////////////////////////////////////////////////
void Model::UpdatePose(double _timeStep)
{
  IGN_PROFILE("tpelib::Model::UpdatePose");

  if (this->linearVelocity == math::Vector3d::Zero &&
      this->angularVelocity == math::Vector3d::Zero)
    return;

  math::Pose3d currentPose = this->GetPose();
  math::Pose3d nextPose(
    currentPose.Pos() + this->linearVelocity * _timeStep,
    currentPose.Rot().Integrate(this->angularVelocity, _timeStep));
  this->SetPose(nextPose);
}
