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

  /// \brief Links in the model
  public: std::vector<std::size_t> linkIds;

  /// \brief Nested models
  public: std::vector<std::size_t> nestedModelIds;
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

  if (this->GetLinkCount() == 0)
  {
    this->dataPtr->firstLinkId = linkId;
    this->dataPtr->canonicalLinkId = linkId;
  }

  const auto[it, success]  = this->GetChildren().insert(
      {linkId, std::make_shared<Link>(linkId)});
  this->dataPtr->linkIds.push_back(linkId);

  it->second->SetParent(this);
  this->ChildrenChanged();
  return *it->second.get();
}

//////////////////////////////////////////////////
std::size_t Model::GetLinkCount() const
{
  return this->dataPtr->linkIds.size();
}

//////////////////////////////////////////////////
Entity &Model::AddModel()
{
  std::size_t modelId = Entity::GetNextId();
  const auto[it, success]  = this->GetChildren().insert(
      {modelId, std::make_shared<Model>(modelId)});
  this->dataPtr->nestedModelIds.push_back(modelId);

  it->second->SetParent(this);
  this->ChildrenChanged();
  return *it->second.get();
}

//////////////////////////////////////////////////
std::size_t Model::GetModelCount() const
{
  return this->dataPtr->nestedModelIds.size();
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

//////////////////////////////////////////////////
bool Model::RemoveModelById(std::size_t _id)
{
  auto it = std::find(this->dataPtr->nestedModelIds.begin(),
                      this->dataPtr->nestedModelIds.end(), _id);
  if (it != this->dataPtr->nestedModelIds.end())
  {
    this->dataPtr->nestedModelIds.erase(it);
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool Model::RemoveLinkById(std::size_t _id)
{
  auto it = std::find(this->dataPtr->linkIds.begin(),
                      this->dataPtr->linkIds.end(), _id);
  if (it != this->dataPtr->linkIds.end())
  {
    this->dataPtr->linkIds.erase(it);
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool Model::RemoveChildById(std::size_t _id)
{
  Entity &ent = this->GetChildById(_id);
  return this->RemoveChildEntityBasedOnType(&ent);
}

//////////////////////////////////////////////////
bool Model::RemoveChildByName(const std::string &_name)
{
  Entity &ent = this->GetChildByName(_name);
  return this->RemoveChildEntityBasedOnType(&ent);
}

//////////////////////////////////////////////////
bool Model::RemoveChildEntityBasedOnType(const Entity *_ent)
{
  if (nullptr == _ent)
    return false;

  bool result = true;
  if (nullptr != dynamic_cast<const Model *>(_ent))
  {
    result &= this->RemoveModelById(_ent->GetId());
  }
  else
  {
    result &= this->RemoveLinkById(_ent->GetId());
  }
  result &= Entity::RemoveChildById(_ent->GetId());
  return result;
}
