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

#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "Link.hh"
#include "Model.hh"

using namespace ignition;
using namespace physics;
using namespace tpelib;

//////////////////////////////////////////////////
Model::Model() : Entity()
{
}

//////////////////////////////////////////////////
Model::Model(std::size_t _id) : Entity(_id)
{
}

//////////////////////////////////////////////////
Entity &Model::AddLink()
{
  std::size_t linkId = Entity::GetNextId();
  const auto[it, success]  = this->GetChildren().insert(
      {linkId, std::make_shared<Link>(linkId)});

  it->second->SetParent(this);
  this->ChildrenChanged();
  return *it->second.get();
}

//////////////////////////////////////////////////
Entity &Model::GetCanonicalLink()
{
  // return first link as canonical link
  return *this->GetChildren().begin()->second;
}


//////////////////////////////////////////////////
void Model::SetLinearVelocity(const math::Vector3d _velocity)
{
  this->linearVelocity = _velocity;
}

//////////////////////////////////////////////////
math::Vector3d Model::GetLinearVelocity() const
{
  return this->linearVelocity;
}

//////////////////////////////////////////////////
void Model::SetAngularVelocity(const math::Vector3d _velocity)
{
  this->angularVelocity = _velocity;
}

//////////////////////////////////////////////////
math::Vector3d Model::GetAngularVelocity() const
{
  return this->angularVelocity;
}

//////////////////////////////////////////////////
void Model::UpdatePose(
  const double _timeStep,
  const math::Vector3d _linearVelocity,
  const math::Vector3d _angularVelocity)
{
  math::Pose3d currentPose = this->GetPose();
  math::Pose3d nextPose(
    currentPose.Pos() + _linearVelocity * _timeStep,
    currentPose.Rot().Integrate(_angularVelocity, _timeStep));
  this->SetPose(nextPose);
}
