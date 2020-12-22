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

#include "Collision.hh"
#include "Link.hh"

using namespace ignition;
using namespace physics;
using namespace tpelib;

//////////////////////////////////////////////////
Link::Link() : Entity()
{
}

//////////////////////////////////////////////////
Link::Link(std::size_t _id) : Entity(_id)
{
}

//////////////////////////////////////////////////
Entity &Link::AddCollision()
{
  std::size_t collisionId = Entity::GetNextId();
  const auto[it, success] = this->GetChildren().insert(
    {collisionId, std::make_shared<Collision>(collisionId)});
  it->second->SetParent(this);
  this->ChildrenChanged();
  return *it->second.get();
}

//////////////////////////////////////////////////
void Link::SetLinearVelocity(const &math::Vector3d _velocity)
{
  this->linearVelocity = _velocity;
}

//////////////////////////////////////////////////
math::Vector3d Link::GetLinearVelocity() const
{
  return this->linearVelocity;
}

//////////////////////////////////////////////////
void Link::SetAngularVelocity(const &math::Vector3d _velocity)
{
  this->angularVelocity = _velocity;
}

//////////////////////////////////////////////////
math::Vector3d Link::GetAngularVelocity() const
{
  return this->angularVelocity;
}

//////////////////////////////////////////////////
void Link::UpdatePose(
  const double _timeStep,
  const math::Vector3d _linearVelocity,
  const math::Vector3d _angularVelocity)
{
  if (_linearVelocity == math::Vector3d::Zero &&
      _angularVelocity == math::Vector3d::Zero)
    return;

  math::Pose3d currentPose = this->GetPose();
  math::Pose3d nextPose(
    currentPose.Pos() + _linearVelocity * _timeStep,
    currentPose.Rot().Integrate(_angularVelocity, _timeStep));
  this->SetPose(nextPose);
}
