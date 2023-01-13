/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DETAIL_WORLD_HH_
#define IGNITION_PHYSICS_DETAIL_WORLD_HH_

#include <string>

#include <ignition/physics/World.hh>

namespace ignition
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void CollisionDetector::World<PolicyT, FeaturesT>::SetCollisionDetector(
    const std::string &_collisionDetector)
{
  this->template Interface<CollisionDetector>()
      ->SetWorldCollisionDetector(this->identity, _collisionDetector);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
const std::string &CollisionDetector::World<PolicyT, FeaturesT>::
    GetCollisionDetector() const
{
  return this->template Interface<CollisionDetector>()
      ->GetWorldCollisionDetector(this->identity);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void Gravity::World<PolicyT, FeaturesT>::SetGravity(
    const RelativeForceType &_gravity)
{
  // Resolve to world coordinates
  auto &impl = *this->template Interface<FrameSemantics>();
  auto gravityInWorld =
      detail::Resolve(impl, _gravity, FrameID::World(), FrameID::World());

  this->template Interface<Gravity>()
      ->SetWorldGravity(this->identity, gravityInWorld);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void Gravity::World<PolicyT, FeaturesT>::SetGravity(
    const LinearVectorType &_gravity,
    const FrameID &_inCoordinatesOf)
{
  // Call SetWorldGravity directly if using world coordinates
  if (_inCoordinatesOf == FrameID::World())
  {
    this->template Interface<Gravity>()
        ->SetWorldGravity(this->identity, _gravity);
  }
  // Otherwise make a RelativeForce object and call the other API
  else
  {
    RelativeForceType gravityInRef(_inCoordinatesOf, _gravity);
    this->SetGravity(gravityInRef);
  }
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto Gravity::World<PolicyT, FeaturesT>::GetGravity(
    const FrameID &_inCoordinatesOf) const
    -> LinearVectorType
{
  // Return quickly if using world coordinates
  auto gravityInWorld = this->template Interface<Gravity>()
                            ->GetWorldGravity(this->identity);
  if (_inCoordinatesOf == FrameID::World())
  {
    return gravityInWorld;
  }

  // Otherwise resolve to proper frame
  auto &impl = *this->template Interface<FrameSemantics>();
  RelativeForceType gravityInRef(FrameID::World(), gravityInWorld);
  return detail::Resolve(impl, gravityInRef, FrameID::World(),
                         _inCoordinatesOf);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void Solver::World<PolicyT, FeaturesT>::SetSolver(
    const std::string &_solver)
{
  this->template Interface<Solver>()
      ->SetWorldSolver(this->identity, _solver);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
const std::string &Solver::World<PolicyT, FeaturesT>::
    GetSolver() const
{
  return this->template Interface<Solver>()
      ->GetWorldSolver(this->identity);
}

}  // namespace physics
}  // namespace ignition

#endif
