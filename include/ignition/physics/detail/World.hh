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

#ifndef IGNITION_PHYSICS_DETAIL_LINK_HH_
#define IGNITION_PHYSICS_DETAIL_LINK_HH_

#include <ignition/physics/World.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/RelativeQuantity.hh>

namespace ignition
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void PhysicsOptions::World<PolicyT, FeaturesT>::SetCollisionDetector(
    const std::string &_collisionDetector)
{
  this->template Interface<PhysicsOptions>()
      ->SetCollisionDetector(this->identity, _collisionDetector);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
const std::string &PhysicsOptions::World<PolicyT, FeaturesT>::
    GetCollisionDetector() const
{
  return this->template Interface<PhysicsOptions>()
      ->GetCollisionDetector(this->identity);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void PhysicsOptions::World<PolicyT, FeaturesT>::SetSolver(
    const std::string &_solver)
{
  this->template Interface<PhysicsOptions>()
      ->SetSolver(this->identity, _solver);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
const std::string &PhysicsOptions::World<PolicyT, FeaturesT>::
    GetSolver() const
{
  return this->template Interface<PhysicsOptions>()
      ->GetSolver(this->identity);
}

}  // namespace physics
}  // namespace ignition

#endif
