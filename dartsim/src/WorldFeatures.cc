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

#include <memory>
#include <string>

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>
#include <dart/simulation/World.hpp>

#include <ignition/common/Console.hh>

#include "WorldFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
void WorldFeatures::SetWorldCollisionDetector(
    const Identity &_id, const std::string &_collisionDetector)
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);
  auto collisionDetector =
       world->getConstraintSolver()->getCollisionDetector();
  if (_collisionDetector == "bullet")
  {
    collisionDetector = dart::collision::BulletCollisionDetector::create();
  }
  else if (_collisionDetector == "fcl")
  {
    collisionDetector = dart::collision::FCLCollisionDetector::create();
  }
  else if (_collisionDetector == "ode")
  {
    collisionDetector = dart::collision::OdeCollisionDetector::create();
  }
  else if (_collisionDetector == "dart")
  {
    collisionDetector = dart::collision::DARTCollisionDetector::create();
  }
  else
  {
    ignerr << "Collision detector [" << _collisionDetector
           << "] is not supported, defaulting to ["
           << collisionDetector->getType() << "]." << std::endl;
  }

  world->getConstraintSolver()->setCollisionDetector(collisionDetector);

  ignmsg << "Using [" << world->getConstraintSolver()->getCollisionDetector()
      ->getType() << "] collision detector" << std::endl;
}

/////////////////////////////////////////////////
const std::string &WorldFeatures::GetWorldCollisionDetector(const Identity &_id)
    const
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);
  return world->getConstraintSolver()->getCollisionDetector()->getType();
}

/////////////////////////////////////////////////
void WorldFeatures::SetWorldGravity(
    const Identity &_id, const LinearVectorType &_gravity)
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);
  world->setGravity(_gravity);
}

/////////////////////////////////////////////////
WorldFeatures::LinearVectorType WorldFeatures::GetWorldGravity(
    const Identity &_id) const
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);
  return world->getGravity();
}

/////////////////////////////////////////////////
void WorldFeatures::SetWorldSolver(const Identity &_id,
    const std::string &_solver)
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);

  auto solver =
      dynamic_cast<dart::constraint::BoxedLcpConstraintSolver *>(
      world->getConstraintSolver());

  if (!solver)
  {
    ignwarn << "Failed to cast constraint solver to [BoxedLcpConstraintSolver]"
            << std::endl;
    return;
  }

  std::shared_ptr<dart::constraint::BoxedLcpSolver> boxedSolver;
  if (_solver == "dantzig" || _solver == "DantzigBoxedLcpSolver")
  {
    boxedSolver =
        std::make_shared<dart::constraint::DantzigBoxedLcpSolver>();
  }
  else if (_solver == "pgs" || _solver == "PgsBoxedLcpSolver")
  {
    boxedSolver = std::make_shared<dart::constraint::PgsBoxedLcpSolver>();
  }
  else
  {
    ignerr << "Solver [" << _solver
           << "] is not supported, defaulting to ["
           << solver->getBoxedLcpSolver()->getType() << "]." << std::endl;
  }

  if (boxedSolver != nullptr)
    solver->setBoxedLcpSolver(boxedSolver);

  ignmsg << "Using [" << solver->getBoxedLcpSolver()->getType()
         << "] solver." << std::endl;
}

/////////////////////////////////////////////////
const std::string &WorldFeatures::GetWorldSolver(const Identity &_id) const
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);

  auto solver =
      dynamic_cast<dart::constraint::BoxedLcpConstraintSolver *>(
      world->getConstraintSolver());

  if (!solver)
  {
    static const std::string empty{""};
    return empty;
  }

  return solver->getBoxedLcpSolver()->getType();
}

}
}
}
