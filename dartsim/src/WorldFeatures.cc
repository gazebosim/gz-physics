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

// Suppressing a -Wreturn-type warning in octomap
// https://github.com/OctoMap/octomap/issues/431
#include <gz/utils/SuppressWarning.hh>
#ifndef _MSC_VER
DETAIL_GZ_UTILS_BEGIN_WARNING_SUPPRESSION("-Wreturn-type")
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
DETAIL_GZ_UTILS_WARN_RESUME
#endif  // _MSC_VER

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>
#include <dart/simulation/World.hpp>

#include <gz/common/Console.hh>

#include "GzCollisionDetector.hh"
#include "WorldFeatures.hh"


namespace gz {
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
    collisionDetector = dart::collision::GzBulletCollisionDetector::create();
  }
  else if (_collisionDetector == "fcl")
  {
    collisionDetector = dart::collision::FCLCollisionDetector::create();
  }
  else if (_collisionDetector == "ode")
  {
    collisionDetector = dart::collision::GzOdeCollisionDetector::create();
  }
  else if (_collisionDetector == "dart")
  {
    collisionDetector = dart::collision::DARTCollisionDetector::create();
  }
  else
  {
    gzerr << "Collision detector [" << _collisionDetector
           << "] is not supported, defaulting to ["
           << collisionDetector->getType() << "]." << std::endl;
  }

  world->getConstraintSolver()->setCollisionDetector(collisionDetector);

  gzmsg << "Using [" << world->getConstraintSolver()->getCollisionDetector()
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
void WorldFeatures::SetWorldCollisionPairMaxContacts(
    const Identity &_id, std::size_t _maxContacts)
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);
  auto collisionDetector =
    world->getConstraintSolver()->getCollisionDetector();

 auto gzCollisionDetector =
    std::dynamic_pointer_cast<dart::collision::GzCollisionDetector>(
    collisionDetector);
  if (gzCollisionDetector)
  {
    gzCollisionDetector->SetCollisionPairMaxContacts(_maxContacts);
  }
  else
  {
    gzwarn << "Currently max contacts feature is only supported by the "
           << "bullet and ode collision detector in dartsim." << std::endl;
  }
}

/////////////////////////////////////////////////
std::size_t WorldFeatures::GetWorldCollisionPairMaxContacts(
    const Identity &_id) const
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);
  auto collisionDetector =
    world->getConstraintSolver()->getCollisionDetector();

 auto gzCollisionDetector =
    std::dynamic_pointer_cast<dart::collision::GzCollisionDetector>(
    collisionDetector);
  if (gzCollisionDetector)
  {
    return gzCollisionDetector->GetCollisionPairMaxContacts();
  }
  else
  {
    gzwarn << "Currently max contacts feature is only supported by the "
           << "bullet and ode collision detector in dartsim." << std::endl;
  }

  return 0u;
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
    gzwarn << "Failed to cast constraint solver to [BoxedLcpConstraintSolver]"
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
    gzerr << "Solver [" << _solver
           << "] is not supported, defaulting to ["
           << solver->getBoxedLcpSolver()->getType() << "]." << std::endl;
  }

  if (boxedSolver != nullptr)
    solver->setBoxedLcpSolver(boxedSolver);

  gzmsg << "Using [" << solver->getBoxedLcpSolver()->getType()
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

/////////////////////////////////////////////////
void WorldFeatures::SetWorldSolverIterations(const Identity &_id,
    std::size_t _iterations)
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);

  auto solver =
      dynamic_cast<dart::constraint::BoxedLcpConstraintSolver *>(
      world->getConstraintSolver());

  if (!solver)
  {
    gzwarn << "Failed to cast constraint solver to [BoxedLcpConstraintSolver]"
            << std::endl;
    return;
  }

  auto boxedLcpSolver = solver->getBoxedLcpSolver();
  if (boxedLcpSolver->getType() == "PgsBoxedLcpSolver")
  {
    auto pgsSolver =
        std::dynamic_pointer_cast<const dart::constraint::PgsBoxedLcpSolver>(
        boxedLcpSolver);

    // getBoxedLcpSolver returns a const shared_ptr so in order to update
    // the solver parameters, we need to create a new boxed lcp solver with
    // updated values and set it back to the constraint solver.
    auto newPgsSolver = std::make_shared<dart::constraint::PgsBoxedLcpSolver>();
    auto option =  pgsSolver->getOption();
    option.mMaxIteration = _iterations;
    newPgsSolver->setOption(option);
    solver->setBoxedLcpSolver(newPgsSolver);
  }
  else
  {
    gzwarn << "Solver iterations is only supported for [PgsBoxedLcpSolver]"
            << std::endl;
  }
}

/////////////////////////////////////////////////
std::size_t WorldFeatures::GetWorldSolverIterations(const Identity &_id) const
{
  auto world = this->ReferenceInterface<dart::simulation::World>(_id);

  auto solver =
      dynamic_cast<dart::constraint::BoxedLcpConstraintSolver *>(
      world->getConstraintSolver());

  if (!solver)
  {
    gzwarn << "Failed to cast constraint solver to [BoxedLcpConstraintSolver]"
            << std::endl;
    return 0u;
  }

  auto boxedLcpSolver = solver->getBoxedLcpSolver();
  if (boxedLcpSolver->getType() == "PgsBoxedLcpSolver")
  {
    auto pgsSolver =
        std::dynamic_pointer_cast<const dart::constraint::PgsBoxedLcpSolver>(
        boxedLcpSolver);
    return pgsSolver->getOption().mMaxIteration;
  }
  else
  {
    gzwarn << "Solver iterations is only supported for [PgsBoxedLcpSolver]"
            << std::endl;
  }
  return 0u;
}

}
}
}
