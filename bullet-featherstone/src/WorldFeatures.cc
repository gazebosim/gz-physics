/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gz/common/Console.hh>

#include "WorldFeatures.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
void WorldFeatures::SetWorldGravity(
    const Identity &_id, const LinearVectorType &_gravity)
{
  auto worldInfo = this->ReferenceInterface<WorldInfo>(_id);
  if (worldInfo)
    worldInfo->world->setGravity(convertVec(_gravity));
}

/////////////////////////////////////////////////
WorldFeatures::LinearVectorType WorldFeatures::GetWorldGravity(
    const Identity &_id) const
{
  const auto worldInfo = this->ReferenceInterface<WorldInfo>(_id);
  if (worldInfo)
  {
    // auto world = this->ReferenceInterface<dart::simulation::World>(_id);
    return WorldFeatures::LinearVectorType(
      worldInfo->world->getGravity().x(),
      worldInfo->world->getGravity().y(),
      worldInfo->world->getGravity().z());
  }
  else
  {
    return WorldFeatures::LinearVectorType(0, 0, 0);
  }
}

/////////////////////////////////////////////////
void WorldFeatures::SetWorldSolver(const Identity &,
    const std::string &_solver)
{
  // bullet-featherstone implementation uses btMultiBodyDynamicsWorld
  // so currently only supports BT_MULTIBODY_SOLVER
  if (_solver != "MultiBodySolver")
  {
    gzwarn << "bullet-featherstone implementation currently only supports "
           << "MultiBodySolver (BT_MULTIBODY_SOLVER)" << std::endl;
  }
}

/////////////////////////////////////////////////
const std::string &WorldFeatures::GetWorldSolver(const Identity &) const
{
  // bullet-featherstone implementation uses btMultiBodyDynamicsWorld
  // so currently only supports BT_MULTIBODY_SOLVER
  static std::string solverType = "MultiBodySolver";
  return solverType;
}

/////////////////////////////////////////////////
void WorldFeatures::SetWorldSolverIterations(const Identity &_id,
    std::size_t _iterations)
{
  const auto worldInfo = this->ReferenceInterface<WorldInfo>(_id);
  if (worldInfo)
  {
    worldInfo->world->getSolverInfo().m_numIterations = _iterations;
  }
}

/////////////////////////////////////////////////
std::size_t WorldFeatures::GetWorldSolverIterations(const Identity &_id) const
{
  const auto worldInfo = this->ReferenceInterface<WorldInfo>(_id);
  if (worldInfo)
  {
    return worldInfo->world->getSolverInfo().m_numIterations;
  }
  return 0u;
}

}
}
}
