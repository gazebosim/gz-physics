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

#include "EntityManagementFeatures.hh"

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "btBulletDynamicsCommon.h"

#include <string>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetEngineName(
    const Identity &/*_engineID*/) const
{
  static const std::string engineName = "bullet";
  return engineName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetEngineIndex(
    const Identity &/*_engineID*/) const
{
  return 0;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  // Create bullet empty multibody dynamics world
  auto collisionConfiguration = new btDefaultCollisionConfiguration();
  auto dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  auto solver = new btMultiBodyConstraintSolver;
  auto world = new btMultiBodyDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

  world->getSolverInfo().m_globalCfm = 1e-3;

  return this->AddWorld({world, _name, collisionConfiguration, dispatcher, broadphase, solver});
}


}
}
}
