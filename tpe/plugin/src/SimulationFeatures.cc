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

#include "SimulationFeatures.hh"

#include <ignition/common/Console.hh>
#include "ignition/common/Profiler.hh"

using namespace ignition;
using namespace physics;
using namespace tpeplugin;

void SimulationFeatures::WorldForwardStep(
  const Identity &_worldID,
  ForwardStep::Output & /*_h*/,
  ForwardStep::State & /*_x*/,
  const ForwardStep::Input & _u)
{
  auto it = this->worlds.find(_worldID);
  if (it == this->worlds.end())
  {
    ignerr << "World with id ["
      << _worldID.id
      << "] not found."
      << std::endl;
  }
  std::shared_ptr<tpelib::World> world = it->second->world;
  auto *dtDur = 
    _u.Query<std::chrono::steady_clock::duration>();
  const double tol = 1e-6;
  if (dtDur)
  {
    std::chrono::duration<double> dt = *dtDur;
    if (std::fabs(dt.count() - world->GetTimeStep()) > tol)
    {
      world->SetTimeStep(dt.count());
      igndbg << "Simulation timestep set to: "
        << world->GetTimeStep()
        << std::endl;
    }
  }
  world->Step();
}

std::vector<SimulationFeatures::ContactInternal>
SimulationFeatures::GetContactsFromLastStep(const Identity &_worldID) const
{
  // TODO(claireyywang): Implement contact points after collision detection is added
  std::vector<SimulationFeatures::ContactInternal> outContacts;
  std::shared_ptr<tpelib::World> world = this->worlds.at(_worldID)->world;
  return outContacts;
}
