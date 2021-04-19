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

#include "SimulationFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

void SimulationFeatures::WorldForwardStep(
    const Identity &_worldID,
    ForwardStep::Output & /*_h*/,
    ForwardStep::State & /*_x*/,
    const ForwardStep::Input & _u)
{
    const WorldInfoPtr &worldInfo = this->worlds.at(_worldID);

    auto *dtDur =
      _u.Query<std::chrono::steady_clock::duration>();
    std::chrono::duration<double> dt = *dtDur;
    worldInfo->world->stepSimulation(dt.count(), 1, dt.count());
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
