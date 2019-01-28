/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DARTSIM_WORLD_HH_
#define IGNITION_PHYSICS_DARTSIM_WORLD_HH_

#include <dart/simulation/World.hpp>

#include <ignition/physics/FeatureList.hh>

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
class RetrieveWorld : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    /// \brief Get the underlying dartsim world for this World object.
    public: dart::simulation::WorldPtr GetDartsimWorld();
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual dart::simulation::WorldPtr GetDartsimWorld(
        const Identity &_worldID) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
dart::simulation::WorldPtr RetrieveWorld::World<PolicyT, FeaturesT>
::GetDartsimWorld()
{
  return this->template Interface<RetrieveWorld>()
      ->GetDartsimWorld(this->identity);
}

}
}
}

#endif
