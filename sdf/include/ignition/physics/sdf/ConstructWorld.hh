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

#ifndef IGNITION_PHYSICS_SDF_CONSTRUCTWORLD_HH_
#define IGNITION_PHYSICS_SDF_CONSTRUCTWORLD_HH_

#include <sdf/World.hh>

#include <ignition/physics/FeatureList.hh>

namespace ignition {
namespace physics {
namespace sdf {

class ConstructSdfWorld : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
  {
    public: using World = ignition::physics::World<PolicyT, FeaturesT>;

    public: std::unique_ptr<World> ConstructWorld(const ::sdf::World &_world);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructSdfWorld(
        std::size_t _engine, const ::sdf::World &_world) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructSdfWorld::Engine<PolicyT, FeaturesT>::ConstructWorld(
    const ::sdf::World &_world) -> std::unique_ptr<World>
{
  const Identity worldID = this->template Interface<ConstructSdfWorld>()
      ->ConstructSdfWorld(this->identity, _world);

  if (!worldID)
    return nullptr;

  return std::make_unique<World>(this->pimpl, worldID);
}

}
}
}

#endif
