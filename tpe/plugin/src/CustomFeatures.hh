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

#ifndef IGNITION_PHYSICS_TPE_PLUGIN_SRC_CUSTOMFEATURES_HH
#define IGNITION_PHYSICS_TPE_PLUGIN_SRC_CUSTOMFEATURES_HH

#include <memory>

#include <ignition/physics/Implements.hh>

#include "ignition/physics/tpe/plugin/src/World.hh"

#include "Base.hh"

namespace ignition {
namespace physics {
namespace tpesim {

using CustomFeatureList = FeatureList<
  RetrieveWorld
>;

class CustomFeatures :
  public virtual Base,
  public virtual Implements3d<CustomFeatureList>
{
  public: std::shared_ptr<tpe::lib::World> GetTpeLibWorld(
    const Identity &_worldID) override;
};

}
}
}

#endif
