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

#ifndef GZ_PHYSICS_SRC_CUSTOMFEATURES_HH
#define GZ_PHYSICS_SRC_CUSTOMFEATURES_HH

#include <gz/physics/Implements.hh>

#include <gz/physics/dartsim/World.hh>

#include "Base.hh"

namespace gz {
namespace physics {
namespace dartsim {

//! [add to list]
using CustomFeatureList = FeatureList<
  RetrieveWorld
>;
//! [add to list]

class CustomFeatures :
    public virtual Base,
    public virtual Implements3d<CustomFeatureList>
{
  public: dart::simulation::WorldPtr GetDartsimWorld(
      const Identity &_worldID) override;
};

}
}
}

#endif
