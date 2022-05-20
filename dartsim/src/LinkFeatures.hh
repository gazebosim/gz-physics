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

#ifndef GZ_PHYSICS_DARTSIM_SRC_LINKFEATURES_HH_
#define GZ_PHYSICS_DARTSIM_SRC_LINKFEATURES_HH_

#include <gz/physics/Link.hh>

#include "Base.hh"

namespace gz {
namespace physics {
namespace dartsim {

struct LinkFeatureList : FeatureList<
  AddLinkExternalForceTorque
> { };

class LinkFeatures :
    public virtual Base,
    public virtual Implements3d<LinkFeatureList>
{
  // ----- Add Link Force/Torque -----
  public: void AddLinkExternalForceInWorld(
      const Identity &_id,
      const LinearVectorType &_force,
      const LinearVectorType &_position) override;

  public: void AddLinkExternalTorqueInWorld(
      const Identity &_id, const AngularVectorType &_torque) override;
};

}
}
}

#endif
