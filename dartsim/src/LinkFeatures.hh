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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_LINKFEATURES_HH_
#define IGNITION_PHYSICS_DARTSIM_SRC_LINKFEATURES_HH_

#include <ignition/physics/Link.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using LinkFeatureList = FeatureList<
  GetLinkForceTorque,
  SetLinkForceTorque
>;

class LinkFeatures :
    public virtual Base,
    public virtual Implements3d<LinkFeatureList>
{
  // ----- Get Link Force/Torque -----
  public: LinearVector3d GetLinkExternalForceInWorld(
      const Identity &_id) const override;

  public: AngularVector3d GetLinkExternalTorqueInWorld(
      const Identity &_id) const override;

  // ----- Set Link Force/Torque -----
  public: void SetLinkExternalForceInWorld(
      const Identity &_id, const LinearVector3d &_force) override;

  public: void SetLinkExternalTorqueInWorld(
      const Identity &_id, const AngularVector3d &_torque) override;
};

}
}
}

#endif
