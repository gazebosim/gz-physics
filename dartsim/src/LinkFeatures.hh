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
  // No GetLinkState because that functionality is implemented by the 
  // LinkFrameSemantics feature
  SetLinkState
>;

class LinkFeatures :
    public virtual Base,
    public virtual Implements3d<LinkFeatureList>
{

  // ----- Set Link State -----
  public: void SetLinkLinearVelocity(
      std::size_t _id, const LinearVector3d &_vel) override;

  public: void SetLinkAngularVelocity(
      std::size_t _id, const AngularVector3d &_vel) override;

  // public: void SetLinkForce(
  //     const std::size_t _id, const std::size_t _dof,
  //     const double _value) override;

};

}
}
}

#endif
