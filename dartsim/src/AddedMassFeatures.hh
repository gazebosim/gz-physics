/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_DARTSIM_SRC_ADDEDMASSFEATURES_HH_
#define GZ_PHYSICS_DARTSIM_SRC_ADDEDMASSFEATURES_HH_

#include <gz/physics/AddedMass.hh>

#include "Base.hh"

namespace gz::physics::dartsim
{

struct AddedMassFeatureList: FeatureList<
  AddedMass
> { };

class AddedMassFeatures :
    public virtual Base,
    public virtual Implements3d<AddedMassFeatureList>
{
  public: void SetLinkAddedMass(const Identity &_link,
                                const gz::math::Matrix6d &_addedMass) override;

  public: gz::math::Matrix6d GetLinkAddedMass(
              const Identity &_link) const override;
};
}  // namespace gz::physics::dartsim

#endif  // GZ_PHYSICS_DARTSIM_SRC_ADDEDMASSFEATURES_HH_
