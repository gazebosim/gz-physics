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

#ifndef GZ_PHYSICS_ADDED_MASS_HH_
#define GZ_PHYSICS_ADDED_MASS_HH_

#include <gz/math/Matrix6.hh>

#include <gz/physics/FeatureList.hh>


namespace gz::physics
{

class AddedMass: public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Link : public virtual Feature::Link<PolicyT, FeaturesT>
  {
    public: void SetAddedMass(const gz::math::Matrix6d &_addedMass);
    public: gz::math::Matrix6d GetAddedMass() const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual void SetLinkAddedMass(const Identity &_link,
                const gz::math::Matrix6d &_addedMass) = 0;
    public: virtual gz::math::Matrix6d GetLinkAddedMass(
                const Identity &_link) const = 0;
  };
};

template <typename PolicyT, typename FeaturesT>
auto AddedMass::Link<PolicyT, FeaturesT>::SetAddedMass(
    const gz::math::Matrix6d &_addedMass) -> void
{
  this->template Interface<AddedMass>()
    ->SetLinkAddedMass(this->identity, _addedMass);
}
template <typename PolicyT, typename FeaturesT>

auto AddedMass::Link<PolicyT, FeaturesT>::GetAddedMass(
) const -> gz::math::Matrix6d
{
  return this->template Interface<AddedMass>()
              ->GetLinkAddedMass(this->identity);
}

}  // namespace gz::physics

#endif  // GZ_PHYSICS_ADDED_MASS_HH_)
