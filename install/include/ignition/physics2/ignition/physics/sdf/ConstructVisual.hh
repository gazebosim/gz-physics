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

#ifndef IGNITION_PHYSICS_SDF_CONSTRUCTVISUAL_HH_
#define IGNITION_PHYSICS_SDF_CONSTRUCTVISUAL_HH_

#include <sdf/Visual.hh>

#include <ignition/physics/FeatureList.hh>

namespace ignition {
namespace physics {
namespace sdf {

class ConstructSdfVisual : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Link : public virtual Feature::Link<PolicyT, FeaturesT>
  {
    // TODO(MXG): Return a Shape type instead of a bool once we have shape
    // features in the core ign-physics library.
    public: bool ConstructVisual(const ::sdf::Visual &_visual);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructSdfVisual(
        const Identity &_linkID, const ::sdf::Visual &_visual) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructSdfVisual::Link<PolicyT, FeaturesT>::ConstructVisual(
    const ::sdf::Visual &_visual) -> bool
{
  return static_cast<bool>(
        this->template Interface<ConstructSdfVisual>()
            ->ConstructSdfVisual(this->identity, _visual));
}

}
}
}

#endif
