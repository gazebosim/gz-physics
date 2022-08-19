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

#ifndef GZ_PHYSICS_SDF_CONSTRUCTCOLLISION_HH_
#define GZ_PHYSICS_SDF_CONSTRUCTCOLLISION_HH_

#include <sdf/Collision.hh>

#include <ignition/physics/FeatureList.hh>

namespace gz {
namespace physics {
namespace sdf {

class ConstructSdfCollision : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Link : public virtual Feature::Link<PolicyT, FeaturesT>
  {
    public: using ShapePtrType = ShapePtr<PolicyT, FeaturesT>;

    public: ShapePtrType ConstructCollision(const ::sdf::Collision &_collision);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructSdfCollision(
        const Identity &_linkID, const ::sdf::Collision &_collision) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructSdfCollision::Link<PolicyT, FeaturesT>::ConstructCollision(
    const ::sdf::Collision &_collision) -> ShapePtrType
{
  return ShapePtrType(this->pimpl,
        this->template Interface<ConstructSdfCollision>()
            ->ConstructSdfCollision(this->identity, _collision));
}

}
}
}

#endif
