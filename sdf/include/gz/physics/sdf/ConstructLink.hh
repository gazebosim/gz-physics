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

#ifndef GZ_PHYSICS_SDF_CONSTRUCTLINK_HH_
#define GZ_PHYSICS_SDF_CONSTRUCTLINK_HH_

#include <sdf/Link.hh>

#include <ignition/physics/FeatureList.hh>

namespace gz {
namespace physics {
namespace sdf {

class ConstructSdfLink : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Model : public virtual Feature::Model<PolicyT, FeaturesT>
  {
    public: using LinkPtrType = LinkPtr<PolicyT, FeaturesT>;

    public: LinkPtrType ConstructLink(const ::sdf::Link &_link);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructSdfLink(
        const Identity &_model, const ::sdf::Link &_link) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructSdfLink::Model<PolicyT, FeaturesT>::ConstructLink(
    const ::sdf::Link &_link) -> LinkPtrType
{
  return LinkPtrType(this->pimpl,
        this->template Interface<ConstructSdfLink>()
              ->ConstructSdfLink(this->identity, _link));
}

}
}
}

#endif
