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

#ifndef IGNITION_PHYSICS_SDF_CONSTRUCTJOINT_HH_
#define IGNITION_PHYSICS_SDF_CONSTRUCTJOINT_HH_

#include <sdf/Joint.hh>

#include <ignition/physics/FeatureList.hh>

namespace ignition {
namespace physics {
namespace sdf {

class ConstructSdfJoint : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Model : public virtual Feature::Model<PolicyT, FeaturesT>
  {
    public: using Joint = ignition::physics::Joint<PolicyT, FeaturesT>;

    public: std::unique_ptr<Joint> ConstructJoint(const ::sdf::Joint &_joint);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructSdfJoint(
        std::size_t _world, const ::sdf::Joint &_joint) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructSdfJoint::Model<PolicyT, FeaturesT>::ConstructJoint(
    const ::sdf::Joint &_joint) -> std::unique_ptr<Joint>
{
  const Identity jointID = this->template Interface<ConstructSdfJoint>()
      ->ConstructSdfJoint(this->identity, _joint);

  if (!jointID)
    return nullptr;

  return std::make_unique<Joint>(this->pimpl, jointID);
}

}
}
}

#endif
