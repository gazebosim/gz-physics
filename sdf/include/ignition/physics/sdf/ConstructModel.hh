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

#ifndef IGNITION_PHYSICS_SDF_CONSTRUCTMODEL_HH_
#define IGNITION_PHYSICS_SDF_CONSTRUCTMODEL_HH_

#include <sdf/Model.hh>

#include <ignition/physics/FeatureList.hh>

namespace ignition {
namespace physics {
namespace sdf {

class ConstructSdfModel : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using Model = ignition::physics::Model<PolicyT, FeaturesT>;

    public: std::unique_ptr<Model> ConstructModel(const ::sdf::Model &_model);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructSdfModel(
        std::size_t _world, const ::sdf::Model &_model) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructSdfModel::World<PolicyT, FeaturesT>::ConstructModel(
    const ::sdf::Model &_model) -> std::unique_ptr<Model>
{
  const Identity modelID = this->template Interface<ConstructSdfModel>()
      ->ConstructSdfModel(this->identity, _model);

  if (!modelID)
    return nullptr;

  return std::make_unique<Model>(this->pimpl, modelID);
}

}
}
}

#endif
