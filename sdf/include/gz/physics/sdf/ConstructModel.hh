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

#ifndef GZ_PHYSICS_SDF_CONSTRUCTMODEL_HH_
#define GZ_PHYSICS_SDF_CONSTRUCTMODEL_HH_

#include <sdf/Model.hh>

#include <ignition/physics/FeatureList.hh>

namespace gz {
namespace physics {
namespace sdf {

/// \brief Construct a model entity from an sdf::Model DOM object. This
/// feature is limited to constructing models that have no nested models.
class ConstructSdfModel : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;

    public: ModelPtrType ConstructModel(const ::sdf::Model &_model);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructSdfModel(
        const Identity &_world, const ::sdf::Model &_model) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructSdfModel::World<PolicyT, FeaturesT>::ConstructModel(
    const ::sdf::Model &_model) -> ModelPtrType
{
  return ModelPtrType(this->pimpl,
        this->template Interface<ConstructSdfModel>()
              ->ConstructSdfModel(this->identity, _model));
}
}
}
}

#endif
