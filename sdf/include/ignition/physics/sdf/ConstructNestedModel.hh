/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_SDF_CONSTRUCTNESTEDMODEL_HH_
#define IGNITION_PHYSICS_SDF_CONSTRUCTNESTEDMODEL_HH_

#include <sdf/Model.hh>

#include <ignition/physics/FeatureList.hh>

namespace ignition {
namespace physics {
namespace sdf {

class ConstructSdfNestedModel : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Model: public virtual Feature::Model<PolicyT, FeaturesT>
  {
    public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;

    public: ModelPtrType ConstructNestedModel(const ::sdf::Model &_model);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructSdfNestedModel(
        const Identity &_modelId, const ::sdf::Model &_model) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructSdfNestedModel::Model<PolicyT, FeaturesT>::ConstructNestedModel(
    const ::sdf::Model &_model) -> ModelPtrType
{
  return ModelPtrType(this->pimpl,
        this->template Interface<ConstructSdfNestedModel>()
              ->ConstructSdfNestedModel(this->identity, _model));
}

}
}
}

#endif
