/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_SDF_FINALIZEMODELS_HH_
#define IGNITION_PHYSICS_SDF_FINALIZEMODELS_HH_

#include <sdf/Model.hh>

#include <ignition/physics/FeatureList.hh>

namespace ignition {
namespace physics {
namespace sdf {

class FinalizeSdfModels : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
  {
    /// \brief This function should be called after models have been
    /// constructed.
    public: void FinalizeModels();
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual void FinalizeSdfModels(
        const Identity &_engine) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void FinalizeSdfModels::Engine<PolicyT, FeaturesT>::FinalizeModels()
{
    this->template Interface<FinalizeSdfModels>()
          ->FinalizeSdfModels(this->identity);
}

}
}
}

#endif
