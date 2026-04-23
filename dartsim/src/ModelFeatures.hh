/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_DARTSIM_SRC_MODELFEATURES_HH_
#define GZ_PHYSICS_DARTSIM_SRC_MODELFEATURES_HH_

#include <gz/physics/Model.hh>

#include "Base.hh"

namespace gz {
namespace physics {
namespace dartsim {

struct ModelFeatureList : FeatureList<
  ModelStaticState,
  ModelGravityEnabled
> { };

class ModelFeatures :
    public virtual Base,
    public virtual Implements3d<ModelFeatureList>
{
  // Documentation inherited
  public: void SetModelStatic(
      const Identity &_id, bool _static) override;

  // Documentation inherited
  public: bool GetModelStatic(const Identity &_id) const override;

  // Documentation inherited
  public: void SetModelGravityEnabled(
      const Identity &_id, bool _enabled) override;

  // Documentation inherited
  public: bool GetModelGravityEnabled(const Identity &_id) const override;
};

}
}
}

#endif
