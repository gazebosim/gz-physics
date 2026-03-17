/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_MUJOCO_SRC_SDFFEATURES_HH_
#define GZ_PHYSICS_MUJOCO_SRC_SDFFEATURES_HH_

#include <gz/physics/Implements.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include "Base.hh"
#include "EntityManagementFeatures.hh"
#include "gz/physics/detail/Identity.hh"

namespace gz {
namespace physics {
namespace mujoco {

struct SDFFeatureList : FeatureList<
  sdf::ConstructSdfWorld,
  sdf::ConstructSdfModel
> { };

class SDFFeatures :
    public virtual Base,
    public virtual EntityManagementFeatures,
    public virtual Implements3d<SDFFeatureList>
{
  public: Identity ConstructSdfWorld(
      const Identity &/*_engine*/,
      const ::sdf::World &_sdfWorld) override;

  public: Identity ConstructSdfModel(
      const Identity &_parentID,
      const ::sdf::Model &_sdfModel) override;

  /// \brief Implementation that handles models and nested models
  /// \param[in] _parentID Id of parent
  /// \param[in] _sdfModel sdf::Model to construct entity from
  /// \return The entity identity if constructed otherwise an invalid identity
  private: Identity ConstructSdfModelImpl(Identity _parentID,
                                          const ::sdf::Model &_sdfModel);
};
}  // namespace mujoco
}  // namespace physics
}  // namespace gz
#endif
