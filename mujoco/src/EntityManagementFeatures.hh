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

#ifndef GZ_PHYSICS_MUJOCO_SRC_GETENTITIESFEATURE_HH_
#define GZ_PHYSICS_MUJOCO_SRC_GETENTITIESFEATURE_HH_

#include "Base.hh"
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/Implements.hh>

namespace gz {
namespace physics {
namespace mujoco {

struct EntityManagementFeatureList : FeatureList<
  // GetEntities
  // RemoveEntities,
  ConstructEmptyWorldFeature
  // ConstructEmptyModelFeature,
  // ConstructEmptyNestedModelFeature,
  // ConstructEmptyLinkFeature
  // CollisionFilterMaskFeature,
  // WorldModelFeature
> { };

class EntityManagementFeatures :
    public virtual Base,
    public virtual Implements3d<EntityManagementFeatureList>
{
  public: Identity ConstructEmptyWorld(const Identity &_engineID, const std::string &_name) override;
};

}
}
}
#endif
