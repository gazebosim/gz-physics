/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_TPE_PLUGIN_SRC_GETENTITIESFEATURE_HH_
#define IGNITION_PHYSICS_TPE_PLUGIN_SRC_GETENTITIESFEATURE_HH_

#include <string>
#include <ignition/physics/ConstructEmpty.hh>
#include <ignition/physics/Shape.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/RemoveEntities.hh>
#include <ignition/physics/Implements.hh>

#include "Base.hh" // optionally depending on software design

namespace ignition {
namespace physics {
namespace tpeplugin {

struct EntityManagementFeatureList : FeatureList<
  ConstructEmptyWorldFeature
> { };

class EntityManagementFeatures :
  public virtual Base,
  public virtual Implements3d<EntityManagementFeatureList>
{
  // ----- Construct empty entities -----
  public: Identity ConstructEmptyWorld(
    const Identity &_engineID, const std::string &_name) override;
};

}
}
}

#endif
