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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_GETENTITIESFEATURE_HH_
#define IGNITION_PHYSICS_DARTSIM_SRC_GETENTITIESFEATURE_HH_

#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Implements.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using EntityManagementFeatureList = FeatureList<GetEntities>;

class EntityManagementFeatures :
    public virtual Base,
    public virtual Implements3d<EntityManagementFeatureList>
{
  public: const std::string &GetEngineName(const std::size_t) const override;

  public: std::size_t GetEngineIndex(const std::size_t) const override;

  public: std::size_t GetWorldCount(const std::size_t) const override;

  public: Identity GetWorld(
      const std::size_t, const std::string &_worldName) const override;


};

}
}
}

#endif
