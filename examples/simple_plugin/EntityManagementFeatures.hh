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

//! [basic include]
#include <string>
#include <ignition/physics/Implements.hh>
//! [basic include]

//! [include feature]
#include <ignition/physics/ConstructEmpty.hh>

namespace ignition {
namespace physics {
namespace simpleplugin {

struct EntityManagementFeatureList : FeatureList<
  ConstructEmptyWorldFeature
> { };

//! [include feature]

//! [override feature]
class EntityManagementFeatures :
  public virtual Implements3d<EntityManagementFeatureList>
{
  /// \brief Construct an empty dummy world.
  /// \param[in] _engineID Identity for the engine.
  /// \param[in] _name Name of the world.
  public: Identity ConstructEmptyWorld(
    const Identity &_engineID, const std::string &_name) override;
};

}
}
}
//! [override feature]
