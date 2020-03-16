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

#ifndef IGNITION_PHYSICS_TPE_LIB_SRC_LINK_HH_
#define IGNITION_PHYSICS_TPE_LIB_SRC_LINK_HH_

#include "Entity.hh"

namespace ignition {
namespace physics {
namespace tpe {
namespace lib {

class Link : public Entity
{
  /// \brief Constructor
  public: Link();

  /// \brief Constructor
  /// \param _id Link id
  public: Link(uint64_t _id);

  /// \brief Destructor
  public: ~Link() = default;

  /// \brief Add a collision
  /// \return Newly created Collision
  public: Entity &AddCollision();
};

}
}
}
}

#endif
