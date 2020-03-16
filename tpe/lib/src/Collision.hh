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

#ifndef IGNITION_PHYSICS_TPE_LIB_SRC_COLLISISION_HH_
#define IGNITION_PHYSICS_TPE_LIB_SRC_COLLISISION_HH_

#include <ignition/common/Console.hh>

#include "Entity.hh"
#include "Shape.hh"

namespace ignition {
namespace physics {
namespace tpe {
namespace lib {

// Forward declartion
class CollisionPrivate;

/// \brief Collision class
class Collision : public Entity
{
  /// \brief Constructor
  public: Collision();

  /// \brief Constructor
  /// \param _id Collision id
  public: Collision(uint64_t _id);

  /// \brief Copy Constructor
  public: Collision(const Collision &_other);

  /// \brief Destructor
  public: ~Collision();

  /// \brief Assignment operator
  /// \param _other collision
  /// \return collision
  public: Collision &operator=(const Collision &_other);

  /// \brief Set Shape
  /// \param _shape shape
  public: void SetShape(const Shape &_shape);

  /// brief Get Shape
  public: Shape *GetShape() const;

  /// \brief Private data pointer class
  private: CollisionPrivate *dataPtr = nullptr;
};

}
}
}
}

#endif
