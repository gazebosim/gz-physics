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

#ifndef IGNITION_PHYSICS_TPE_LIB_SRC_WORLD_HH_
#define IGNITION_PHYSICS_TPE_LIB_SRC_WORLD_HH_

#include <vector>
#include <ignition/utils/SuppressWarning.hh>

#include "gz/physics/tpelib/Export.hh"

#include "CollisionDetector.hh"
#include "Entity.hh"

namespace ignition {
namespace physics {
namespace tpelib {

class Model;

/// \brief World Class
class IGNITION_PHYSICS_TPELIB_VISIBLE World : public Entity
{
  /// \brief Constructor
  public: World();

  /// \brief Destructor
  public: virtual ~World() = default;

  /// \brief Set the time of the world
  /// \param[in] _time time of the world
  public: void SetTime(double _time);

  /// \brief Get the time of the world
  /// \return current time of the world
  public: double GetTime() const;

  /// \brief Set the time step of the world
  /// \param[in] _timestep time step to set to.
  public: void SetTimeStep(double _timeStep);

  /// \brief Get the timestep
  /// \return double current timestep of the world
  public: double GetTimeStep() const;

  /// \brief Step forward at a constant timestep
  public: void Step();

  /// \brief Add a model to this world
  /// \return Model added to the world
  public: Entity &AddModel();

  /// \brief Get contacts from last step
  /// \return Contacts from last step
  public: std::vector<Contact> GetContacts() const;

  /// \brief World time
  protected: double time{0.0};

  /// \brief Time step size
  protected: double timeStep{0.1};

  /// \brief Collision detector
  protected: CollisionDetector collisionDetector;

  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief list of contacts
  protected: std::vector<Contact> contacts;
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

}  // namespace tpelib
}  // namespace physics
}  // namespace ignition

#endif
