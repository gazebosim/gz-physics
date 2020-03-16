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

#ifndef IGNITION_PHYSICS_TPE_LIB_SRC_ENGINE_HH_
#define IGNITION_PHYSICS_TPE_LIB_SRC_ENGINE_HH_

#include <string>
#include <map>

#include "Entity.hh"

namespace ignition {
namespace physics {
namespace tpe {
namespace lib{

class World;

/// \brief Engine class
class Engine
{
  /// \brief Constructor
  public: Engine();

  /// \brief Destructor
  public: ~Engine() = default;

  /// \brief Add world to engine
  /// \return added World entity
  public: Entity &AddWorld();

  /// \brief Get World
  /// \param_worldId World ID
  public: Entity &GetWorldById(uint64_t _worldId);

  /// \brief Get total number of worlds
  /// \return number of worlds
  public: uint64_t GetWorldCount() const;

  /// \brief Get all worlds in engine
  /// \return a map of id -> world
  public: std::map<uint64_t, Entity> GetWorlds();

  /// \brief Remove World from engine
  /// \return true/false if world is removed/not
  public: bool RemoveWorldById(uint64_t _worldId);

  /// \brief World entities in engine
  protected: std::map<uint64_t, Entity> worlds;
};

} // namespace lib
} // namespace tpe
} // namespace physics
} // namespace ignition

#endif
