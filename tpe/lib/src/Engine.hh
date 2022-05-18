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

#ifndef GZ_PHYSICS_TPE_LIB_SRC_ENGINE_HH_
#define GZ_PHYSICS_TPE_LIB_SRC_ENGINE_HH_

#include <map>
#include <memory>

#include <ignition/utils/SuppressWarning.hh>

#include "gz/physics/tpelib/Export.hh"

#include "Entity.hh"

namespace gz {
namespace physics {
namespace tpelib {

class World;

/// \brief Engine class
class IGNITION_PHYSICS_TPELIB_VISIBLE Engine
{
  /// \brief Constructor
  public: Engine();

  /// \brief Destructor
  public: ~Engine() = default;

  /// \brief Add world to engine
  /// \return added World entity
  public: Entity &AddWorld();

  /// \brief Get World
  /// \param[in] _worldId World ID
  public: Entity &GetWorldById(std::size_t _worldId);

  /// \brief Get total number of worlds
  /// \return number of worlds
  public: std::size_t GetWorldCount() const;

  /// \brief Get all worlds in engine
  /// \return a map of id -> world
  public: std::map<std::size_t, std::shared_ptr<Entity>> GetWorlds() const;

  /// \brief Remove World from engine
  /// \return true/false if world is removed/not
  public: bool RemoveWorldById(std::size_t _worldId);

  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief World entities in engine
  protected: std::map<std::size_t, std::shared_ptr<Entity>> worlds;
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

}  // namespace tpelib
}  // namespace physics
}  // namespace gz

#endif
