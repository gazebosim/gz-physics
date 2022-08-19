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

#ifndef GZ_PHYSICS_REMOVEENTITIES_HH_
#define GZ_PHYSICS_REMOVEENTITIES_HH_

#include <string>

#include <gz/physics/FeatureList.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief This feature removes a Model entity from the index-specified
    /// World.
    class IGNITION_PHYSICS_VISIBLE RemoveModelFromWorld : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        /// \brief Remove a Model that exists within this World.
        /// \param[in] _index
        ///   Index of the model within this world.
        /// \return True if the model was found and removed.
        public: bool RemoveModel(std::size_t _index);

        /// \brief Remove a Model that exists within this World.
        /// \param[in] _name
        ///   Name of the model within this world.
        /// \return True if the model was found and removed.
        public: bool RemoveModel(const std::string &_name);
      };

      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        /// \brief Remove this model
        /// \returns True if the model was found and removed
        public: bool Remove();

        /// \brief Check if the entity is removed
        public: bool Removed() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        // World functions
        public: virtual bool RemoveModelByIndex(
            const Identity &_worldID, std::size_t _modelIndex) = 0;

        public: virtual bool RemoveModelByName(
            const Identity &_worldID, const std::string &_modelName) = 0;

        // Model functions
        public: virtual bool RemoveModel(
            const Identity &_modelID) = 0;

        public: virtual bool ModelRemoved(const Identity &_modelID) const = 0;
      };
    };

    using RemoveEntities = FeatureList<
      RemoveModelFromWorld
    >;
  }
}

#include <gz/physics/detail/RemoveEntities.hh>

#endif
