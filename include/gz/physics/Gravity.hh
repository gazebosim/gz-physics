/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_GRAVITY_HH_
#define GZ_PHYSICS_GRAVITY_HH_

#include <gz/physics/FeatureList.hh>
#include <gz/physics/GetEntities.hh>

namespace gz
{
  namespace physics
  {
    /// \brief Feature for getting and setting whether gravity affects a link
    /// or a model.
    class GZ_PHYSICS_VISIBLE GravityEnabled : public virtual Feature
    {
      /// \brief The Link API for getting and setting gravity mode.
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        /// \brief Set whether gravity is enabled for this link.
        /// \param[in] _enabled True to enable gravity, false to disable it.
        public: void SetGravityEnabled(bool _enabled);

        /// \brief Get whether gravity is enabled for this link.
        /// \return True if gravity is enabled, false otherwise.
        public: bool GetGravityEnabled() const;
      };

      /// \brief The Model API for getting and setting gravity mode.
      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        /// \brief Set whether gravity is enabled for all of this model's links
        /// and the links of nested models recursively.
        /// \param[in] _enabled True to enable gravity, false to disable it.
        public: void SetGravityEnabled(bool _enabled);

        /// \brief Get whether gravity is enabled for this model.
        /// \return True if gravity is enabled for all child links and the
        /// links of nested models recursively, false otherwise.
        public: bool GetGravityEnabled() const;
      };

      /// \private The implementation API for gravity mode.
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        /// \brief Implementation API for setting the link gravity mode.
        /// \param[in] _id Identity of the link.
        /// \param[in] _enabled True to enable gravity.
        public: virtual void SetLinkGravityEnabled(
            const Identity &_id, bool _enabled) = 0;

        /// \brief Implementation API for getting the link gravity mode.
        /// \param[in] _id Identity of the link.
        /// \return True if gravity is enabled.
        public: virtual bool GetLinkGravityEnabled(
            const Identity &_id) const = 0;

        /// \brief Implementation API for setting the model gravity mode.
        /// \param[in] _id Identity of the model.
        /// \param[in] _enabled True to enable gravity.
        public: virtual void SetModelGravityEnabled(
            const Identity &_id, bool _enabled) = 0;

        /// \brief Implementation API for getting the model gravity mode.
        /// \param[in] _id Identity of the model.
        /// \return True if gravity is enabled.
        public: virtual bool GetModelGravityEnabled(
            const Identity &_id) const = 0;
      };
    };
  }
}

#include <gz/physics/detail/Gravity.hh>

#endif
