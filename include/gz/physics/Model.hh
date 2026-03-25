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

#ifndef GZ_PHYSICS_MODEL_HH_
#define GZ_PHYSICS_MODEL_HH_

#include <gz/physics/FeatureList.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief Feature for getting and setting the static state of a model.
    /// A static model has all its bodies fixed in place (infinite mass).
    class GZ_PHYSICS_VISIBLE ModelStaticState : public virtual Feature
    {
      /// \brief The Model API for getting and setting static state.
      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        /// \brief Set whether this model is static (fixed in place).
        /// \param[in] _static True to make the model static, false otherwise.
        public: void SetStatic(bool _static);

        /// \brief Get whether this model is static.
        /// \return True if the model is static, false otherwise.
        public: bool GetStatic() const;
      };

      /// \private The implementation API for model static state.
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        /// \brief Implementation API for setting the model static state.
        /// \param[in] _id Identity of the model.
        /// \param[in] _static True to make the model static.
        public: virtual void SetModelStatic(
            const Identity &_id, bool _static) = 0;

        /// \brief Implementation API for getting the model static state.
        /// \param[in] _id Identity of the model.
        /// \return True if the model is static.
        public: virtual bool GetModelStatic(const Identity &_id) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief Feature for getting and setting whether gravity affects a model.
    class GZ_PHYSICS_VISIBLE ModelGravityEnabled : public virtual Feature
    {
      /// \brief The Model API for getting and setting gravity mode.
      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        /// \brief Set whether gravity is enabled for this model.
        /// \param[in] _enabled True to enable gravity, false to disable it.
        public: void SetGravityEnabled(bool _enabled);

        /// \brief Get whether gravity is enabled for this model.
        /// \return True if gravity is enabled, false otherwise.
        public: bool GetGravityEnabled() const;
      };

      /// \private The implementation API for model gravity mode.
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
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

#include <gz/physics/detail/Model.hh>

#endif
