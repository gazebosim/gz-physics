/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_KINEMATIC_HH_
#define GZ_PHYSICS_KINEMATIC_HH_

#include <gz/physics/FeatureList.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    class GZ_PHYSICS_VISIBLE Kinematic
      : public virtual Feature
    {
      /// \brief The Link API for setting link to be kinematic
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        /// \brief Set link to be kinematic.
        /// \param[i] _kinematic True to make this link kinematic.
        public: void SetKinematic(bool _kinematic);

        /// \brief Get whether this link is kinematic.
        /// \return True if the link is kinematic, false otherwise.
        public: bool GetKinematic() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        /// \brief Implementation API for setting a link to be kinematic
        /// \param[in] _id Identity of the link
        /// \param[in] _kinematic True to make this link kinematic
        public: virtual void SetLinkKinematic(
            const Identity &_shapeID, bool _kinematic) = 0;

        /// \brief Implementation API for getting whether a link is kinematic
        /// \param[in] _id Identity of the link
        /// \return True if the link is kinematic, false otherwise.
        public: virtual bool GetLinkKinematic(
            const Identity &_shapeID) const = 0;
      };
    };
  }
}

#include <gz/physics/detail/Kinematic.hh>

#endif
