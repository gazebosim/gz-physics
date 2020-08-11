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

#ifndef IGNITION_PHYSICS_GETBOUNDINGBOX_HH_
#define IGNITION_PHYSICS_GETBOUNDINGBOX_HH_

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Shape.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    using GetLinkBoundingBoxRequiredFeatures = FeatureList<
      GetShapeBoundingBox,
      GetShapeFromLink,
      LinkFrameSemantics>;
    class IGNITION_PHYSICS_VISIBLE GetLinkBoundingBox
        : public virtual
          FeatureWithRequirements<GetLinkBoundingBoxRequiredFeatures>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link
          : public virtual GetShapeFromLink::Link<PolicyT, FeaturesT>
      {
        public: using AlignedBoxType =
            typename FromPolicy<PolicyT>::template Use<AlignedBox>;

        /// \brief Get the axis aligned bounding box for the shapes attached
        /// to this link in the requested frame.
        /// \param[in] _referenceFrame
        ///   The desired frame for the bounding box. By default, this will be
        ///   the world frame.
        ///   \note Axis-aligned bounding boxes will expand each time they are
        ///   transformed into a new frame that has a different orientation.
        /// \return Axis aligned bounding box for the shape, transformed into
        /// the requested coordinate frame.
        public: AlignedBoxType GetAxisAlignedBoundingBox(
            const FrameID &_referenceFrame = FrameID::World()) const;
      };
    };

    /////////////////////////////////////////////////
    using GetModelBoundingBoxRequiredFeatures = FeatureList<
      GetLinkBoundingBox,
      GetLinkFromModel,
      ModelFrameSemantics>;
    class IGNITION_PHYSICS_VISIBLE GetModelBoundingBox
        : public virtual
          FeatureWithRequirements<GetModelBoundingBoxRequiredFeatures>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Model
          : public virtual GetLinkFromModel::Model<PolicyT, FeaturesT>
      {
        public: using AlignedBoxType =
            typename FromPolicy<PolicyT>::template Use<AlignedBox>;

        /// \brief Get the axis aligned bounding box for the links attached
        /// to this model in the requested frame.
        /// \param[in] _referenceFrame
        ///   The desired frame for the bounding box. By default, this will be
        ///   the world frame.
        ///   \note Axis-aligned bounding boxes will expand each time they are
        ///   transformed into a new frame that has a different orientation.
        /// \return Axis aligned bounding box for the shape, transformed into
        /// the requested coordinate frame.
        public: AlignedBoxType GetAxisAlignedBoundingBox(
            const FrameID &_referenceFrame = FrameID::World()) const;
      };
    };

    // see Shape.hh for GetShapeBoundingBox
  }
}

#include <ignition/physics/detail/GetBoundingBox.hh>

#endif
