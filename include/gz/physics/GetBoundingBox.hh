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

#ifndef GZ_PHYSICS_GETBOUNDINGBOX_HH_
#define GZ_PHYSICS_GETBOUNDINGBOX_HH_

#include <gz/physics/FeatureList.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Shape.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    using GetLinkBoundingBoxRequiredFeatures = FeatureList<
      GetShapeBoundingBox,
      GetShapeFromLink,
      LinkFrameSemantics>;

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the axis aligned bounding box for the
    /// shapes attached to this link in the requested frame. The default frame
    /// is the world frame.
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

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the axis aligned bounding box for the
    /// model in the requested frame. The default frame is the world frame.
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

        /// \brief Get the axis aligned bounding box for the model
        /// in the requested frame.
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

#include <gz/physics/detail/GetBoundingBox.hh>

#endif
