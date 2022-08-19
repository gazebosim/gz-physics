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

#ifndef GZ_PHYSICS_BOXSHAPE_HH_
#define GZ_PHYSICS_BOXSHAPE_HH_

#include <string>

#include <ignition/physics/DeclareShapeType.hh>
#include <ignition/physics/Geometry.hh>

namespace gz
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_SHAPE_TYPE(BoxShape)

    class IGNITION_PHYSICS_VISIBLE GetBoxShapeProperties
      : public virtual FeatureWithRequirements<BoxShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class BoxShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        /// \brief Get the dimensions of this BoxShape
        /// \return the dimensions of this BoxShape
        public: Dimensions GetSize() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: virtual Dimensions GetBoxShapeSize(
            const Identity &_boxID) const = 0;
      };
    };

    class IGNITION_PHYSICS_VISIBLE SetBoxShapeProperties
        : public virtual FeatureWithRequirements<BoxShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class BoxShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        /// \brief Set the dimensions of this BoxShape
        /// \param[in] _size
        ///   The desired dimensions of this BoxShape
        public: void SetSize(const Dimensions &_size);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: virtual void SetBoxShapeSize(
            const Identity &_boxID, const Dimensions &_size) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature constructs a new box shape and attaches the
    /// desired pose in the link frame. The pose could be defined to be the box
    /// center point or any corner in actual implementation.
    class IGNITION_PHYSICS_VISIBLE AttachBoxShapeFeature
        : public virtual FeatureWithRequirements<BoxShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: using ShapePtrType = BoxShapePtr<PolicyT, FeaturesT>;

        /// \brief Rigidly attach a BoxShape to this link.
        /// \param[in] _size
        ///   The size of the BoxShape's dimensions.
        /// \param[in] _pose
        ///   The desired pose of the BoxShape relative to the Link frame.
        /// \returns a reference to the newly constructed BoxShape
        // TODO(MXG): Create a struct that contains all the relevant properties,
        // and pass that in here.
        public: ShapePtrType AttachBoxShape(
            const std::string &_name = "box",
            const Dimensions &_size = Dimensions::Constant(1.0),
            const PoseType &_pose = PoseType::Identity());
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual Identity AttachBoxShape(
            const Identity &_linkID,
            const std::string &_name,
            const Dimensions &_size,
            const PoseType &_pose) = 0;
      };
    };
  }
}

#include <gz/physics/detail/BoxShape.hh>

#endif
