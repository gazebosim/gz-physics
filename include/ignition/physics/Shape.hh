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

#ifndef IGNITION_PHYSICS_SHAPE_HH_
#define IGNITION_PHYSICS_SHAPE_HH_

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/RelativeQuantity.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief This feature retrieves the Shape kinematic properties such as the
    /// the transform from the the Link frame to this Shape frame.
    class IGNITION_PHYSICS_VISIBLE GetShapeKinematicProperties
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Shape : public virtual Feature::Shape<PolicyT, FeaturesT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        /// \brief Get the transform from the Link frame to the frame of this
        /// Shape.
        /// \remark Shapes are rigidly attached to their Link, but their
        /// transform may be offset from the Link frame.
        /// \return the transform from the Link frame to the frame of this Shape
        public: PoseType GetRelativeTransform() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual PoseType GetShapeRelativeTransform(
            const Identity &_shapeID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE SetShapeKinematicProperties
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Shape : public virtual Feature::Shape<PolicyT, FeaturesT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        /// \brief Set the transform from the Link frame to the frame of this
        /// Shape.
        /// \remark Shapes are rigidly attached to their Link, but their
        /// transform may be offset from the Link frame.
        /// \param[in] _pose
        ///   The desired relative transformation from the owning Link
        public: void SetRelativeTransform(const PoseType &_pose);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual void SetShapeRelativeTransform(
            const Identity &_shapeID, const PoseType &_pose) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the Shape collision properties such as
    /// the Shape surface friction coefficient and restitution coefficient.
    class IGNITION_PHYSICS_VISIBLE GetShapeCollisionProperties
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Shape : public virtual Feature::Shape<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Get the coefficient of friction between this shape and
        /// another shape.
        /// \param[in] _other
        ///   The other shape. Coefficient of friction is a function of two
        ///   objects, and this argument determines the other object.
        /// \return The coefficient of friction between this shape and _other.
        public: Scalar GetFrictionCoefficient(
            const BaseShapePtr<PolicyT> &_other) const;

        /// \brief Get the coefficient of restitution between this shape and
        /// another shape.
        /// \param[in] _other
        ///   The other shape. Coefficient of restitution is a function of two
        ///   objects, and this argument determines the other object.
        /// \return The coefficient of restitution between this shape and
        /// _other.
        public: Scalar GetRestitutionCoefficient(
            const BaseShapePtr<PolicyT> &_other) const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual Scalar GetFrictionCoefficient(
            const Identity &_shape0, const Identity &_shape1) const = 0;

        public: virtual Scalar GetRestitutionCoefficient(
            const Identity &_shape0, const Identity &_shape1) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the Shape's axis aligned bounding box in 
    /// the requested frame.
    class IGNITION_PHYSICS_VISIBLE GetShapeBoundingBox
        : public virtual FeatureWithRequirements<ShapeFrameSemantics>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Shape
          : public virtual ShapeFrameSemantics::Shape<PolicyT, FeaturesT>
      {
        public: using AlignedBoxType =
            typename FromPolicy<PolicyT>::template Use<AlignedBox>;

        /// \brief Get the axis aligned bounding box for the shape in the
        /// requested frame.
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

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using AlignedBoxType =
            typename FromPolicy<PolicyT>::template Use<AlignedBox>;

        /// \brief Implementation function for querying the axis-aligned
        /// bounding box of a shape.
        /// \return The axis-aligned bounding box in the frame of the box.
        public: virtual AlignedBoxType GetShapeAxisAlignedBoundingBox(
            const Identity &_shape) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE SetShapeCollisionProperties
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Shape : public virtual Feature::Shape<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the coefficient of friction between this shape and
        /// another shape.
        /// \param[in] _other
        ///   The other shape. Coefficient of friction is a function of two
        ///   objects, and this argument determines the other object.
        /// \param[in] _value
        ///   The value to set the coefficient to.
        public: void SetFrictionCoefficient(
            const BaseShapePtr<PolicyT> &_other, Scalar _value);

        /// \brief Set the coefficient of restitution between this shape and
        /// another shape.
        /// \param[in] _other
        ///   The other shape. Coefficient of restitution is a function of two
        ///   objects, and this argument determines the other object.
        /// \param[in] _value
        ///   The value to set the coefficient to.
        public: void SetRestitutionCoefficient(
            const BaseShapePtr<PolicyT> &_other, Scalar _value);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual void SetShapeFrictionCoefficient(
            const Identity &_shape0,
            const Identity &_shape1,
            Scalar _value) = 0;

        public: virtual void SetShapeRestitutionCoefficient(
            const Identity &_shape0,
            const Identity &_shape1,
            Scalar _value) = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE CollisionFilterMaskFeature
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Shape : public virtual Feature::Shape<PolicyT, FeaturesT>
      {
        /// \brief Set the collision filter bitmask of this shape
        /// \param[in] _mask
        ///   A sixteen bit bitmask, if the bitwise AND of two objects
        ///   evaluates to 0, their collision will be ignored
        public: void SetCollisionFilterMask(const uint16_t _mask);

        /// \brief Get the collision filter bitmask of this shape
        /// \return The collision filter bitmask of this shape
        public: uint16_t GetCollisionFilterMask() const;

        /// \brief Removes the collision filter bitmask from this shape
        public: void RemoveCollisionFilterMask();
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual void SetCollisionFilterMask(
            const Identity &_shapeID, const uint16_t _mask) = 0;

        public: virtual uint16_t GetCollisionFilterMask(
            const Identity &_shapeID) const = 0;

        public: virtual void RemoveCollisionFilterMask(
            const Identity &_shapeID) = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE GetShapeFrictionPyramidSlipCompliance
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Shape : public virtual Feature::Shape<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Get the slip compliance for the first friction direction
        /// of a friction pyramid model.
        /// \return
        ///   The value of the slip compliance.
        public: Scalar GetPrimarySlipCompliance() const;

        /// \brief Get the slip compliance for the second friction direction
        /// of a friction pyramid model.
        /// \return
        ///   The value of the slip compliance.
        public: Scalar GetSecondarySlipCompliance() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual Scalar GetShapeFrictionPyramidPrimarySlipCompliance(
            const Identity &_shapeID) const = 0;

        public: virtual Scalar GetShapeFrictionPyramidSecondarySlipCompliance(
            const Identity &_shapeID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE SetShapeFrictionPyramidSlipCompliance
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Shape : public virtual Feature::Shape<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the slip compliance for the first friction direction
        /// of a friction pyramid model.
        /// \param[in] _value
        ///   The value to set the slip compliance to.
        /// \return
        ///   True if the parameter was set successfully, false otherwise.
        public: bool SetPrimarySlipCompliance(Scalar _value);

        /// \brief Set the slip compliance for the second friction direction
        /// of a friction pyramid model.
        /// \param[in] _value
        ///   The value to set the slip compliance to.
        /// \return
        ///   True if the parameter was set successfully, false otherwise.
        public: bool SetSecondarySlipCompliance(Scalar _value);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual bool SetShapeFrictionPyramidPrimarySlipCompliance(
            const Identity &_shapeID, Scalar _value) = 0;

        public: virtual bool SetShapeFrictionPyramidSecondarySlipCompliance(
            const Identity &_shapeID, Scalar _value) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/Shape.hh>

#endif
