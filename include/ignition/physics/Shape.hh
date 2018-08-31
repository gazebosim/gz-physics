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
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
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
            std::size_t _shapeID) const = 0;
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
            std::size_t _shapeID, const PoseType &_pose) = 0;
      };
    };

    /////////////////////////////////////////////////
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
            std::size_t _shape0, std::size_t _shape1) const = 0;

        public: virtual Scalar GetRestitutionCoefficient(
            std::size_t _shape0, std::size_t _shape1) const = 0;
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
            std::size_t _shape0, std::size_t _shape1, Scalar _value) = 0;

        public: virtual void SetShapeRestitutionCoefficient(
            std::size_t _shape0, std::size_t _shape1, Scalar _value) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/Shape.hh>

#endif
