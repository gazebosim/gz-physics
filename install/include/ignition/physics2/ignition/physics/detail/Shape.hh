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

#ifndef IGNITION_PHYSICS_DETAIL_SHAPE_HH_
#define IGNITION_PHYSICS_DETAIL_SHAPE_HH_

#include <ignition/physics/Shape.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeKinematicProperties::Shape<PolicyT, FeaturesT>
    ::GetRelativeTransform() const -> PoseType
    {
      return this->template Interface<GetShapeKinematicProperties>()
                 ->GetShapeRelativeTransform(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetShapeKinematicProperties::Shape<PolicyT, FeaturesT>
    ::SetRelativeTransform(const PoseType &_pose)
    {
      this->template Interface<SetShapeKinematicProperties>()
          ->SetShapeRelativeTransform(this->identity, _pose);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeCollisionProperties::Shape<PolicyT, FeaturesT>
    ::GetFrictionCoefficient(const BaseShapePtr<PolicyT> &_other) const
    -> Scalar
    {
      return this->template Interface<GetShapeCollisionProperties>()
                 ->GetFrictionCoefficient(this->identity, _other);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeCollisionProperties::Shape<PolicyT, FeaturesT>
    ::GetRestitutionCoefficient(const BaseShapePtr<PolicyT> &_other) const
    -> Scalar
    {
      return this->template Interface<GetShapeCollisionProperties>()
                 ->GetRestitutionCoefficient(this->identity, _other);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeBoundingBox::Shape<PolicyT, FeaturesT>
    ::GetAxisAlignedBoundingBox(const FrameID &_referenceFrame) const
    -> AlignedBoxType
    {
      using RelativeAlignedBox =
          ignition::physics::RelativeAlignedBox<
            typename PolicyT::Scalar, PolicyT::Dim>;

      return detail::Resolve<PolicyT>(
            *this->template Interface<FrameSemantics>(),
            RelativeAlignedBox(
              this->GetFrameID(),
              this->template Interface<GetShapeBoundingBox>()
                ->GetShapeAxisAlignedBoundingBox(this->identity)),
            _referenceFrame, _referenceFrame);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetShapeCollisionProperties::Shape<PolicyT, FeaturesT>
    ::SetFrictionCoefficient(const BaseShapePtr<PolicyT> &_other, Scalar _value)
    {
      this->template Interface<SetShapeCollisionProperties>()
          ->SetShapeFrictionCoefficient(this->identity, _other, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetShapeCollisionProperties::Shape<PolicyT, FeaturesT>
    ::SetRestitutionCoefficient(
        const BaseShapePtr<PolicyT> &_other, Scalar _value)
    {
      this->template Interface<SetShapeCollisionProperties>()
          ->SetShapeRestitutionCoefficient(this->identity, _other, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void CollisionFilterMaskFeature::Shape<PolicyT, FeaturesT>
    ::SetCollisionFilterMask(const uint16_t _mask)
    {
      this->template Interface<CollisionFilterMaskFeature>()
        ->SetCollisionFilterMask(this->identity, _mask);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    uint16_t CollisionFilterMaskFeature::Shape<PolicyT, FeaturesT>
    ::GetCollisionFilterMask() const
    {
      return this->template Interface<CollisionFilterMaskFeature>()
        ->GetCollisionFilterMask(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void CollisionFilterMaskFeature::Shape<PolicyT, FeaturesT>
    ::RemoveCollisionFilterMask()
    {
      this->template Interface<CollisionFilterMaskFeature>()
        ->RemoveCollisionFilterMask(this->identity);
    }
  }
}

#endif
