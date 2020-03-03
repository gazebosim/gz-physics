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
    auto GetLinkBoundingBox::Link<PolicyT, FeaturesT>
    ::GetAxisAlignedBoundingBox(const FrameID &_referenceFrame) const
    -> AlignedBoxType
    {
      AlignedBoxType result;
      std::size_t shapeCount = this->GetShapeCount();
      for (std::size_t i = 0; i < shapeCount; ++i)
      {
        // for each shape in this link, merge its AlignedBox into result
        auto shape = this->GetShape(i);
        result =
            result.merged(shape->GetAxisAlignedBoundingBox(_referenceFrame));
      }
      return result;
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetModelBoundingBox::Model<PolicyT, FeaturesT>
    ::GetAxisAlignedBoundingBox(const FrameID &_referenceFrame) const
    -> AlignedBoxType
    {
      AlignedBoxType result;
      std::size_t linkCount = this->GetLinkCount();
      for (std::size_t i = 0; i < linkCount; ++i)
      {
        // for each shape in this link, merge its AlignedBox into result
        auto link = this->GetLink(i);
        result =
            result.merged(link->GetAxisAlignedBoundingBox(_referenceFrame));
      }
      return result;
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
  }
}

#endif
