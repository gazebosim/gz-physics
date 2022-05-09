/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_DETAIL_CAPSULESHAPE_HH_
#define GZ_PHYSICS_DETAIL_CAPSULESHAPE_HH_

#include <string>

#include <gz/physics/CapsuleShape.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetCapsuleShapeProperties::CapsuleShape<PolicyT, FeaturesT>
    ::GetRadius() const -> Scalar
    {
      return this->template Interface<GetCapsuleShapeProperties>()
          ->GetCapsuleShapeRadius(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetCapsuleShapeProperties::CapsuleShape<PolicyT, FeaturesT>
    ::GetLength() const -> Scalar
    {
      return this->template Interface<GetCapsuleShapeProperties>()
          ->GetCapsuleShapeLength(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetCapsuleShapeProperties::CapsuleShape<PolicyT, FeaturesT>
    ::SetRadius(Scalar _radius)
    {
      this->template Interface<SetCapsuleShapeProperties>()
          ->SetCapsuleShapeRadius(this->identity, _radius);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetCapsuleShapeProperties::CapsuleShape<PolicyT, FeaturesT>
    ::SetLength(Scalar _length)
    {
      this->template Interface<SetCapsuleShapeProperties>()
          ->SetCapsuleShapeLength(this->identity, _length);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto AttachCapsuleShapeFeature::Link<PolicyT, FeaturesT>
    ::AttachCapsuleShape(
        const std::string &_name,
        Scalar _radius,
        Scalar _length,
        const PoseType &_pose) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<AttachCapsuleShapeFeature>()
                ->AttachCapsuleShape(
                            this->identity, _name, _radius, _length, _pose));
    }
  }
}

#endif
