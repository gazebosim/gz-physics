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

#ifndef GZ_PHYSICS_DETAIL_CONESHAPE_HH_
#define GZ_PHYSICS_DETAIL_CONESHAPE_HH_

#include <string>

#include <gz/physics/ConeShape.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetConeShapeProperties::ConeShape<PolicyT, FeaturesT>
    ::GetRadius() const -> Scalar
    {
      return this->template Interface<GetConeShapeProperties>()
          ->GetConeShapeRadius(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetConeShapeProperties::ConeShape<PolicyT, FeaturesT>
    ::GetHeight() const -> Scalar
    {
      return this->template Interface<GetConeShapeProperties>()
          ->GetConeShapeHeight(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetConeShapeProperties::ConeShape<PolicyT, FeaturesT>
    ::SetRadius(Scalar _radius)
    {
      this->template Interface<SetConeShapeProperties>()
          ->SetConeShapeRadius(this->identity, _radius);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetConeShapeProperties::ConeShape<PolicyT, FeaturesT>
    ::SetHeight(Scalar _height)
    {
      this->template Interface<SetConeShapeProperties>()
          ->SetConeShapeHeight(this->identity, _height);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto AttachConeShapeFeature::Link<PolicyT, FeaturesT>
    ::AttachConeShape(
        const std::string &_name,
        Scalar _radius,
        Scalar _height,
        const PoseType &_pose) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<AttachConeShapeFeature>()
                ->AttachConeShape(
                            this->identity, _name, _radius, _height, _pose));
    }
  }
}

#endif
