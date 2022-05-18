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

#ifndef GZ_PHYSICS_DETAIL_ELLIPSOIDSHAPE_HH_
#define GZ_PHYSICS_DETAIL_ELLIPSOIDSHAPE_HH_

#include <string>

#include <gz/physics/EllipsoidShape.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEllipsoidShapeProperties::EllipsoidShape<PolicyT, FeaturesT>
    ::GetRadii() const -> Dimensions
    {
      return this->template Interface<GetEllipsoidShapeProperties>()
          ->GetEllipsoidShapeRadii(this->identity);
    }
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetEllipsoidShapeProperties::EllipsoidShape<PolicyT, FeaturesT>
    ::SetRadii(Dimensions _radii)
    {
      this->template Interface<SetEllipsoidShapeProperties>()
          ->SetEllipsoidShapeRadii(this->identity, _radii);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto AttachEllipsoidShapeFeature::Link<PolicyT, FeaturesT>
    ::AttachEllipsoidShape(
        const std::string &_name,
        Dimensions _radii,
        const PoseType &_pose) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<AttachEllipsoidShapeFeature>()
                ->AttachEllipsoidShape(
                            this->identity, _name, _radii, _pose));
    }
  }
}

#endif
