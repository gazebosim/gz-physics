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

#ifndef IGNITION_PHYSICS_DETAIL_SPHERESHAPE_HH_
#define IGNITION_PHYSICS_DETAIL_SPHERESHAPE_HH_

#include <string>

#include <ignition/physics/SphereShape.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetSphereShapeProperties::SphereShape<PolicyT, FeaturesT>
    ::GetRadius() const -> Scalar
    {
      return this->template Interface<GetSphereShapeProperties>()
          ->GetSphereShapeRadius(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetSphereShapeProperties::SphereShape<PolicyT, FeaturesT>::SetRadius(
        Scalar _radius)
    {
      this->template Interface<GetSphereShapeProperties>()
          ->SetSphereShapeRadius(this->identity, _radius);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto AttachSphereShapeFeature::Link<PolicyT, FeaturesT>
    ::AttachSphereShape(
        const std::string &_name,
        Scalar _radius,
        const PoseType &_pose) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<AttachSphereShapeFeature>()
                ->AttachSphereShape(this->identity, _name, _radius, _pose));
    }
  }
}

#endif
