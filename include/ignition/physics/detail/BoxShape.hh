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

#ifndef IGNITION_PHYSICS_DETAIL_BOXSHAPE_HH_
#define IGNITION_PHYSICS_DETAIL_BOXSHAPE_HH_

#include <ignition/physics/BoxShape.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetBoxShapeProperties::BoxShape<PolicyT, FeaturesT>
    ::GetSize() const -> Dimensions
    {
      return this->template Interface<GetBoxShapeProperties>()
                 ->GetBoxShapeSize(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetBoxShapeProperties::BoxShape<PolicyT, FeaturesT>
    ::SetSize(const Dimensions &_size)
    {
      this->template Interface<SetBoxShapeProperties>()
          ->SetBoxShapeSize(this->identity, _size);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto AttachBoxShapeFeature::Link<PolicyT, FeaturesT>::AttachBoxShape(
        const std::string &_name,
        const Dimensions &_size,
        const PoseType &_pose) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<AttachBoxShapeFeature>()
                ->AttachBoxShape(this->identity, _name, _size, _pose));
    }
  }
}

#endif
