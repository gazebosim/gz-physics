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

#ifndef IGNITION_PHYSICS_DETAIL_REVOLUTEJOINT_HH_
#define IGNITION_PHYSICS_DETAIL_REVOLUTEJOINT_HH_

#include <string>

#include <ignition/physics/RevoluteJoint.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetRevoluteJointProperties::RevoluteJoint<PolicyT, FeaturesT>::
    GetAxis() const -> Axis
    {
      return this->template Interface<GetRevoluteJointProperties>()
          ->GetRevoluteJointAxis(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetRevoluteJointProperties::RevoluteJoint<PolicyT, FeaturesT>::
    SetAxis(const Axis &_axis)
    {
      this->template Interface<SetRevoluteJointProperties>()
          ->SetRevoluteJointAxis(this->identity, _axis);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto AttachRevoluteJointFeature::Link<PolicyT, FeaturesT>
    ::AttachRevoluteJoint(
        const BaseLinkPtr<PolicyT> &_parent,
        const std::string &_name,
        const Axis &_axis) -> JointPtrType
    {
      return JointPtrType(this->pimpl,
            this->template Interface<AttachRevoluteJointFeature>()
              ->AttachRevoluteJoint(this->identity, _parent, _name, _axis));
    }
  }
}

#endif
