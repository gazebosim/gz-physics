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

#ifndef IGNITION_PHYSICS_DETAIL_PRISMATICJOINT_HH_
#define IGNITION_PHYSICS_DETAIL_PRISMATICJOINT_HH_

#include <ignition/physics/PrismaticJoint.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetPrismaticJointProperties::PrismaticJoint<PolicyT, FeaturesT>::
    GetAxis() const -> Axis
    {
      return this->template Interface<GetPrismaticJointProperties>()
          ->GetPrismaticJointAxis(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetPrismaticJointProperties::PrismaticJoint<PolicyT, FeaturesT>::
    SetAxis(const Axis &_axis)
    {
      this->template Interface<SetPrismaticJointProperties>()
          ->SetPrismaticJointAxis(this->identity, _axis);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto AttachPrismaticJointFeature::Link<PolicyT, FeaturesT>::
    AttachPrismaticJoint(
        const BaseLinkPtr<PolicyT> &_parent,
        const Axis &_axis) -> JointPtrType
    {
      return JointPtrType(this->pimpl,
            this->template Interface<AttachPrismaticJointFeature>()
              ->AttachPrismaticJoint(this->identity, _parent, _axis));
    }
  }
}

#endif
