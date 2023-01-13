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

#ifndef GZ_PHYSICS_DETAIL_FIXEDJOINT_HH_
#define GZ_PHYSICS_DETAIL_FIXEDJOINT_HH_

#include <string>

#include <gz/physics/FixedJoint.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto AttachFixedJointFeature::Link<PolicyT, FeaturesT>::AttachFixedJoint(
        const BaseLinkPtr<PolicyT> &_parent,
        const std::string &_name) -> JointPtrType
    {
      return JointPtrType(this->pimpl,
            this->template Interface<AttachFixedJointFeature>()
                ->AttachFixedJoint(this->identity, _parent, _name));
    }
  }
}

#endif
