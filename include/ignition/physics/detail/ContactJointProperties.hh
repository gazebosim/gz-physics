/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DETAIL_CONTACTJOINTPROPERTIES_HH_
#define IGNITION_PHYSICS_DETAIL_CONTACTJOINTPROPERTIES_HH_

#include <string>
#include <utility>
#include <vector>

#include <ignition/physics/ContactJointProperties.hh>

namespace ignition
{
namespace physics
{

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void SetContactJointPropertiesCallbackFeature::World<PolicyT, FeaturesT>::
  AddContactJointPropertiesCallback(
    const std::string &_callbackID,
    SurfaceParamsCallback _callback)
{
  using Impl = Implementation<PolicyT>;
  using ContactInternal = typename Impl::ContactImpl;

  auto pimpl = this->pimpl;
  auto convertContact = [pimpl](const ContactInternal& _internal) -> Contact
  {
    using ContactPoint =
      typename GetContactsFromLastStepFeature::World<PolicyT, FeaturesT>
        ::ContactPoint;

    ContactPoint contactPoint {
      ShapePtrType(pimpl, _internal.collision1),
      ShapePtrType(pimpl, _internal.collision2),
      _internal.point
    };

    Contact contactOutput;
    contactOutput.template Get<ContactPoint>() = std::move(contactPoint);

    using ExtraContactData =
      GetContactsFromLastStepFeature::ExtraContactDataT<PolicyT>;

    auto *extraContactData =
      _internal.extraData.template Query<ExtraContactData>();

    if (extraContactData)
    {
      contactOutput.template Get<ExtraContactData>() =
        std::move(*extraContactData);
    }

    return contactOutput;
  };

  typename Impl::SurfaceParamsCallback callbackInternal = nullptr;
  if (_callback)
  {
    callbackInternal = [_callback, convertContact](
      const typename Impl::ContactImpl &_contact,
      const size_t _numContactsOnCollision,
      ContactSurfaceParams<PolicyT> &_params)
      {
        _callback(convertContact(_contact), _numContactsOnCollision, _params);
      };
  }

  this->template Interface<SetContactJointPropertiesCallbackFeature>()
    ->AddContactJointPropertiesCallback(
      this->identity, _callbackID, callbackInternal);
}

/////////////////////////////////////////////////
template<typename PolicyT, typename FeaturesT>
bool SetContactJointPropertiesCallbackFeature::World<PolicyT, FeaturesT>::
  RemoveContactJointPropertiesCallback(const std::string& _callbackID)
{
  return this->template Interface<SetContactJointPropertiesCallbackFeature>()->
    RemoveContactJointPropertiesCallback(this->identity, _callbackID);
}

}  // namespace physics
}  // namespace ignition

#endif
