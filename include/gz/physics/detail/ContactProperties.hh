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

#ifndef GZ_PHYSICS_DETAIL_CONTACTPROPERTIES_HH_
#define GZ_PHYSICS_DETAIL_CONTACTPROPERTIES_HH_

#include <string>
#include <utility>
#include <vector>

#include <gz/physics/ContactProperties.hh>

namespace ignition
{
namespace physics
{

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void SetContactPropertiesCallbackFeature::World<PolicyT, FeaturesT>::
  AddContactPropertiesCallback(
    const std::string &_callbackID,
    SurfaceParamsCallback _callback)
{
  using Impl = Implementation<PolicyT>;
  using ContactInternal = typename Impl::ContactImpl;

  auto pimplPtr = this->pimpl;
  auto convertContact = [pimplPtr](const ContactInternal& _internal)
  {
    using ContactPoint =
      typename GetContactsFromLastStepFeature::World<PolicyT, FeaturesT>
        ::ContactPoint;

    ContactPoint contactPoint {
      ShapePtrType(pimplPtr, _internal.collision1),
      ShapePtrType(pimplPtr, _internal.collision2),
      _internal.point
    };

    typename GetContactsFromLastStepFeature::World<PolicyT, FeaturesT>::Contact
    contactOutput;
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

  this->template Interface<SetContactPropertiesCallbackFeature>()
    ->AddContactPropertiesCallback(
      this->identity, _callbackID, callbackInternal);
}

/////////////////////////////////////////////////
template<typename PolicyT, typename FeaturesT>
bool SetContactPropertiesCallbackFeature::World<PolicyT, FeaturesT>::
  RemoveContactPropertiesCallback(const std::string& _callbackID)
{
  return this->template Interface<SetContactPropertiesCallbackFeature>()->
    RemoveContactPropertiesCallback(this->identity, _callbackID);
}

}  // namespace physics
}  // namespace ignition

#endif
