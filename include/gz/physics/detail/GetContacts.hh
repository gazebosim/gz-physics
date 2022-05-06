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

#ifndef GZ_PHYSICS_DETAIL_GETCONTACTS_HH_
#define GZ_PHYSICS_DETAIL_GETCONTACTS_HH_

#include <utility>
#include <vector>
#include <gz/physics/GetContacts.hh>

namespace ignition
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto GetContactsFromLastStepFeature::World<
    PolicyT, FeaturesT>::GetContactsFromLastStep() const -> std::vector<Contact>
{
  auto contactsInternal =
      this->template Interface<GetContactsFromLastStepFeature>()
          ->GetContactsFromLastStep(this->identity);
  // contactInternal provides the identities of collision1 and collision2. Here,
  // we create a new vector and create ShapePtrs out of those identities. Seems
  // inefficient, but I don't know if there's a better way to handle this.
  std::vector<Contact> output;
  output.reserve(contactsInternal.size());
  for (auto &contact : contactsInternal)
  {
    ContactPoint contactPoint{ShapePtrType(this->pimpl, contact.collision1),
                              ShapePtrType(this->pimpl, contact.collision2),
                              contact.point};

    // Note: Using emplace_back and using Get on the resulting reference seems
    // to be the only way to add contacts into the vector. Using push_back like
    // the following does not work:
    //   Contact contactOutput
    //   contactOutput.template Get<ContactPoint>() = std::move(contactPoint);
    //   output.push_back(contactOutput);
    //
    auto &contactOutput = output.emplace_back();
    contactOutput.template Get<ContactPoint>() = std::move(contactPoint);

    auto *extraContactData =
        contact.extraData.template Query<ExtraContactData>();

    if (extraContactData)
    {
      contactOutput.template Get<ExtraContactData>() =
          std::move(*extraContactData);
    }
  }
  return output;
}

}  // namespace physics
}  // namespace ignition

#endif
