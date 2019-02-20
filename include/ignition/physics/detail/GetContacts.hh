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

#ifndef IGNITION_PHYSICS_DETAIL_GETCONTACTS_HH_
#define IGNITION_PHYSICS_DETAIL_GETCONTACTS_HH_

#include <vector>
#include <ignition/physics/GetContacts.hh>

namespace ignition
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto GetContactsFromLastStepFeature::World<
    PolicyT, FeaturesT>::GetContactsFromLastStep() const -> std::vector<Contact>
{
  auto contactInternal =
      this->template Interface<GetContactsFromLastStepFeature>()
          ->GetContactsFromLastStep(this->identity);
  // contactInternal provides the identities of collision1 and collision2. Here,
  // we create a new vector and create ShapePtrs out of those identities. Seems
  // inefficient, but I don't know if there's a better way to handle this.
  std::vector<Contact> output;
  output.reserve(contactInternal.size());
  for (auto &contact : contactInternal)
  {
    output.push_back({ShapePtrType(this->pimpl, contact.collision1),
                      ShapePtrType(this->pimpl, contact.collision2),
                      contact.point});
  }
  return output;
}

}  // namespace physics
}  // namespace ignition

#endif
