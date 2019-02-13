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

#ifndef IGNITION_PHYSICS_GETCONTACTS_HH_
#define IGNITION_PHYSICS_GETCONTACTS_HH_

#include <vector>
#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
namespace physics
{
/// \brief GetContactsFromLastStepFeature is a feature for retrieving the list
/// of contacts generated in the previous simulation step.
class IGNITION_PHYSICS_VISIBLE GetContactsFromLastStepFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using ShapePtrType = ShapePtr<PolicyT, FeaturesT>;
    public: using VectorType =
        typename FromPolicy<PolicyT>::template Use<Vector>;

    public: struct Contact
    {
      ShapePtrType collision1;
      ShapePtrType collision2;
      VectorType point;
    };

    /// \brief Get contacts generated in the previous simulation step
    public: std::vector<Contact> GetContactsFromLastStep() const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: using VectorType =
        typename FromPolicy<PolicyT>::template Use<Vector>;

    public: struct ContactInternal
    {
      Identity collision1;
      Identity collision2;
      VectorType point;
    };

    public: virtual std::vector<ContactInternal> GetContactsFromLastStep(
        const Identity &_worldID) const = 0;
  };
  };
}
}

#include "ignition/physics/detail/GetContacts.hh"

#endif /* end of include guard: IGNITION_PHYSICS_GETCONTACTS_HH_ */
