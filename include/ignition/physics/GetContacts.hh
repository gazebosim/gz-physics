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
#include <ignition/physics/SpecifyData.hh>

namespace ignition
{
namespace physics
{
/// \brief GetContactsFromLastStepFeature is a feature for retrieving the list
/// of contacts generated in the previous simulation step.
class IGNITION_PHYSICS_VISIBLE GetContactsFromLastStepFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  public: template <typename VectorType, typename Scalar>
  struct ExtraContactDataT
  {
    /// \brief The contact force from body acting on the first body
    /// expressed in the world frame
    VectorType force;
    /// \brief The normal of the force from the second body to the first
    /// body expressed in the world frame
    VectorType normal;
    /// \brief The penetration depth
    Scalar depth;
  };

  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using Scalar = typename PolicyT::Scalar;
    public: using ShapePtrType = ShapePtr<PolicyT, FeaturesT>;
    public: using VectorType =
        typename FromPolicy<PolicyT>::template Use<Vector>;
    public: using ExtraContactData = ExtraContactDataT<VectorType, Scalar>;

    public: struct ContactPoint
    {
      /// \brief Collision shape of the first body
      ShapePtrType collision1;
      /// \brief Collision shape of the second body
      ShapePtrType collision2;
      /// \brief The point of contact expressed in the world frame
      VectorType point;
    };

    public: using Contact = SpecifyData<
            RequireData<ContactPoint>,
            ExpectData<ExtraContactData> >;

    /// \brief Get contacts generated in the previous simulation step
    public: std::vector<Contact> GetContactsFromLastStep() const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: using Scalar = typename PolicyT::Scalar;
    public: using VectorType =
        typename FromPolicy<PolicyT>::template Use<Vector>;
    public: using ExtraContactData = ExtraContactDataT<VectorType, Scalar>;

    public: struct ContactInternal
    {
      /// \brief Identity of the first body
      Identity collision1;
      /// \brief Identity of the second body
      Identity collision2;
      /// \brief The point of contact expressed in the world frame
      VectorType point;
      /// \brief Extra data related to contact.
      CompositeData extraData;
    };

    public: virtual std::vector<ContactInternal> GetContactsFromLastStep(
        const Identity &_worldID) const = 0;
  };
};
}
}

#include "ignition/physics/detail/GetContacts.hh"

#endif /* end of include guard: IGNITION_PHYSICS_GETCONTACTS_HH_ */
