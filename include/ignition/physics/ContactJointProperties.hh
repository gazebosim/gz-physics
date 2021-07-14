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

#ifndef IGNITION_PHYSICS_CONTACTJOINTPROPERTIES_HH_
#define IGNITION_PHYSICS_CONTACTJOINTPROPERTIES_HH_

#include <string>
#include <vector>

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/Geometry.hh>
#include <ignition/physics/GetContacts.hh>
#include <ignition/physics/SpecifyData.hh>

namespace ignition
{
namespace physics
{
/// \brief SetContactJointPropertiesCallbackFeature is a feature for setting the
/// properties of a contact joint after it is created but before it affects the
/// forward step.
class IGNITION_PHYSICS_VISIBLE SetContactJointPropertiesCallbackFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  public: template <typename PolicyT> struct ContactSurfaceParams
  {
    std::optional<typename PolicyT::Scalar> frictionCoeff;
    std::optional<typename PolicyT::Scalar> secondaryFrictionCoeff;
    std::optional<typename PolicyT::Scalar> rollingFrictionCoeff;
    std::optional<typename PolicyT::Scalar> secondaryRollingFrictionCoeff;
    std::optional<typename PolicyT::Scalar> normalRollingFrictionCoeff;
    std::optional<typename PolicyT::Scalar> slipCompliance;
    std::optional<typename PolicyT::Scalar> secondarySlipCompliance;
    std::optional<typename PolicyT::Scalar> restitutionCoeff;
    std::optional<typename FromPolicy<PolicyT>::template Use<Vector>>
      firstFrictionalDirection;
    std::optional<typename FromPolicy<PolicyT>::template Use<Vector>>
      contactSurfaceMotionVelocity;
    std::optional<typename PolicyT::Scalar> errorReductionParameter;
    std::optional<typename PolicyT::Scalar> maxErrorReductionVelocity;
    std::optional<typename PolicyT::Scalar> maxErrorAllowance;
    std::optional<typename PolicyT::Scalar> constraintForceMixing;
  };

  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using JointPtrType = JointPtr<PolicyT, FeaturesT>;

    public: using ShapePtrType = typename GetContactsFromLastStepFeature
      ::World<PolicyT, FeaturesT>::ShapePtrType;

    public: using Contact = typename GetContactsFromLastStepFeature
      ::World<PolicyT, FeaturesT>::Contact;

    /// \brief This callback is called for every detected contact point and
    /// allows customizing properties of the contact surface.
    /// \param _contact[in] The contact object containint contact point,
    ///                     normal, force etc.
    /// \param _numContactsOnCollision[in] Number of contact points on the same
    ///                                    collision object. This can be used
    ///                                    e.g. for force normalization.
    /// \param _surfaceParams[in,out] Parameters of the contact surface. They
    ///                               are pre-filled by the physics engine and
    ///                               the callback can alter them.
    public: typedef std::function<
        void(
          const Contact& /*_contact*/,
          size_t /*_numContactsOnCollision*/,
          ContactSurfaceParams<PolicyT>& /*_surfaceParams*/)
      > SurfaceParamsCallback;

    /// \brief Add the callback.
    public: void AddContactJointPropertiesCallback(
      const std::string &_callbackID, SurfaceParamsCallback _callback);

    /// \brief Remove the callback.
    public: bool RemoveContactJointPropertiesCallback(
      const std::string &_callbackID);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: using ContactImpl = typename GetContactsFromLastStepFeature
      ::Implementation<PolicyT>::ContactInternal;

    public: typedef std::function<
        void(const ContactImpl&, size_t, ContactSurfaceParams<PolicyT>&)
      > SurfaceParamsCallback;

    /// \brief Add the callback.
    public: virtual void AddContactJointPropertiesCallback(
      const Identity &_worldID,
      const std::string &_callbackID,
      SurfaceParamsCallback _callback) = 0;

    /// \brief Remove the callback.
    public: virtual bool RemoveContactJointPropertiesCallback(
      const Identity &_worldID, const std::string &_callbackID) = 0;
  };
};

}
}

#include "ignition/physics/detail/ContactJointProperties.hh"

#endif /* end of include guard: IGNITION_PHYSICS_CONTACTJOINTPROPERTIES_HH_ */
