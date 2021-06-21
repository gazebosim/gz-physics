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
  public: template <typename PolicyT> struct FrictionCoeff
  {
    typename PolicyT::Scalar frictionCoeff;
  };

  public: template <typename PolicyT> struct SecondaryFrictionCoeff
  {
    typename PolicyT::Scalar secondaryFrictionCoeff;
  };

  public: template <typename PolicyT> struct RollingFrictionCoeff
  {
    typename PolicyT::Scalar rollingFrictionCoeff;
  };

  public: template <typename PolicyT> struct SecondaryRollingFrictionCoeff
  {
    typename PolicyT::Scalar secondaryRollingFrictionCoeff;
  };

  public: template <typename PolicyT> struct NormalRollingFrictionCoeff
  {
    typename PolicyT::Scalar normalRollingFrictionCoeff;
  };

  public: template <typename PolicyT> struct SlipCompliance
  {
    typename PolicyT::Scalar slipCompliance;
  };

  public: template <typename PolicyT> struct SecondarySlipCompliance
  {
    typename PolicyT::Scalar secondarySlipCompliance;
  };

  public: template <typename PolicyT> struct RestitutionCoeff
  {
    typename PolicyT::Scalar restitutionCoeff;
  };

  public: template <typename PolicyT> struct FirstFrictionalDirection
  {
    typename FromPolicy<PolicyT>::template Use<Vector> firstFrictionalDirection;
  };

  public: template <typename PolicyT> struct ContactSurfaceMotionVelocity
  {
    typename FromPolicy<PolicyT>::template Use<Vector>
      contactSurfaceMotionVelocity;
  };

  public: template <typename PolicyT> struct ErrorReductionParameter
  {
    typename PolicyT::Scalar errorReductionParameter;
  };

  public: template <typename PolicyT> struct MaxErrorReductionVelocity
  {
    typename PolicyT::Scalar maxErrorReductionVelocity;
  };

  public: template <typename PolicyT> struct MaxErrorAllowance
  {
    typename PolicyT::Scalar maxErrorAllowance;
  };

  public: template <typename PolicyT> struct ConstraintForceMixing
  {
    typename PolicyT::Scalar constraintForceMixing;
  };

  public: template <typename PolicyT> using ContactSurfaceParams =
    SpecifyData<ExpectData<
      FrictionCoeff<PolicyT>, SecondaryFrictionCoeff<PolicyT>,
      RollingFrictionCoeff<PolicyT>, SecondaryRollingFrictionCoeff<PolicyT>,
      NormalRollingFrictionCoeff<PolicyT>,
      SlipCompliance<PolicyT>, SecondarySlipCompliance<PolicyT>,
      RestitutionCoeff<PolicyT>,
      FirstFrictionalDirection<PolicyT>,
      ContactSurfaceMotionVelocity<PolicyT>,
      ErrorReductionParameter<PolicyT>,
      MaxErrorReductionVelocity<PolicyT>,
      MaxErrorAllowance<PolicyT>,
      ConstraintForceMixing<PolicyT> > >;

  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using JointPtrType = JointPtr<PolicyT, FeaturesT>;

    public: using ShapePtrType = typename GetContactsFromLastStepFeature
      ::World<PolicyT, FeaturesT>::ShapePtrType;

    public: using Contact = typename GetContactsFromLastStepFeature
      ::World<PolicyT, FeaturesT>::Contact;

    public: typedef std::function<
        void(const Contact&, size_t, ContactSurfaceParams<PolicyT>&)
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
