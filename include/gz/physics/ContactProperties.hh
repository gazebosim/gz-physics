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

#ifndef GZ_PHYSICS_CONTACTPROPERTIES_HH_
#define GZ_PHYSICS_CONTACTPROPERTIES_HH_

#include <string>
#include <vector>

#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/Geometry.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/SpecifyData.hh>

namespace gz
{
namespace physics
{
/// \brief SetContactPropertiesCallbackFeature is a feature for setting the
/// properties of a contact after it is created but before it affects the
/// forward step.
class IGNITION_PHYSICS_VISIBLE SetContactPropertiesCallbackFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  /// \brief This struct gets filled by the simulator and contains various
  /// properties of a contact joint (surface, constraint). All of the values
  /// are optional, which means that they are only filled if the physics engine
  /// supports them. Some originally unfilled values may still be processed by
  /// the physics engine if they are set - this just means there is no default
  /// value for them.
  public: template <typename PolicyT> struct ContactSurfaceParams
  {
    /// \brief Coefficient of friction along the 1st friction direction.
    std::optional<typename PolicyT::Scalar> frictionCoeff;

    /// \brief Coefficient of friction along the 2nd friction direction.
    std::optional<typename PolicyT::Scalar> secondaryFrictionCoeff;

    /// \brief Coefficient of rolling friction along the 1st friction direction.
    std::optional<typename PolicyT::Scalar> rollingFrictionCoeff;

    /// \brief Coefficient of rolling friction along the 2nd friction direction.
    std::optional<typename PolicyT::Scalar> secondaryRollingFrictionCoeff;

    /// \brief Coefficient of torsional friction.
    std::optional<typename PolicyT::Scalar> torsionalFrictionCoeff;

    /// \brief Force-dependent slip coefficient along the 1st friction
    /// direction.
    std::optional<typename PolicyT::Scalar> slipCompliance;

    /// \brief Force-dependent slip coefficient along the 2nd friction
    /// direction.
    std::optional<typename PolicyT::Scalar> secondarySlipCompliance;

    /// \brief Defines the bounciness of the contact. 0 is not bouncy. Values
    /// between 0 and 1 are allowed.
    std::optional<typename PolicyT::Scalar> restitutionCoeff;

    /// \brief The first frictional direction. It should be perpendicular to the
    /// contact normal. The second frictional direction can be computed as a
    /// vector perpendicular both to the normal and to the first direction.
    std::optional<typename FromPolicy<PolicyT>::template Use<Vector>>
      firstFrictionalDirection;

    /// \brief Desired velocity of the colliding bodies in the contact point.
    /// Setting this to non-zero asks the physics engine to add such forces
    /// that can achieve that the colliding bodies have the specified velocity.
    /// The X component specifies velocity along 1st friction direction.
    /// The Y component specifies velocity along 2nd friction direction.
    /// The Z component specifies velocity along the contact normal.
    std::optional<typename FromPolicy<PolicyT>::template Use<Vector>>
      contactSurfaceMotionVelocity;

    /// \brief Joint error reduction parameter. This is the fraction of the
    /// joint error that will be attempted to be corrected in each simulation
    /// step. Allowed values are 0 to 1. Default is usually somewhere between.
    std::optional<typename PolicyT::Scalar> errorReductionParameter;

    /// \brief Maximum velocity that can be used to reduce joint error.
    std::optional<typename PolicyT::Scalar> maxErrorReductionVelocity;

    /// \brief Maximum joint error for which no error reduction is performed.
    std::optional<typename PolicyT::Scalar> maxErrorAllowance;

    /// \brief Constraint force mixing. This should be a non-negative number.
    /// If greater than 0, this number is added to the diagonal of the system
    /// matrix making the contact softer and the solution more stable.
    std::optional<typename PolicyT::Scalar> constraintForceMixing;
  };

  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using ShapePtrType = typename GetContactsFromLastStepFeature
      ::World<PolicyT, FeaturesT>::ShapePtrType;

    /// \brief This callback is called for every detected contact point and
    /// allows customizing properties of the contact surface.
    /// \param _contact[in] The contact object containing contact point,
    ///                     normal, force etc. Please note that the force will
    ///                     be always zero because the forward step has not yet
    ///                     been run to compute the force.
    /// \param _numContactsOnCollision[in] Number of contact points on the same
    ///                                    collision object. This can be used
    ///                                    e.g. for force normalization.
    /// \param _surfaceParams[in,out] Parameters of the contact surface. They
    ///                               are pre-filled by the physics engine and
    ///                               the callback can alter them.
    public: typedef std::function<
        void(
          const typename GetContactsFromLastStepFeature::
            World<PolicyT, FeaturesT>::Contact& /*_contact*/,
          size_t /*_numContactsOnCollision*/,
          ContactSurfaceParams<PolicyT>& /*_surfaceParams*/)
      > SurfaceParamsCallback;

    /// \brief Add the callback.
    public: void AddContactPropertiesCallback(
      const std::string &_callbackID, SurfaceParamsCallback _callback);

    /// \brief Remove the callback.
    public: bool RemoveContactPropertiesCallback(
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
    public: virtual void AddContactPropertiesCallback(
      const Identity &_worldID,
      const std::string &_callbackID,
      SurfaceParamsCallback _callback) = 0;

    /// \brief Remove the callback.
    public: virtual bool RemoveContactPropertiesCallback(
      const Identity &_worldID, const std::string &_callbackID) = 0;
  };
};

}
}

#include "gz/physics/detail/ContactProperties.hh"

#endif /* end of include guard: GZ_PHYSICS_CONTACTPROPERTIES_HH_ */
