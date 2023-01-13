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

#ifndef GZ_PHYSICS_DARTSIM_SRC_SIMULATIONFEATURES_HH_
#define GZ_PHYSICS_DARTSIM_SRC_SIMULATIONFEATURES_HH_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <dart/config.hpp>
#ifdef DART_HAS_CONTACT_SURFACE
#include <dart/constraint/ContactSurface.hpp>
#endif

#include <ignition/math/Pose3.hh>

#include <ignition/physics/CanWriteData.hh>

#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/ContactProperties.hh>
#include <ignition/physics/SpecifyData.hh>

#include "Base.hh"

namespace dart
{
namespace collision
{
class Contact;
}
}

namespace ignition {
namespace physics {
namespace dartsim {

struct SimulationFeatureList : FeatureList<
  ForwardStep,
#ifdef DART_HAS_CONTACT_SURFACE
  SetContactPropertiesCallbackFeature,
#endif
  GetContactsFromLastStepFeature
> { };

#ifdef DART_HAS_CONTACT_SURFACE
class IgnContactSurfaceHandler : public dart::constraint::ContactSurfaceHandler
{
  public: dart::constraint::ContactSurfaceParams createParams(
    const dart::collision::Contact& _contact,
    size_t _numContactsOnCollisionObject) const override;

  public: dart::constraint::ContactConstraintPtr createConstraint(
    dart::collision::Contact& _contact,
    size_t _numContactsOnCollisionObject,
    double _timeStep) const override;

  public: typedef SetContactPropertiesCallbackFeature Feature;
  public: typedef Feature::Implementation<FeaturePolicy3d> Impl;

  public: Impl::SurfaceParamsCallback surfaceParamsCallback;

  public: std::function<
  std::optional<Impl::ContactImpl>(const dart::collision::Contact&)
  > convertContact;

  public: mutable typename Feature::ContactSurfaceParams<FeaturePolicy3d>
  lastIgnParams;
};

using IgnContactSurfaceHandlerPtr = std::shared_ptr<IgnContactSurfaceHandler>;
#endif

class SimulationFeatures :
    public CanWriteExpectedData<SimulationFeatures,
      ExpectData<ChangedWorldPoses>>,
    public virtual Base,
    public virtual Implements3d<SimulationFeatureList>
{
  public: using GetContactsFromLastStepFeature::Implementation<FeaturePolicy3d>
    ::ContactInternal;

  public: SimulationFeatures() = default;
  public: ~SimulationFeatures() override = default;

  public: void WorldForwardStep(
      const Identity &_worldID,
      ForwardStep::Output &_h,
      ForwardStep::State &_x,
      const ForwardStep::Input &_u) override;

  public: void Write(ChangedWorldPoses &_changedPoses) const;

  public: std::vector<ContactInternal> GetContactsFromLastStep(
      const Identity &_worldID) const override;

  /// \brief link poses from the most recent pose change/update.
  /// The key is the link's ID, and the value is the link's pose
  private: mutable std::unordered_map<std::size_t, math::Pose3d> prevLinkPoses;

  private: std::optional<ContactInternal> convertContact(
    const dart::collision::Contact& _contact) const;

#ifdef DART_HAS_CONTACT_SURFACE
  public: void AddContactPropertiesCallback(
      const Identity &_worldID,
      const std::string &_callbackID,
      SurfaceParamsCallback _callback) override;

  public: bool RemoveContactPropertiesCallback(
      const Identity &_worldID, const std::string &_callbackID) override;

  private: std::unordered_map<
    std::string, IgnContactSurfaceHandlerPtr> contactSurfaceHandlers;
#endif
};

}
}
}

#endif
