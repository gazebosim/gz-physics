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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_SIMULATIONFEATURES_HH_
#define IGNITION_PHYSICS_DARTSIM_SRC_SIMULATIONFEATURES_HH_

#include <memory>
#include <string>
#include <vector>

#include <dart/constraint/ContactSurface.hpp>

#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/GetContacts.hh>
#include <ignition/physics/ContactJointProperties.hh>

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
  GetContactsFromLastStepFeature,
  SetContactJointPropertiesCallbackFeature
> { };

class IgnContactSurfaceHandler : public dart::constraint::ContactSurfaceHandler
{
  public: dart::constraint::ContactSurfaceParams createParams(
    const dart::collision::Contact& _contact,
    size_t _numContactsOnCollisionObject) const override;

  public: dart::constraint::ContactConstraintPtr createConstraint(
    dart::collision::Contact& _contact,
    size_t _numContactsOnCollisionObject,
    double _timeStep) const override;

  public: typedef SetContactJointPropertiesCallbackFeature Feature;
  public: typedef Feature::Implementation<FeaturePolicy3d> Impl;

  public: Impl::SurfaceParamsCallback surfaceParamsCallback;

  public: std::function<
  std::optional<Impl::ContactImpl>(const dart::collision::Contact&)
  > convertContact;

  public: mutable typename Feature::ContactSurfaceParams<FeaturePolicy3d>
  lastIgnParams;
};
using IgnContactSurfaceHandlerPtr = std::shared_ptr<IgnContactSurfaceHandler>;

class SimulationFeatures :
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

  public: std::vector<ContactInternal> GetContactsFromLastStep(
      const Identity &_worldID) const override;

  public: void AddContactJointPropertiesCallback(
      const Identity &_worldID,
      const std::string &_callbackID,
      SurfaceParamsCallback _callback) override;

  public: bool RemoveContactJointPropertiesCallback(
      const Identity &_worldID, const std::string &_callbackID) override;

  private: std::optional<ContactInternal> convertContact(
    const dart::collision::Contact& _contact) const;

  private: std::unordered_map<
    std::string, IgnContactSurfaceHandlerPtr> contactSurfaceHandlers;
};



}
}
}

#endif
