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

#include <memory>
#include <string>
#include <unordered_map>

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/ContactSurface.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include "SimulationFeatures.hh"

#include "ignition/common/Profiler.hh"
#include "ignition/physics/GetContacts.hh"

namespace ignition {
namespace physics {
namespace dartsim {

void SimulationFeatures::WorldForwardStep(
    const Identity &_worldID,
    ForwardStep::Output & /*_h*/,
    ForwardStep::State & /*_x*/,
    const ForwardStep::Input & _u)
{
  IGN_PROFILE("SimulationFeatures::WorldForwardStep");
  auto *world = this->ReferenceInterface<DartWorld>(_worldID);
  auto *dtDur =
      _u.Query<std::chrono::steady_clock::duration>();
  const double tol = 1e-6;

  if (dtDur)
  {
    std::chrono::duration<double> dt = *dtDur;
    if (std::fabs(dt.count() - world->getTimeStep()) > tol)
    {
      world->setTimeStep(dt.count());
      igndbg << "Simulation timestep set to: " << world->getTimeStep()
             << std::endl;
    }
  }

  // TODO(MXG): Parse input
  world->step();
  // TODO(MXG): Fill in output and state
}

std::vector<SimulationFeatures::ContactInternal>
SimulationFeatures::GetContactsFromLastStep(const Identity &_worldID) const
{
  std::vector<SimulationFeatures::ContactInternal> outContacts;
  auto *const world = this->ReferenceInterface<DartWorld>(_worldID);
  const auto colResult = world->getLastCollisionResult();

  for (const auto &dtContact : colResult.getContacts())
  {
    auto contact = this->convertContact(dtContact);
    if (contact)
      outContacts.push_back(contact.value());
  }

  return outContacts;
}

void SimulationFeatures::AddContactJointPropertiesCallback(
  const Identity& _worldID, const std::string& _callbackID,
  SurfaceParamsCallback _callback)
{
  auto *world = this->ReferenceInterface<DartWorld>(_worldID);

  auto handler = std::make_shared<IgnContactSurfaceHandler>();
  handler->surfaceParamsCallback = _callback;
  handler->convertContact = [this](const dart::collision::Contact& _contact) {
    return this->convertContact(_contact);
  };

  this->contactSurfaceHandlers[_callbackID] = handler;
  world->getConstraintSolver()->setContactSurfaceHandler(handler);
}

bool SimulationFeatures::RemoveContactJointPropertiesCallback(
  const Identity& _worldID, const std::string& _callbackID)
{
  auto *world = this->ReferenceInterface<DartWorld>(_worldID);

  if (this->contactSurfaceHandlers.find(_callbackID) !=
    this->contactSurfaceHandlers.end())
  {
    const auto handler = this->contactSurfaceHandlers[_callbackID];
    this->contactSurfaceHandlers.erase(_callbackID);
    return world->getConstraintSolver()->removeContactSurfaceHandler(handler);
  }
  else
  {
    ignerr << "Could not find the contact surface handler to be removed"
           << std::endl;
    return false;
  }
}

std::optional<SimulationFeatures::ContactInternal>
SimulationFeatures::convertContact(
  const dart::collision::Contact& _contact) const
{
  auto *dtCollObj1 = _contact.collisionObject1;
  auto *dtCollObj2 = _contact.collisionObject2;

  auto *dtShapeFrame1 = dtCollObj1->getShapeFrame();
  auto *dtShapeFrame2 = dtCollObj2->getShapeFrame();

  if (this->shapes.HasEntity(dtShapeFrame1->asShapeNode()) &&
      this->shapes.HasEntity(dtShapeFrame2->asShapeNode()))
  {
    std::size_t shape1ID =
      this->shapes.IdentityOf(dtShapeFrame1->asShapeNode());
    std::size_t shape2ID =
      this->shapes.IdentityOf(dtShapeFrame2->asShapeNode());

    CompositeData extraData;

    // Add normal, depth and wrench to extraData.
    auto& extraContactData =
      extraData.Get<SimulationFeatures::ExtraContactData>();
    extraContactData.force = _contact.force;
    extraContactData.normal = _contact.normal;
    extraContactData.depth = _contact.penetrationDepth;


    return SimulationFeatures::ContactInternal {
      this->GenerateIdentity(shape1ID, this->shapes.at(shape1ID)),
      this->GenerateIdentity(shape2ID, this->shapes.at(shape2ID)),
      _contact.point, extraData
    };
  }

  return std::nullopt;
}

dart::constraint::ContactSurfaceParams IgnContactSurfaceHandler::createParams(
  const dart::collision::Contact& _contact,
  const size_t _numContactsOnCollisionObject) const
{
  auto pDart = ContactSurfaceHandler::createParams(
    _contact, _numContactsOnCollisionObject);

  if (!this->surfaceParamsCallback)
    return pDart;

  typedef SetContactJointPropertiesCallbackFeature F;
  typedef FeaturePolicy3d P;
  typename F::ContactSurfaceParams<P> pIgn;

  pIgn.Get<F::FrictionCoeff<P>>().frictionCoeff = pDart.mFrictionCoeff;
  pIgn.Get<F::SecondaryFrictionCoeff<P>>().secondaryFrictionCoeff =
    pDart.mSecondaryFrictionCoeff;
  pIgn.Get<F::SlipCompliance<P>>().slipCompliance = pDart.mSlipCompliance;
  pIgn.Get<F::SecondarySlipCompliance<P>>().secondarySlipCompliance =
    pDart.mSecondarySlipCompliance;
  pIgn.Get<F::RestitutionCoeff<P>>().restitutionCoeff =
    pDart.mRestitutionCoeff;
  pIgn.Get<F::FirstFrictionalDirection<P>>().firstFrictionalDirection =
    pDart.mFirstFrictionalDirection;
  pIgn.Get<F::ContactSurfaceMotionVelocity<P>>().contactSurfaceMotionVelocity =
    pDart.mContactSurfaceMotionVelocity;

  auto contactInternal = this->convertContact(_contact);
  if (contactInternal)
  {
    this->surfaceParamsCallback(contactInternal.value(),
                                _numContactsOnCollisionObject, pIgn);

    pDart.mFrictionCoeff = pIgn.Get<F::FrictionCoeff<P>>().frictionCoeff;
    pDart.mSecondaryFrictionCoeff =
      pIgn.Get<F::SecondaryFrictionCoeff<P>>().secondaryFrictionCoeff;
    pDart.mSlipCompliance = pIgn.Get<F::SlipCompliance<P>>().slipCompliance;
    pDart.mSecondarySlipCompliance =
      pIgn.Get<F::SecondarySlipCompliance<P>>().secondarySlipCompliance;
    pDart.mRestitutionCoeff =
      pIgn.Get<F::RestitutionCoeff<P>>().restitutionCoeff;
    pDart.mFirstFrictionalDirection =
      pIgn.Get<F::FirstFrictionalDirection<P>>().firstFrictionalDirection;
    pDart.mContactSurfaceMotionVelocity =
      pIgn.Get<F::ContactSurfaceMotionVelocity<P>>().
        contactSurfaceMotionVelocity;
  }

  this->lastIgnParams = pIgn;

  return pDart;
}

dart::constraint::ContactConstraintPtr
IgnContactSurfaceHandler::createConstraint(
  dart::collision::Contact& _contact,
  const size_t _numContactsOnCollisionObject,
  const double _timeStep) const
{
  // this call sets this->lastIgnParams
  auto constraint = dart::constraint::ContactSurfaceHandler::createConstraint(
    _contact, _numContactsOnCollisionObject, _timeStep);

  typedef SetContactJointPropertiesCallbackFeature F;
  typedef FeaturePolicy3d P;
  typename F::ContactSurfaceParams<P>& p = this->lastIgnParams;

  if (this->lastIgnParams.Has<F::ErrorReductionParameter<P>>())
    constraint->setErrorReductionParameter(
      p.Get<F::ErrorReductionParameter<P>>().errorReductionParameter);

  if (this->lastIgnParams.Has<F::MaxErrorAllowance<P>>())
    constraint->setErrorAllowance(
      p.Get<F::MaxErrorAllowance<P>>().maxErrorAllowance);

  if (this->lastIgnParams.Has<F::MaxErrorReductionVelocity<P>>())
    constraint->setMaxErrorReductionVelocity(
      p.Get<F::MaxErrorReductionVelocity<P>>().maxErrorReductionVelocity);

  if (this->lastIgnParams.Has<F::ConstraintForceMixing<P>>())
    constraint->setConstraintForceMixing(
      p.Get<F::ConstraintForceMixing<P>>().constraintForceMixing);

  return constraint;
}

}
}
}
