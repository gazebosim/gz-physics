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
#include <utility>


#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#ifdef DART_HAS_CONTACT_SURFACE
#include <dart/constraint/ContactSurface.hpp>
#endif

#include <ignition/common/Profiler.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/eigen3/Conversions.hh>

#include "ignition/physics/GetContacts.hh"

#include "SimulationFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

void SimulationFeatures::WorldForwardStep(
    const Identity &_worldID,
    ForwardStep::Output & _h,
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
  this->Write(_h.Get<ChangedWorldPoses>());
  // TODO(MXG): Fill in state
}

void SimulationFeatures::Write(ChangedWorldPoses &_changedPoses) const
{
  // remove link poses from the previous iteration
  _changedPoses.entries.clear();
  _changedPoses.entries.reserve(this->links.size());

  std::unordered_map<std::size_t, math::Pose3d> newPoses;

  for (const auto &[id, info] : this->links.idToObject)
  {
    // make sure the link exists
    if (info && info->link)
    {
      WorldPose wp;
      wp.pose = ignition::math::eigen3::convert(
          info->link->getWorldTransform());
      wp.body = id;

      // If the link's pose is new or has changed, save this new pose and
      // add it to the output poses. Otherwise, keep the existing link pose
      auto iter = this->prevLinkPoses.find(id);
      if ((iter == this->prevLinkPoses.end()) ||
          !iter->second.Pos().Equal(wp.pose.Pos(), 1e-6) ||
          !iter->second.Rot().Equal(wp.pose.Rot(), 1e-6))
      {
        _changedPoses.entries.push_back(wp);
        newPoses[id] = wp.pose;
      }
      else
        newPoses[id] = iter->second;
    }
  }

  // Save the new poses so that they can be used to check for updates in the
  // next iteration. Re-setting this->prevLinkPoses with the contents of
  // newPoses ensures that we aren't caching data for links that were removed
  this->prevLinkPoses = std::move(newPoses);
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

#ifdef DART_HAS_CONTACT_SURFACE
void SimulationFeatures::AddContactPropertiesCallback(
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
  world->getConstraintSolver()->addContactSurfaceHandler(handler);
}

bool SimulationFeatures::RemoveContactPropertiesCallback(
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

dart::constraint::ContactSurfaceParams IgnContactSurfaceHandler::createParams(
  const dart::collision::Contact& _contact,
  const size_t _numContactsOnCollisionObject) const
{
  auto pDart = ContactSurfaceHandler::createParams(
    _contact, _numContactsOnCollisionObject);

  if (!this->surfaceParamsCallback)
    return pDart;

  typedef SetContactPropertiesCallbackFeature F;
  typedef FeaturePolicy3d P;
  typename F::ContactSurfaceParams<P> pIgn;

  pIgn.frictionCoeff = pDart.mFrictionCoeff;
  pIgn.secondaryFrictionCoeff = pDart.mSecondaryFrictionCoeff;
  pIgn.slipCompliance = pDart.mSlipCompliance;
  pIgn.secondarySlipCompliance = pDart.mSecondarySlipCompliance;
  pIgn.restitutionCoeff = pDart.mRestitutionCoeff;
  pIgn.firstFrictionalDirection = pDart.mFirstFrictionalDirection;
  pIgn.contactSurfaceMotionVelocity = pDart.mContactSurfaceMotionVelocity;

  auto contactInternal = this->convertContact(_contact);
  if (contactInternal)
  {
    this->surfaceParamsCallback(contactInternal.value(),
                                _numContactsOnCollisionObject, pIgn);

    if (pIgn.frictionCoeff)
      pDart.mFrictionCoeff = pIgn.frictionCoeff.value();
    if (pIgn.secondaryFrictionCoeff)
      pDart.mSecondaryFrictionCoeff = pIgn.secondaryFrictionCoeff.value();
    if (pIgn.slipCompliance)
      pDart.mSlipCompliance = pIgn.slipCompliance.value();
    if (pIgn.secondarySlipCompliance)
      pDart.mSecondarySlipCompliance = pIgn.secondarySlipCompliance.value();
    if (pIgn.restitutionCoeff)
      pDart.mRestitutionCoeff = pIgn.restitutionCoeff.value();
    if (pIgn.firstFrictionalDirection)
      pDart.mFirstFrictionalDirection = pIgn.firstFrictionalDirection.value();
    if (pIgn.contactSurfaceMotionVelocity)
      pDart.mContactSurfaceMotionVelocity =
        pIgn.contactSurfaceMotionVelocity.value();

    static bool warnedRollingFrictionCoeff = false;
    if (!warnedRollingFrictionCoeff && pIgn.rollingFrictionCoeff)
    {
      ignwarn << "DART doesn't support rolling friction setting" << std::endl;
      warnedRollingFrictionCoeff = true;
    }

    static bool warnedSecondaryRollingFrictionCoeff = false;
    if (!warnedSecondaryRollingFrictionCoeff &&
      pIgn.secondaryRollingFrictionCoeff)
    {
      ignwarn << "DART doesn't support secondary rolling friction setting"
              << std::endl;
      warnedSecondaryRollingFrictionCoeff = true;
    }

    static bool warnedTorsionalFrictionCoeff = false;
    if (!warnedTorsionalFrictionCoeff && pIgn.torsionalFrictionCoeff)
    {
      ignwarn << "DART doesn't support torsional friction setting"
              << std::endl;
      warnedTorsionalFrictionCoeff = true;
    }
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

  typedef SetContactPropertiesCallbackFeature F;
  typedef FeaturePolicy3d P;
  typename F::ContactSurfaceParams<P>& p = this->lastIgnParams;

  if (this->lastIgnParams.errorReductionParameter)
    constraint->setErrorReductionParameter(p.errorReductionParameter.value());

  if (this->lastIgnParams.maxErrorAllowance)
    constraint->setErrorAllowance(p.maxErrorAllowance.value());

  if (this->lastIgnParams.maxErrorReductionVelocity)
    constraint->setMaxErrorReductionVelocity(
      p.maxErrorReductionVelocity.value());

  if (this->lastIgnParams.constraintForceMixing)
    constraint->setConstraintForceMixing(p.constraintForceMixing.value());

  return constraint;
}
#endif

}
}
}
