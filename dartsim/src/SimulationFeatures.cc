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

#include <gz/common/Profiler.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/eigen3/Conversions.hh>

#include "gz/physics/GetContacts.hh"

#include "SimulationFeatures.hh"

#if DART_VERSION_AT_LEAST(6, 13, 0)
// The ContactSurfaceParams class was first added to version 6.10 of our fork
// of dart, and then merged upstream and released in version 6.13.0 with
// different public member variable names.
// See https://github.com/dartsim/dart/pull/1626 and
// https://github.com/gazebo-forks/dart/pull/22 for more info.
#define DART_HAS_UPSTREAM_FRICTION_VARIABLE_NAMES
// There's a sign difference between our fork and upstream dart in the
// implementation of surface velocities. Here, we assume that 6.13 is the
// upstream version. There's a potential that a user might update our fork to
// version 6.13.0 or later. To support that case, we'll assume
// DART_HAS_POSITIVE_CONTACT_SURFACE_MOTION_VELOCITY will be defined in the
// fork. Note, if we simply check for
// `DART_HAS_POSITIVE_CONTACT_SURFACE_MOTION_VELOCITY ` without the 6.13
// condition above, we might break users that are building with our 6.10 fork
// without updating or adding the
// DART_HAS_POSITIVE_CONTACT_SURFACE_MOTION_VELOCITY define in their version of
// dart.
#ifndef DART_HAS_POSITIVE_CONTACT_SURFACE_MOTION_VELOCITY
  #define DART_HAS_NEGATIVE_CONTACT_SURFACE_MOTION_VELOCITY
#endif
#endif

namespace gz {
namespace physics {
namespace dartsim {

void SimulationFeatures::WorldForwardStep(
    const Identity &_worldID,
    ForwardStep::Output & _h,
    ForwardStep::State & /*_x*/,
    const ForwardStep::Input & _u)
{
  GZ_PROFILE("SimulationFeatures::WorldForwardStep");
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
      gzdbg << "Simulation timestep set to: " << world->getTimeStep()
             << std::endl;
    }
  }

  for (const auto &[id, info] : this->links.idToObject)
  {
    if (info && info->inertial->FluidAddedMass().has_value())
    {
      auto com = Eigen::Vector3d(info->inertial->Pose().Pos().X(),
                                 info->inertial->Pose().Pos().Y(),
                                 info->inertial->Pose().Pos().Z());

      auto mass = info->inertial->MassMatrix().Mass();
      auto g = world->getGravity();

      info->link->addExtForce(mass * g, com, false, true);
    }
  }

  // TODO(MXG): Parse input
  world->step();
  this->WriteRequiredData(_h);
  this->Write(_h.Get<ChangedWorldPoses>());
  // TODO(MXG): Fill in state
}

void SimulationFeatures::Write(WorldPoses &_worldPoses) const
{
  // remove link poses from the previous iteration
  _worldPoses.entries.clear();
  _worldPoses.entries.reserve(this->links.size());

  for (const auto &[id, info] : this->links.idToObject)
  {
    WorldPose wp;
    wp.pose = gz::math::eigen3::convert(
        info->link->getWorldTransform());
    wp.body = id;
    _worldPoses.entries.push_back(wp);
  }
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
      wp.pose = gz::math::eigen3::convert(
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

SimulationFeatures::RayIntersection
SimulationFeatures::GetRayIntersectionFromLastStep(
  const Identity &_worldID,
  const LinearVector3d &_from,
  const LinearVector3d &_to) const
{
  auto *const world = this->ReferenceInterface<DartWorld>(_worldID);
  auto collisionDetector = world->getConstraintSolver()->getCollisionDetector();
  auto collisionGroup = world->getConstraintSolver()->getCollisionGroup().get();

  // Perform raycast
  dart::collision::RaycastOption option;
  dart::collision::RaycastResult result;
  collisionDetector->raycast(collisionGroup, _from, _to, option, &result);

  SimulationFeatures::RayIntersection intersection;
  if (result.hasHit())
  {
    // Store intersection data if there is a ray hit
    const auto &firstHit = result.mRayHits[0];
    intersection.point = firstHit.mPoint;
    intersection.normal = firstHit.mNormal;
    intersection.fraction = firstHit.mFraction;
  }
  else
  {
    // Set invalid measurements to NaN according to REP-117
    intersection.point = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    intersection.normal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    intersection.fraction = std::numeric_limits<double>::quiet_NaN();
  }

  return intersection;
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

  auto handler = std::make_shared<GzContactSurfaceHandler>();
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
    gzerr << "Could not find the contact surface handler to be removed"
           << std::endl;
    return false;
  }
}

dart::constraint::ContactSurfaceParams GzContactSurfaceHandler::createParams(
  const dart::collision::Contact& _contact,
  const size_t _numContactsOnCollisionObject) const
{
  auto pDart = ContactSurfaceHandler::createParams(
    _contact, _numContactsOnCollisionObject);

  if (!this->surfaceParamsCallback)
    return pDart;

  typedef SetContactPropertiesCallbackFeature F;
  typedef FeaturePolicy3d P;
  typename F::ContactSurfaceParams<P> pGz;

  auto convertMotionVelocity = [](Eigen::Vector3d _input) {
#ifdef DART_HAS_NEGATIVE_CONTACT_SURFACE_MOTION_VELOCITY
    // The y and z components correspond to the velocities in the first and
    // second friction directions. These have to be inverted in the upstream
    // version of DART. https://github.com/gazebo-forks/dart/pull/33
    _input.y() = -_input.y();
    _input.z() = -_input.z();
#endif
    return _input;
  };

#ifdef DART_HAS_UPSTREAM_FRICTION_VARIABLE_NAMES
  pGz.frictionCoeff = pDart.mPrimaryFrictionCoeff;
#else
  pGz.frictionCoeff = pDart.mFrictionCoeff;
#endif
  pGz.secondaryFrictionCoeff = pDart.mSecondaryFrictionCoeff;
#ifdef DART_HAS_UPSTREAM_FRICTION_VARIABLE_NAMES
  pGz.slipCompliance = pDart.mPrimarySlipCompliance;
#else
  pGz.slipCompliance = pDart.mSlipCompliance;
#endif
  pGz.secondarySlipCompliance = pDart.mSecondarySlipCompliance;
  pGz.restitutionCoeff = pDart.mRestitutionCoeff;
  pGz.firstFrictionalDirection = pDart.mFirstFrictionalDirection;
  pGz.contactSurfaceMotionVelocity =
      convertMotionVelocity(pDart.mContactSurfaceMotionVelocity);

  auto contactInternal = this->convertContact(_contact);
  if (contactInternal)
  {
    this->surfaceParamsCallback(contactInternal.value(),
                                _numContactsOnCollisionObject, pGz);

    if (pGz.frictionCoeff)
    {
#ifdef DART_HAS_UPSTREAM_FRICTION_VARIABLE_NAMES
      pDart.mPrimaryFrictionCoeff = pGz.frictionCoeff.value();
#else
      pDart.mFrictionCoeff = pGz.frictionCoeff.value();
#endif
    }
    if (pGz.secondaryFrictionCoeff)
      pDart.mSecondaryFrictionCoeff = pGz.secondaryFrictionCoeff.value();
    if (pGz.slipCompliance)
    {
#ifdef DART_HAS_UPSTREAM_FRICTION_VARIABLE_NAMES
      pDart.mPrimarySlipCompliance = pGz.slipCompliance.value();
#else
      pDart.mSlipCompliance = pGz.slipCompliance.value();
#endif
    }
    if (pGz.secondarySlipCompliance)
      pDart.mSecondarySlipCompliance = pGz.secondarySlipCompliance.value();
    if (pGz.restitutionCoeff)
      pDart.mRestitutionCoeff = pGz.restitutionCoeff.value();
    if (pGz.firstFrictionalDirection)
      pDart.mFirstFrictionalDirection = pGz.firstFrictionalDirection.value();
    if (pGz.contactSurfaceMotionVelocity)
    {
      pDart.mContactSurfaceMotionVelocity =
          convertMotionVelocity(pGz.contactSurfaceMotionVelocity.value());
    }

    static bool warnedRollingFrictionCoeff = false;
    if (!warnedRollingFrictionCoeff && pGz.rollingFrictionCoeff)
    {
      gzwarn << "DART doesn't support rolling friction setting" << std::endl;
      warnedRollingFrictionCoeff = true;
    }

    static bool warnedSecondaryRollingFrictionCoeff = false;
    if (!warnedSecondaryRollingFrictionCoeff &&
      pGz.secondaryRollingFrictionCoeff)
    {
      gzwarn << "DART doesn't support secondary rolling friction setting"
              << std::endl;
      warnedSecondaryRollingFrictionCoeff = true;
    }

    static bool warnedTorsionalFrictionCoeff = false;
    if (!warnedTorsionalFrictionCoeff && pGz.torsionalFrictionCoeff)
    {
      gzwarn << "DART doesn't support torsional friction setting"
              << std::endl;
      warnedTorsionalFrictionCoeff = true;
    }
  }

  this->lastGzParams = pGz;

  return pDart;
}

dart::constraint::ContactConstraintPtr
GzContactSurfaceHandler::createConstraint(
  dart::collision::Contact& _contact,
  const size_t _numContactsOnCollisionObject,
  const double _timeStep) const
{
  // this call sets this->lastGzParams
  auto constraint = dart::constraint::ContactSurfaceHandler::createConstraint(
    _contact, _numContactsOnCollisionObject, _timeStep);

  typedef SetContactPropertiesCallbackFeature F;
  typedef FeaturePolicy3d P;
  typename F::ContactSurfaceParams<P>& p = this->lastGzParams;

  if (this->lastGzParams.errorReductionParameter)
    constraint->setErrorReductionParameter(p.errorReductionParameter.value());

  if (this->lastGzParams.maxErrorAllowance)
    constraint->setErrorAllowance(p.maxErrorAllowance.value());

  if (this->lastGzParams.maxErrorReductionVelocity)
    constraint->setMaxErrorReductionVelocity(
      p.maxErrorReductionVelocity.value());

  if (this->lastGzParams.constraintForceMixing)
    constraint->setConstraintForceMixing(p.constraintForceMixing.value());

  return constraint;
}
#endif

}
}
}
