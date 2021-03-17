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

#include <unordered_map>
#include <utility>

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/CollisionResult.hpp>

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
  if (!world)
  {
    ignerr << "World with id [" << _worldID.id << "] not found." << std::endl;
    return;
  }
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
  this->WriteRequiredData(_h);
  this->Write(_h.Get<JointPositions>());
  // TODO(MXG): Fill in state
}

void SimulationFeatures::Write(JointPositions &/*_positions*/) const
{
  // TODO(adlarkin) implement this, if it's needed?
}

void SimulationFeatures::Write(WorldPoses &_poses) const
{
  // remove link poses from the previous iteration
  _poses.entries.clear();

  std::unordered_map<std::size_t, math::Pose3d> newPoses;

  for (const auto &link : this->links.idToObject)
  {
    const auto id = link.first;
    const auto info = link.second;

    // make sure the link exists
    if (info && info->link)
    {
      WorldPose wp;
      wp.pose = ignition::math::eigen3::convert(
          info->link->getWorldTransform());
      wp.pose.Pos() = ignition::math::eigen3::convert(
          info->link->getCOM());
      wp.body = id;

      // if the link's pose is new or has changed,
      // add the link to the output poses
      auto iter = this->prevLinkPoses.find(id);
      if ((iter == this->prevLinkPoses.end()) ||
          !iter->second.Pos().Equal(wp.pose.Pos(), 1e-6) ||
          !iter->second.Rot().Equal(wp.pose.Rot(), 1e-6))
      {
        _poses.entries.push_back(wp);
      }

      newPoses[id] = wp.pose;
    }
  }

  // Save the new poses so that they can be used to check for updates in the
  // next iteration. Re-setting this->prevLinkPoses with the contents of
  // newPoses ensures that we aren't caching data for links that were removed
  this->prevLinkPoses.clear();
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
    dart::collision::CollisionObject *dtCollObj1 = dtContact.collisionObject1;
    dart::collision::CollisionObject *dtCollObj2 = dtContact.collisionObject2;

    const dart::dynamics::ShapeFrame *dtShapeFrame1 =
      dtCollObj1->getShapeFrame();
    const dart::dynamics::ShapeFrame *dtShapeFrame2 =
      dtCollObj2->getShapeFrame();

    dart::dynamics::ConstBodyNodePtr dtBodyNode1;
    dart::dynamics::ConstBodyNodePtr dtBodyNode2;

    if (this->shapes.HasEntity(dtShapeFrame1->asShapeNode()) &&
        this->shapes.HasEntity(dtShapeFrame2->asShapeNode()))
    {
      std::size_t shape1ID =
          this->shapes.IdentityOf(dtShapeFrame1->asShapeNode());
      std::size_t shape2ID =
          this->shapes.IdentityOf(dtShapeFrame2->asShapeNode());

      CompositeData extraData;

      // Add normal, depth and wrench to extraData.
      auto &extraContactData = extraData.Get<ExtraContactData>();
      extraContactData.force = dtContact.force;
      extraContactData.normal = dtContact.normal;
      extraContactData.depth = dtContact.penetrationDepth;

      outContacts.push_back(
          {this->GenerateIdentity(shape1ID, this->shapes.at(shape1ID)),
           this->GenerateIdentity(shape2ID, this->shapes.at(shape2ID)),
           dtContact.point, extraData});
    }
  }
  return outContacts;
}
}
}
}
