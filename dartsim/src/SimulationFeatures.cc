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

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/CollisionResult.hpp>

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

  // Compute ID to populate the joint forces
  for (size_t i = 0; i < world->getNumSkeletons(); ++i) {
    world->getSkeleton(i)->computeInverseDynamics(
      /*_withExternalForces=*/true,
      /*_withDampingForces=*/true,
      /*_withSpringForces=*/true);
  }
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
