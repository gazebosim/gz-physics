/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "SimulationFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

void SimulationFeatures::WorldForwardStep(
    const Identity &_worldID,
    ForwardStep::Output & /*_h*/,
    ForwardStep::State & /*_x*/,
    const ForwardStep::Input & _u)
{
    const WorldInfoPtr &worldInfo = this->worlds.at(_worldID);
    this->UpdateJoints();
    auto *dtDur =
      _u.Query<std::chrono::steady_clock::duration>();
    std::chrono::duration<double> dt = *dtDur;
    worldInfo->world->stepSimulation(dt.count(), 1, dt.count());
}

void SimulationFeatures::UpdateJoints()
{
  for (auto &jointId : this->damping_joints)
  {
    double result = ignition::math::NAN_D;
    auto jointInfo = this->joints.at(jointId);
    double damping = jointInfo->damping;
    btHingeAccumulatedAngleConstraint* hinge =
      static_cast<btHingeAccumulatedAngleConstraint*>(jointInfo->joint);
    if (hinge)
    {
      result = 0.0;
      // Get the axis of the joint
      btVector3 vec =
        hinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
        hinge->getFrameOffsetA().getBasis().getColumn(2);

      math::Vector3 globalAxis(vec[0], vec[1], vec[2]);

      if (this->links.find(jointInfo->childLinkId) != this->links.end())
      {
        btRigidBody *childLink = this->links.at(jointInfo->childLinkId)->link;
        btVector3 aux = childLink->getAngularVelocity();
        math::Vector3 angularVelocity(aux[0], aux[1], aux[2]);
        ignerr << "childvelocity " << angularVelocity << std::endl;
        // result +=
        // globalAxis.Dot(convertVec(childLink->getAngularVelocity()));
        result += globalAxis.Dot(angularVelocity);
      }
      if (this->links.find(jointInfo->parentLinkId) != this->links.end())
      {
        btRigidBody *parentLink =
          this->links.at(jointInfo->parentLinkId)->link;
        btVector3 aux = parentLink->getAngularVelocity();
        math::Vector3 angularVelocity(aux[0], aux[1], aux[2]);
        ignerr << "parentvelocity " << angularVelocity << std::endl;
        // result -=
        // globalAxis.Dot(convertVec(parentLink->getAngularVelocity()));
        result -= globalAxis.Dot(angularVelocity);
      }
    }
    ignerr << "velocity " << result << std::endl;
    double damping_force = damping * result;
    btVector3 hingeAxisLocalA =
    hinge->getFrameOffsetA().getBasis().getColumn(2);
    btVector3 hingeAxisLocalB =
    hinge->getFrameOffsetB().getBasis().getColumn(2);

    btVector3 hingeAxisWorldA =
    hinge->getRigidBodyA().getWorldTransform().getBasis() *
    hingeAxisLocalA;
    btVector3 hingeAxisWorldB =
    hinge->getRigidBodyB().getWorldTransform().getBasis() *
    hingeAxisLocalB;

    btVector3 hingeTorqueA = damping_force * hingeAxisWorldA;
    btVector3 hingeTorqueB = damping_force * hingeAxisWorldB;

    // hinge->getRigidBodyA().applyTorque(hingeTorqueA);
    // hinge->getRigidBodyB().applyTorque(-hingeTorqueB);
  }
}

}
}
}
