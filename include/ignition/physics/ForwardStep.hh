/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_FORWARDSTEP_HH_
#define IGNITION_PHYSICS_FORWARDSTEP_HH_

#include <ignition/math.hh>
#include <ignition/common/PluginMacros.hh>

#include "ignition/physics/SpecifyData.hh"
#include "ignition/physics/CompositeDataMacros.hh"

namespace ignition
{
  namespace physics
  {
    // ---------------- Output Data Structures -----------------
    // In the long-term, these data structures should be defined in some kind of
    // meta-file, and our build system should generate these definitions during
    // compilation. These are being defined here in this header until we know
    // more about how the ECS will work.

    struct WorldPose
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::WorldPose)

      ignition::math::Pose3d pose;

      std::size_t body;
    };

    struct TargetPose : WorldPose
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::TargetPose)
    };

    struct EndEffectorPose : WorldPose
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::EndEffectorPose)
    };

    struct WorldPoses
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::WorldPoses)

      std::vector<WorldPose> entries;
      std::string annotation;
    };

    struct Point
    {
      ignition::math::Vector3d point;

      std::size_t relativeTo;
      std::size_t inCoordinatesOf;
    };

    struct FreeVector
    {
      ignition::math::Vector3d vec;

      std::size_t inCoordinatesOf;
    };

    struct WorldTwist
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::WorldTwist)

      ignition::math::Vector3d linearVelocity;
      ignition::math::Vector3d angularVelocity;

      std::size_t body;
    };

    struct JointPositions
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::JointPositions)

      std::vector<std::size_t> dofs;
      std::vector<double> positions;
      std::string annotation;
    };

    struct Contacts
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::Contacts)

      std::vector<Point> entries;
      std::string annotation;
    };

    // ---------------- Input Data Structures -----------------
    // Same note as for Output Data Structures. Eventually, these should be
    // defined in some kind of meta files.

    struct TimeStep
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::TimeStepping)

      double dt;
    };

    struct ForceTorque
    {
      std::size_t body;
      Point location;

      FreeVector force;
      FreeVector torque;

      std::string annotation;
    };

    struct GeneralizedParameters
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::GeneralizedParameters)

      std::vector<std::size_t> dofs;
      std::vector<double> forces;
      std::string annotation;
    };

    struct PIDValues
    {
      double P;
      double I;
      double D;
    };

    struct ApplyExternalForceTorques
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::ApplyExternalForceTorques)

      std::vector<ForceTorque> entries;
      std::string annotation;
    };

    struct ApplyGeneralizedForces
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::ApplyJointGeneralizedForces)

      std::vector<GeneralizedParameters> forces;
      std::string annotation;
    };

    struct VelocityControlCommands
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::VelocityControlCommands)

      std::vector<GeneralizedParameters> commands;
      std::string annotation;
    };

    struct ServoControlCommands
    {
      IGN_PHYSICS_DATA_LABEL(ignition::physics::ServoControlCommands)

      std::vector<GeneralizedParameters> commands;
      std::vector<PIDValues> gains;

      std::string annotation;
    };


    // ---------------- ForwardStep Interface -----------------
    class ForwardStep
    {
      public: IGN_COMMON_SPECIALIZE_INTERFACE(ignition::physics::ForwardStep)

      public: using Input = ExpectData<
              ApplyExternalForceTorques,
              ApplyGeneralizedForces,
              VelocityControlCommands,
              ServoControlCommands>;

      public: using Output = SpecifyData<
          RequireData<WorldPoses>,
          ExpectData<Contacts, JointPositions> >;

      public: using State = CompositeData;

      public: virtual void Step(Output &h, State &x, const Input &u) = 0;

      public: virtual ~ForwardStep() = default;
    };


    // ---------------- SetState Interface -----------------
    class SetState
    {
      public: IGN_COMMON_SPECIALIZE_INTERFACE(ignition::physics::SetState)

      public: using State = CompositeData;

      public: virtual void SetStateTo(const State &x) = 0;

      public: virtual ~SetState() = default;
    };
  }
}

#endif
