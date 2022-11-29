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

#ifndef GZ_PHYSICS_FORWARDSTEP_HH_
#define GZ_PHYSICS_FORWARDSTEP_HH_

#include <string>
#include <vector>

#include <gz/math.hh>

#include <gz/physics/SpecifyData.hh>
#include <gz/physics/FeatureList.hh>

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
      ignition::math::Pose3d pose;

      std::size_t body;
    };

    struct WorldPoses
    {
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

    struct JointPositions
    {
      std::vector<std::size_t> dofs;
      std::vector<double> positions;
      std::string annotation;
    };

    struct Contacts
    {
      std::vector<Point> entries;
      std::string annotation;
    };

    // ---------------- Input Data Structures -----------------
    // Same note as for Output Data Structures. Eventually, these should be
    // defined in some kind of meta files.

    struct TimeStep
    {
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
      std::vector<ForceTorque> entries;
      std::string annotation;
    };

    struct ApplyGeneralizedForces
    {
      std::vector<GeneralizedParameters> forces;
      std::string annotation;
    };

    struct VelocityControlCommands
    {
      std::vector<GeneralizedParameters> commands;
      std::string annotation;
    };

    struct ServoControlCommands
    {
      std::vector<GeneralizedParameters> commands;
      std::vector<PIDValues> gains;

      std::string annotation;
    };

    /////////////////////////////////////////////////
    /// \brief ForwardStep is a feature that allows a simulation of a world to
    /// take one step forward in time.
    class ForwardStep : public virtual Feature
    {
      public: using Input = ExpectData<
              ApplyExternalForceTorques,
              ApplyGeneralizedForces,
              VelocityControlCommands,
              ServoControlCommands>;

      public: using Output = SpecifyData<
          RequireData<WorldPoses>,
          ExpectData<Contacts, JointPositions> >;

      public: using State = CompositeData;

      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        public: void Step(Output &_h, State &_x, const Input &_u)
        {
          this->template Interface<ForwardStep>()->
              WorldForwardStep(this->identity, _h, _x, _u);
        }
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual void WorldForwardStep(
            const Identity &_worldID,
            Output &_h,
            State &_x,
            const Input &_u) = 0;
      };
    };

    // ---------------- SetState Interface -----------------
    // class SetState
    // {
    //   public: using State = CompositeData;

    //   public: virtual void SetStateTo(const State &x) = 0;

    //   public: virtual ~SetState() = default;
    // };
  }
}

#endif
