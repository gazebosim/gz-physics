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

#include <unordered_map>
#include <ode/ode.h>

#include "ignition/common/Console.hh"

#include "BuggyDemoController.hh"
#include "MathConversions.hh"
#include <cmath>
#include <functional>

namespace ignition
{
  namespace physics
  {
    namespace ode
    {
      struct OdeState
      {
        using BodyPoseMap = std::unordered_map<dBodyID, ignition::math::Vector3d>;
        using BodyQuatMap = std::unordered_map<dBodyID, ignition::math::Quaterniond>;
        BodyPoseMap poseStates;
        BodyQuatMap quatStates;
      };

      class PrivateBuggyDemoController
      {
        /// \brief Chassis length
        public: static constexpr dReal chassisLength = 0.7;
        /// \brief Chassis width
        public: static constexpr dReal chassisWidth = 0.5;
        /// \brief Chassis height
        public: static constexpr dReal chassisHeight = 0.2;
        /// \brief Wheel radius
        public: static constexpr dReal wheelRadius = 0.18;
        /// \brief Starting height of chassis
        public: static constexpr dReal startHeight = 0.5;
        /// \brief Chassis mass
        public: static constexpr dReal chassisMass = 1;
        /// \brief Wheel mass
        public: static constexpr dReal wheelMass = 0.2;
        /// \brief World ID
        public: static dWorldID world;
        /// \brief Space ID
        public: static dSpaceID space;
        /// \brief Body ID, 1 chassis, 3 wheels, in total 4 bodies
        public: static dBodyID body[4];
        /// \brief Joint ID, joint[0] is the front wheel
        public: static dJointID joint[3];
        /// \brief Contact group ID
        public: static dJointGroupID contactgroup;
        /// \brief GeomID for the ground, which the buggy collides with
        public: static dGeomID ground;
        /// \brief Car space
        public: static dSpaceID carSpace;
        /// \brief GeomID for the chassis
        public: static dGeomID box[1];
        /// \brief GeomID for the 3 wheels, sphere[0] is the front wheel
        public: static dGeomID sphere[3];
        /// \brief GeomID for the ground box to represent planar ground
        public: static dGeomID groundBox;
        /// \brief Speed of the buggy
        public: static double speed;
        /// \brief Steering angle
        public: static double steer;

        PrivateBuggyDemoController()
        {
        }

        ~PrivateBuggyDemoController()
        {
        }

        static void initialize();
        static void nearCallback(void *, dGeomID o1, dGeomID o2);
        static void SetState(const SetState::State &x);
        static void WriteState(ForwardStep::State &x);
        static void Simulate();
        static void WriteOutputPoses(CompositeData &h);
      };

      using self = PrivateBuggyDemoController;
      dWorldID self::world;
      dSpaceID self::space;
      dBodyID self::body[4];
      dJointID self::joint[3];
      dJointGroupID self::contactgroup;
      dGeomID self::ground;
      dSpaceID self::carSpace;
      dGeomID self::box[1];
      dGeomID self::sphere[3];
      dGeomID self::groundBox;
      double self::speed = 0.0;
      double self::steer = 0.0;

      void self::initialize()
      {
        dMass m;
        // create world
        dInitODE2(0);
        self::world = dWorldCreate();
        space = dHashSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        dWorldSetGravity(self::world, 0, 0, -0.5);
        ground = dCreatePlane(space, 0, 0, 1, 0);

        // chasis body
        body[0] = dBodyCreate(world);
        dBodySetPosition(body[0], 0, 0, startHeight);
        dMassSetBox(&m, 1, chassisLength, chassisWidth, chassisHeight);
        dMassAdjust(&m, chassisMass);
        dBodySetMass(body[0], &m);
        box[0] = dCreateBox(0, chassisLength, chassisWidth, chassisHeight);
        dGeomSetBody(box[0], body[0]);

        // wheel bodies
        for (std::size_t i=1; i<=3; ++i)
        {
          body[i] = dBodyCreate(world);
          dQuaternion q;
          dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);
          dBodySetQuaternion(body[i], q);
          dMassSetSphere(&m, 1, wheelRadius);
          dMassAdjust(&m, wheelMass);
          dBodySetMass(body[i], &m);
          sphere[i-1] = dCreateSphere(0, wheelRadius);
          dGeomSetBody(sphere[i-1], body[i]);
        }

        dBodySetPosition(body[1], 0.5 * chassisLength, 0,
            startHeight - 0.5 * chassisHeight);
        dBodySetPosition(body[2], -0.5 * chassisLength, 0.5 * chassisWidth,
            startHeight - 0.5 * chassisHeight);
        dBodySetPosition(body[3], -0.5 * chassisLength, -0.5 * chassisWidth,
            startHeight - 0.5 * chassisHeight);

        // front and back wheel hinges
        for (std::size_t i=0; i<3; ++i)
        {
          joint[i] = dJointCreateHinge2(world, 0);
          dJointAttach(joint[i], body[0], body[i+1]);
          const dReal *a = dBodyGetPosition(body[i+1]);
          dJointSetHinge2Anchor(joint[i], a[0], a[1], a[2]);
          const dVector3 yunit = {0, 1, 0};
          const dVector3 zunit = {0, 0, 1};
          dJointSetHinge2Axes(joint[i], zunit, yunit);
        }

        // set joint suspension
        for (std::size_t i=0; i<3; ++i)
        {
          dJointSetHinge2Param(joint[i], dParamSuspensionERP, 0.4);
          dJointSetHinge2Param(joint[i], dParamSuspensionCFM, 0.8);
        }

        // lock back wheels along the steering axis
        for (std::size_t i=1; i<3; i++)
        {
          // set stops to make sure wheels always stay in alignment
          dJointSetHinge2Param(joint[i], dParamLoStop, 0);
          dJointSetHinge2Param(joint[i], dParamHiStop, 0);
        }

        // create car space and add it to the top level space
        carSpace = dSimpleSpaceCreate(space);
        dSpaceSetCleanup(carSpace, 0);
        dSpaceAdd(carSpace, box[0]);
        dSpaceAdd(carSpace, sphere[0]);
        dSpaceAdd(carSpace, sphere[1]);
        dSpaceAdd(carSpace, sphere[2]);

        // environment
        groundBox = dCreateBox(space, 2, 1.5, 1);
        dMatrix3 R;
        dRFromAxisAndAngle(R, 0, 1, 0, -0.15);
        dGeomSetPosition(groundBox, 2, 0, -0.34);
        dGeomSetRotation(groundBox, R);
      }

      void self::nearCallback(void *, dGeomID o1, dGeomID o2)
      {
        int i, n;

        // only collide things with the ground
        int g1 = (o1 == ground || o1 == groundBox);
        int g2 = (o2 == ground || o2 == groundBox);
        if (!(g1 ^ g2))
          return;

        const int N = 10;
        dContact contact[N];
        n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
        if (n > 0) {
          for (i = 0; i < n; ++i) {
            contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
              dContactSoftERP | dContactSoftCFM | dContactApprox1;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.slip1 = 0.1;
            contact[i].surface.slip2 = 0.1;
            contact[i].surface.soft_erp = 0.5;
            contact[i].surface.soft_cfm = 0.3;
            dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
            dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
                dGeomGetBody(contact[i].geom.g2));
          }
        }
      }

      void self::Simulate()
      {
        // motor
        dJointSetHinge2Param(joint[0], dParamVel2, -self::speed);
        dJointSetHinge2Param(joint[0], dParamFMax2, 0.1);

        // steering
        dReal v = steer - dJointGetHinge2Angle1(joint[0]);
        if (v > 0.1) {
          v = 0.1;
        }
        if (v < -0.1) {
          v = -0.1;
        }
        v *= 10.0;

        dJointSetHinge2Param(joint[0], dParamVel, v);
        dJointSetHinge2Param(joint[0], dParamFMax, 0.2);
        dJointSetHinge2Param(joint[0], dParamLoStop, -0.75);
        dJointSetHinge2Param(joint[0], dParamHiStop, 0.75);
        dJointSetHinge2Param(joint[0], dParamFudgeFactor, 0.1);

        dSpaceCollide(space, 0, &nearCallback);
        dWorldStep(world, 0.05);

        // remove all contact joints
        dJointGroupEmpty(contactgroup);
      }

      void self::SetState(const SetState::State &x)
      {
        const OdeState *state = x.Query<OdeState>();
        if (!state)
        {
          ignerr << "[ignition::physics::ode::BuggyDemoController::SetState]"
            << "The state provided does not contain an OdeState, which"
            << "this plugins need in order to go to a specified state!"
            << std::endl;
          return;
        }

        for (const auto &entry : state->poseStates)
        {
          const dReal *pos = convert(entry.second);
          dBodySetPosition(entry.first, *pos, *(pos+1), *(pos+2));
        }

        for (const auto &entry : state->quatStates)
        {
          const dReal *quat = convert(entry.second);
          dBodySetQuaternion(entry.first, quat);
        }
      }

      void self::WriteState(ForwardStep::State &x)
      {
        OdeState &state = x.Get<OdeState>();
        state.poseStates.clear();
        state.quatStates.clear();
        for (int i=0; i<4; ++i)
        {
          state.poseStates[body[i]] = convertVec(dBodyGetPosition(body[i]));
          state.quatStates[body[i]] = convertQuat(dBodyGetQuaternion(body[i]));
        }
      }

      void self::WriteOutputPoses(CompositeData &h)
      {
        ChassisPose &chassisPose = h.Get<ChassisPose>();
        chassisPose.pose.Pos() = convertVec(dBodyGetPosition(self::body[0]));
        chassisPose.pose.Rot() = convertQuat(dBodyGetQuaternion(self::body[0]));
        chassisPose.body = 0;

        FrontWheelPose &wheelPose = h.Get<FrontWheelPose>();
        wheelPose.pose.Pos() = convertVec(dBodyGetPosition(self::body[1]));
        wheelPose.pose.Rot() = convertQuat(dBodyGetQuaternion(self::body[1]));
        wheelPose.body = 1;
      }

      BuggyDemoController::~BuggyDemoController()
      {
        dGeomDestroy(self::box[0]);
        dGeomDestroy(self::sphere[0]);
        dGeomDestroy(self::sphere[1]);
        dGeomDestroy(self::sphere[2]);
        dJointGroupDestroy(self::contactgroup);
        dSpaceDestroy(self::space);
        dWorldDestroy(self::world);
        dCloseODE();
      }

      BuggyDemoController::BuggyDemoController()
      {
        self::initialize();
      }

      void BuggyDemoController::Step(
          Output &h, ForwardStep::State &x, const Input &u)
      {
        const ApplySpeedSteer *speedSteer = u.Query<ApplySpeedSteer>();
        if (speedSteer)
        {
          self::speed = speedSteer->speed;
          self::steer = speedSteer->steer;
        }

        self::Simulate();
        self::WriteState(x);

        h.ResetQueries();
        this->WriteRequiredData(h);
        self::WriteOutputPoses(h);
      }

      void BuggyDemoController::SetStateTo(const SetState::State &x)
      {
        self::SetState(x);
      }

      void BuggyDemoController::Write(WorldPoses &poses) const
      {
        poses.entries.reserve(4);
        for (std::size_t i=0; i<4; ++i)
        {
          WorldPose wp;
          wp.pose.Pos() = convertVec(dBodyGetPosition(self::body[i]));
          wp.pose.Rot() = convertQuat(dBodyGetQuaternion(self::body[i]));
          wp.body = i;
          poses.entries.push_back(wp);
        }
      }
    }
  }
}
