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

#include <ignition/common/Console.hh>
#include <ignition/physics/Register.hh>

#include "../MockDoublePendulum.hh"
#include "BulletMathConversions.hh"

namespace mock
{
    namespace bullet
    {
      struct BulletState
      {
        // TODO: consider using bullet's built-in serialization for state
        // btSerializer and btBulletFile
        using JointStateMap =
            std::unordered_map<btHingeAccumulatedAngleConstraint*,
                               btScalar>;
        struct LinkState
        {
          ignition::physics::WorldPose pose;
          ignition::physics::WorldTwist twist;
          ignition::physics::ForceTorque wrench;
        };
        using LinkStateMap =
            std::unordered_map<btRigidBody*,
                               LinkState>;

        JointStateMap jointStates;
        LinkStateMap linkStates;
      };


      class PrivateBulletDoublePendulum
      {
        // based on HelloWorld.cpp example
        // TODO: clean up allocated memory in a destructor
        // see example for how to teardown cleanly
        public: btDefaultCollisionConfiguration *collisionConfig = nullptr;
        public: btCollisionDispatcher *dispatcher = nullptr;
        public: btBroadphaseInterface *overlappingPairCache = nullptr;
        public: btSequentialImpulseConstraintSolver *solver = nullptr;
        public: btDiscreteDynamicsWorld *world = nullptr;

        public: btCollisionShape *shape = nullptr;
        public: btRigidBody *link1 = nullptr;
        public: btRigidBody *link2 = nullptr;

        // TODO: consider using btMultibody for comparison
        public: btHingeAccumulatedAngleConstraint *joint1 = nullptr;
        public: btHingeAccumulatedAngleConstraint *joint2 = nullptr;

        public: double dt = 1e-3;
        public: double forces[2] = {0, 0};

        public: using BodyMap =
            std::unordered_map<std::size_t, btRigidBody*>;

        public: BodyMap mapToBodies;

        public: std::size_t lastId;

        PrivateBulletDoublePendulum()
          : collisionConfig(new btDefaultCollisionConfiguration),
            dispatcher(new btCollisionDispatcher(collisionConfig)),
            overlappingPairCache(new btDbvtBroadphase()),
            solver(new btSequentialImpulseConstraintSolver),
            world(new btDiscreteDynamicsWorld(dispatcher,
                                              overlappingPairCache,
                                              solver,
                                              collisionConfig)),
            lastId(0)
        {
          this->shape = new btBoxShape(0.5*btVector3(0.1, 0.1, 1.0));
          const double mass = 1.0;
          const double ixx = 1.0;
          const double iyy = 1.0;
          const double izz = 1.0;

          btTransform startTransform1;
          btTransform startTransform2;
          startTransform1.setIdentity();
          startTransform2.setIdentity();
          startTransform1.setOrigin(btVector3(0, 0.1, 2.4));
          startTransform2.setOrigin(btVector3(0, 0.2, 3.3));
          btDefaultMotionState *motionState1 = new btDefaultMotionState(startTransform1);
          btDefaultMotionState *motionState2 = new btDefaultMotionState(startTransform2);
          this->link1 = new btRigidBody(
              btRigidBody::btRigidBodyConstructionInfo(
                  mass, motionState1, shape, btVector3(ixx, iyy, izz)));
          this->link2 = new btRigidBody(
              btRigidBody::btRigidBodyConstructionInfo(
                  mass, motionState2, shape, btVector3(ixx, iyy, izz)));
          this->world->addRigidBody(this->link1);
          this->world->addRigidBody(this->link2);

          // make sure objects don't fall asleep
          this->link1->setActivationState(DISABLE_DEACTIVATION);
          this->link2->setActivationState(DISABLE_DEACTIVATION);

          // joint1 connected to world
          this->joint1 = new btHingeAccumulatedAngleConstraint(
                              *this->link1,
                              btVector3(0, 0, -0.45),
                              btVector3(0, 1, 0));

          // joint2 connecting link1 and link2
          this->joint2 = new btHingeAccumulatedAngleConstraint(
                              *this->link2,
                              *this->link1,
                              btVector3(0, 0, -0.45),
                              btVector3(0, 0.1, 0.45),
                              btVector3(0, 1, 0),
                              btVector3(0, 1, 0));

          this->world->addConstraint(this->joint1);
          this->world->addConstraint(this->joint2);

          this->SetBodyMap();
          this->world->setGravity(btVector3(0, 0, -9.81));
        }

        void SetBodyMap()
        {
          this->mapToBodies[lastId++] = this->link1;
          this->mapToBodies[lastId++] = this->link2;
        }

        void SetState(const SetState::State &x)
        {
          const BulletState *state = x.Query<BulletState>();
          if (!state)
          {
            ignerr << "[ignition::physics::dart::BulletDoublePendulum::"
                   << "SetState] The state provided does not contain a "
                   << "BulletState, which this plugins needs in order to go to a "
                   << "specified state!\n";
            return;
          }

          for (const auto &entry : state->jointStates)
          {
            btHingeAccumulatedAngleConstraint *hinge = entry.first;
            const btScalar angle = entry.second;
            hinge->setAccumulatedHingeAngle(angle);
          }

          for (const auto &entry : state->linkStates)
          {
            btRigidBody *body = entry.first;
            BulletState::LinkState linkState = entry.second;
            body->setCenterOfMassTransform(convert(linkState.pose.pose));
            body->setLinearVelocity(convert(linkState.twist.linearVelocity));
            body->setAngularVelocity(convert(linkState.twist.angularVelocity));
            body->clearForces();
            body->applyCentralForce(convert(linkState.wrench.force.vec));
            body->applyTorque(convert(linkState.wrench.torque.vec));
          }
        }

        void WriteState(ForwardStep::State &x)
        {
          BulletState &state = x.Get<BulletState>();
          state.jointStates.clear();
          state.linkStates.clear();

          state.jointStates[this->joint1] = this->joint1->getAccumulatedHingeAngle();
          state.jointStates[this->joint2] = this->joint2->getAccumulatedHingeAngle();

          BulletState::LinkState linkState1;
          BulletState::LinkState linkState2;
          linkState1.pose.pose = convert(this->link1->getCenterOfMassTransform());
          linkState2.pose.pose = convert(this->link2->getCenterOfMassTransform());
          linkState1.twist.linearVelocity = convert(this->link1->getLinearVelocity());
          linkState2.twist.linearVelocity = convert(this->link2->getLinearVelocity());
          linkState1.twist.angularVelocity = convert(this->link1->getAngularVelocity());
          linkState2.twist.angularVelocity = convert(this->link2->getAngularVelocity());
          linkState1.wrench.force.vec = convert(this->link1->getTotalForce());
          linkState2.wrench.force.vec = convert(this->link2->getTotalForce());
          linkState1.wrench.torque.vec = convert(this->link1->getTotalTorque());
          linkState2.wrench.torque.vec = convert(this->link2->getTotalTorque());
          state.linkStates[this->link1] = linkState1;
          state.linkStates[this->link2] = linkState2;
        }

        void ApplyJointTorque(btHingeConstraint *_joint, const double _torque)
        {
          // copied from gazebo::physics::BulletHingeJoint::SetForceImpl
          if (_joint)
          {
            // z-axis of constraint frame
            btVector3 hingeAxisLocalA =
              _joint->getFrameOffsetA().getBasis().getColumn(2);
            btVector3 hingeAxisLocalB =
              _joint->getFrameOffsetB().getBasis().getColumn(2);

            btVector3 hingeAxisWorldA =
              _joint->getRigidBodyA().getWorldTransform().getBasis() *
              hingeAxisLocalA;
            btVector3 hingeAxisWorldB =
              _joint->getRigidBodyB().getWorldTransform().getBasis() *
              hingeAxisLocalB;

            btVector3 hingeTorqueA = _torque * hingeAxisWorldA;
            btVector3 hingeTorqueB = _torque * hingeAxisWorldB;

            _joint->getRigidBodyA().applyTorque(hingeTorqueA);
            _joint->getRigidBodyB().applyTorque(-hingeTorqueB);
          }
        }

        void SetInputs(const GeneralizedParameters *_efforts)
        {
          if (_efforts != nullptr)
          {
            this->forces[_efforts->dofs[0]] = _efforts->forces[0];
            this->forces[_efforts->dofs[1]] = _efforts->forces[1];
          }
        }

        void SetTimeStep(const TimeStep *_timeStep)
        {
          if (_timeStep != nullptr)
          {
            this->dt = _timeStep->dt;
          }
        }

        void Simulate()
        {
          ApplyJointTorque(this->joint1, this->forces[0]);
          ApplyJointTorque(this->joint2, this->forces[1]);

          this->world->stepSimulation(this->dt, 1, this->dt);
        }
      };

      BulletDoublePendulum::~BulletDoublePendulum()
      {
        // Do nothing
      }

      BulletDoublePendulum::BulletDoublePendulum()
        : dataPtr(new PrivateBulletDoublePendulum)
      {
        // Do nothing
      }

      void BulletDoublePendulum::Step(
          Output &h, ForwardStep::State &x, const Input &u)
      {
        this->dataPtr->SetInputs(u.Query<GeneralizedParameters>());
        this->dataPtr->SetTimeStep(u.Query<TimeStep>());

        this->dataPtr->Simulate();

        this->dataPtr->WriteState(x);

        h.ResetQueries();
        this->WriteRequiredData(h);
        this->Write(h.Get<ignition::physics::JointPositions>());
      }

      void BulletDoublePendulum::SetStateTo(const SetState::State &x)
      {
        this->dataPtr->SetState(x);
      }

      void BulletDoublePendulum::Write(JointPositions &_out) const
      {
        _out.dofs = {0, 1};
        _out.positions.clear();
        _out.positions.resize(2u);

        _out.positions[0] = this->dataPtr->joint1->getAccumulatedHingeAngle();
        _out.positions[1] = this->dataPtr->joint2->getAccumulatedHingeAngle();
      }

      void BulletDoublePendulum::Write(WorldPoses &poses) const
      {
        poses.entries.clear();
        poses.entries.reserve(this->dataPtr->mapToBodies.size());

        for(const auto &entry : this->dataPtr->mapToBodies)
        {
          const std::size_t id = entry.first;
          const btRigidBody *body = entry.second;

          WorldPose wp;
          btTransform bt = body->getCenterOfMassTransform();
          wp.pose = convert(bt);
          wp.body = id;

          poses.entries.push_back(wp);
        }
      }
    }
}
