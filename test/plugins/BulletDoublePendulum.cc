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
#include <unordered_map>

#include <ignition/common/Console.hh>
#include <ignition/physics/RegisterMore.hh>

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

      //void SetState(const SetState::State &x)
      //{
      //  const BulletState *state = x.Query<BulletState>();
      //  if (!state)
      //  {
      //    ignerr << "[mock::bullet::BulletDoublePendulum::"
      //           << "SetState] The state provided does not contain a "
      //           << "BulletState, which this plugins needs in order to go to a "
      //           << "specified state!\n";
      //    return;
      //  }

      //  for (const auto &entry : state->jointStates)
      //  {
      //    btHingeAccumulatedAngleConstraint *hinge = entry.first;
      //    const btScalar angle = entry.second;
      //    hinge->setAccumulatedHingeAngle(angle);
      //  }

      //  for (const auto &entry : state->linkStates)
      //  {
      //    btRigidBody *body = entry.first;
      //    BulletState::LinkState linkState = entry.second;
      //    body->setCenterOfMassTransform(mock::bullet::convert(linkState.pose.pose));
      //    body->setLinearVelocity(mock::bullet::convert(linkState.twist.linearVelocity));
      //    body->setAngularVelocity(mock::bullet::convert(linkState.twist.angularVelocity));
      //    body->clearForces();
      //    body->applyCentralForce(mock::bullet::convert(linkState.wrench.force.vec));
      //    body->applyTorque(mock::bullet::convert(linkState.wrench.torque.vec));
      //  }
      //}

      void WriteState(ignition::physics::ForwardStep::State &x)
      {
        BulletState &state = x.Get<BulletState>();
        state.jointStates.clear();
        state.linkStates.clear();

        state.jointStates[this->joint1] = this->joint1->getAccumulatedHingeAngle();
        state.jointStates[this->joint2] = this->joint2->getAccumulatedHingeAngle();

        BulletState::LinkState linkState1;
        BulletState::LinkState linkState2;
        linkState1.pose.pose = mock::bullet::convert(this->link1->getCenterOfMassTransform());
        linkState2.pose.pose = mock::bullet::convert(this->link2->getCenterOfMassTransform());
        linkState1.twist.linearVelocity = mock::bullet::convert(this->link1->getLinearVelocity());
        linkState2.twist.linearVelocity = mock::bullet::convert(this->link2->getLinearVelocity());
        linkState1.twist.angularVelocity = mock::bullet::convert(this->link1->getAngularVelocity());
        linkState2.twist.angularVelocity = mock::bullet::convert(this->link2->getAngularVelocity());
        linkState1.wrench.force.vec = mock::bullet::convert(this->link1->getTotalForce());
        linkState2.wrench.force.vec = mock::bullet::convert(this->link2->getTotalForce());
        linkState1.wrench.torque.vec = mock::bullet::convert(this->link1->getTotalTorque());
        linkState2.wrench.torque.vec = mock::bullet::convert(this->link2->getTotalTorque());
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

      void SetInputs(const ignition::physics::GeneralizedParameters *_efforts)
      {
        if (_efforts != nullptr)
        {
          this->forces[_efforts->dofs[0]] = _efforts->forces[0];
          this->forces[_efforts->dofs[1]] = _efforts->forces[1];
        }
      }

      void SetTimeStep(const ignition::physics::TimeStep *_timeStep)
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

    class BulletDoublePendulum
        : public virtual mock::MockDoublePendulum,
          public ignition::physics::Implements3d<MockDoublePendulumList>
    {
      using Identity = ignition::physics::Identity;

      public: BulletDoublePendulum()
        : dataPtr(new PrivateBulletDoublePendulum)
      {
        // Do nothing
      }

      public: ~BulletDoublePendulum()
      {
        // Do nothing
      }

      public: Identity InitiateEngine(std::size_t /*_engineID*/) override
      {
        return this->GenerateIdentity(0);
      }

      public: const std::string &GetEngineName(
          std::size_t /*_engineID*/) const override
      {
        static const std::string name("BulletDoublePendulum engine");
        return name;
      }

      public: std::size_t GetEngineIndex(
          std::size_t /*_engineID*/) const override
      {
        return 0;
      }

      public: std::size_t GetWorldCount(
          std::size_t /*_engineID*/) const override
      {
        return 1;
      }

      public: Identity GetWorld(
          std::size_t /*_engineID*/,
          std::size_t /*_worldIndex*/) const override
      {
        return this->GenerateIdentity(1);
      }

      public: Identity GetWorld(
          std::size_t /*_engineID*/,
          const std::string &/*_worldName*/) const override
      {
        return this->GenerateIdentity(1);
      }

      public: const std::string &GetWorldName(
          std::size_t /*_worldID*/) const override
      {
        static const std::string name("BulletDoublePendulum world");
        return name;
      }

      public: std::size_t GetWorldIndex(
          std::size_t /*_worldID*/) const override
      {
        return 0;
      }

      public: Identity GetEngineOfWorld(
          std::size_t /*_worldID*/) const override
      {
        return this->GenerateIdentity(0);
      }

      public: void WorldForwardStep(
          const std::size_t /*_worldId*/,
          ignition::physics::ForwardStep::Output &_h,
          ignition::physics::ForwardStep::State &_x,
          const ignition::physics::ForwardStep::Input &_u) override
      {
        this->dataPtr->SetInputs(_u.Query<ignition::physics::GeneralizedParameters>());
        this->dataPtr->SetTimeStep(_u.Query<ignition::physics::TimeStep>());

        this->dataPtr->Simulate();

        this->dataPtr->WriteState(_x);

        _h.ResetQueries();
        this->WriteRequiredData(_h);
        this->Write(_h.Get<ignition::physics::JointPositions>());
      }

      //public: void SetStateTo(const SetState::State &x)
      //{
      //  this->dataPtr->SetState(x);
      //}

      public: void Write(ignition::physics::JointPositions &_positions) const override
      {
        _positions.dofs = {0, 1};
        _positions.positions.clear();
        _positions.positions.resize(2u);

        _positions.positions[0] = this->dataPtr->joint1->getAccumulatedHingeAngle();
        _positions.positions[1] = this->dataPtr->joint2->getAccumulatedHingeAngle();
      }

      public: void Write(ignition::physics::WorldPoses &_poses) const override
      {
        _poses.entries.clear();
        _poses.entries.reserve(this->dataPtr->mapToBodies.size());

        for(const auto &entry : this->dataPtr->mapToBodies)
        {
          const std::size_t id = entry.first;
          const btRigidBody *body = entry.second;

          ignition::physics::WorldPose wp;
          btTransform bt = body->getCenterOfMassTransform();
          wp.pose = mock::bullet::convert(bt);
          wp.body = id;

          _poses.entries.push_back(wp);
        }
      }

      private: std::unique_ptr<PrivateBulletDoublePendulum> dataPtr;
    };

    using FeaturePolicy3d = ignition::physics::FeaturePolicy3d;
    IGN_PHYSICS_ADD_PLUGIN(
        BulletDoublePendulum,
        FeaturePolicy3d,
        MockDoublePendulumList)
  }
}
