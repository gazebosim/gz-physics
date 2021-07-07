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

#include <memory>
#include <unordered_map>

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/common/Console.hh>
#include <ignition/physics/Register.hh>

#include "../MockDoublePendulum.hh"

namespace mock
{
  namespace dart
  {
    struct DartState
    {
      using StateMap =
          std::unordered_map<::dart::dynamics::SkeletonPtr,
                             ::dart::dynamics::Skeleton::Configuration>;

      StateMap states;
    };


    class PrivateDARTDoublePendulum
    {
      public: ::dart::simulation::WorldPtr world;

      public: ::dart::dynamics::SkeletonPtr robot;

      public: ::dart::dynamics::Joint* joint1 = nullptr;
      public: ::dart::dynamics::Joint* joint2 = nullptr;

      public: Eigen::VectorXd forces = Eigen::VectorXd::Zero(2);

      public: using BodyMap =
          std::unordered_map<std::size_t, ::dart::dynamics::WeakBodyNodePtr>;

      public: BodyMap mapToBodies;

      public: std::size_t lastId;

      PrivateDARTDoublePendulum()
        : world(new ::dart::simulation::World),
          lastId(0)
      {
        ::dart::utils::DartLoader loader;
        this->robot = loader.parseSkeleton(IGNITION_PHYSICS_RESOURCE_DIR "/rrbot.xml");
        this->world->addSkeleton(this->robot);

        this->joint1 = this->robot->getJoint("joint1");
        this->joint2 = this->robot->getJoint("joint2");

        for (std::size_t i=0; i < robot->getNumJoints(); ++i)
        {
// From upstream 6.10.0 or OSRF 6.10.0~osrf19~2021-06-10
#if DART_VERSION_AT_LEAST(6, 10, 0)
          this->robot->getJoint(i)->setLimitEnforcement(false);
#else
          this->robot->getJoint(i)->setPositionLimitEnforced(false);
#endif
          this->robot->getJoint(i)->setDampingCoefficient(0, 0);
        }

        this->SetBodyMap();
        this->world->setGravity(Eigen::Vector3d(0, 0, -9.81));
      }

      void SetBodyMap()
      {
        for (std::size_t i=0; i < this->world->getNumSkeletons(); ++i)
        {
          ::dart::dynamics::SkeletonPtr skel = this->world->getSkeleton(i);
          for (std::size_t j=0; j < skel->getNumBodyNodes(); ++j)
          {
            ::dart::dynamics::BodyNode * const bn = skel->getBodyNode(j);
            this->mapToBodies[lastId++] = bn;
          }
        }
      }

      //void SetState(const SetState::State &x)
      //{
      //  const DartState *state = x.Query<DartState>();
      //  if (!state)
      //  {
      //    ignerr << "[ignition::physics::dart::DARTDoublePendulum::"
      //           << "SetState] The state provided does not contain a "
      //           << "DartState, which this plugins needs in order to go to a "
      //           << "specified state!\n";
      //    return;
      //  }

      //  std::unordered_set<::dart::dynamics::SkeletonPtr> allSkels;
      //  for (const auto &entry : state->states)
      //  {
      //    const ::dart::dynamics::SkeletonPtr &skel = entry.first;
      //    const ::dart::dynamics::SkeletonPtr &worldSkel =
      //        world->getSkeleton(skel->getName());
      //    if (skel != worldSkel)
      //    {
      //      world->removeSkeleton(worldSkel);
      //      world->addSkeleton(skel);
      //    }

      //    skel->setConfiguration(entry.second);

      //    allSkels.insert(skel);
      //  }

      //  std::unordered_set<::dart::dynamics::SkeletonPtr> removeSkels;
      //  for (std::size_t i=0; i < world->getNumSkeletons(); ++i)
      //  {
      //    const ::dart::dynamics::SkeletonPtr &worldSkel =
      //        this->world->getSkeleton(i);

      //    if (allSkels.count(worldSkel) == 0)
      //      removeSkels.insert(worldSkel);
      //  }

      //  for (const auto &skel : removeSkels)
      //    this->world->removeSkeleton(skel);
      //}

      void WriteState(ignition::physics::ForwardStep::State &x)
      {
        DartState &state = x.Get<DartState>();
        state.states.clear();

        for (std::size_t i=0; i < world->getNumSkeletons(); ++i)
        {
          const ::dart::dynamics::SkeletonPtr &skel =
              this->world->getSkeleton(i);

          state.states[skel] = skel->getConfiguration();
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
          this->world->setTimeStep(_timeStep->dt);
        }
      }

      void Simulate()
      {
        this->robot->setForces(this->forces);

        this->world->step();
      }
    };

    class DARTDoublePendulum
        : public virtual mock::MockDoublePendulum,
          public ignition::physics::Implements3d<MockDoublePendulumList>
    {
      using Identity = ignition::physics::Identity;

      public: DARTDoublePendulum()
        : dataPtr(new PrivateDARTDoublePendulum)
      {
        // Do nothing
      }

      public: ~DARTDoublePendulum()
      {
        // Do nothing
      }

      public: Identity InitiateEngine(std::size_t /*_engineID*/) override
      {
        return this->GenerateIdentity(0);
      }

      public: const std::string &GetEngineName(
          const Identity &/*_engineID*/) const override
      {
        static const std::string name("DARTDoublePendulum engine");
        return name;
      }

      public: std::size_t GetEngineIndex(
          const Identity &/*_engineID*/) const override
      {
        return 0;
      }

      public: std::size_t GetWorldCount(
          const Identity &/*_engineID*/) const override
      {
        return 1;
      }

      public: Identity GetWorld(
          const Identity &/*_engineID*/,
          std::size_t /*_worldIndex*/) const override
      {
        return this->GenerateIdentity(1);
      }

      public: Identity GetWorld(
          const Identity &/*_engineID*/,
          const std::string &/*_worldName*/) const override
      {
        return this->GenerateIdentity(1);
      }

      public: const std::string &GetWorldName(
          const Identity &/*_worldID*/) const override
      {
        static const std::string name("DARTDoublePendulum world");
        return name;
      }

      public: std::size_t GetWorldIndex(
          const Identity &/*_worldID*/) const override
      {
        return 0;
      }

      public: Identity GetEngineOfWorld(
          const Identity &/*_worldID*/) const override
      {
        return this->GenerateIdentity(0);
      }

      public: void WorldForwardStep(
          const Identity &/*_worldId*/,
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

      //public: void SetStateTo(const SetState::State &x) override;

      public: void Write(ignition::physics::JointPositions &_positions) const override
      {
        auto configuration = this->dataPtr->robot->getConfiguration(
                              ::dart::dynamics::Skeleton::CONFIG_POSITIONS);
        _positions.dofs = configuration.mIndices;
        _positions.positions.clear();
        _positions.positions.resize(this->dataPtr->robot->getNumDofs());

        for (const auto &dof : _positions.dofs)
        {
          if (dof < this->dataPtr->robot->getNumDofs())
          {
            _positions.positions[dof] = configuration.mPositions[dof];
          }
        }
      }

      public: void Write(ignition::physics::WorldPoses &_poses) const override
      {
        _poses.entries.clear();
        _poses.entries.reserve(this->dataPtr->mapToBodies.size());

        std::vector<std::size_t> cleanup;
        for(const auto &entry : this->dataPtr->mapToBodies)
        {
          const std::size_t id = entry.first;
          const ::dart::dynamics::BodyNodePtr bn = entry.second.lock();
          if(!bn)
          {
            // Remove bodies that no longer exist
            cleanup.push_back(id);
            continue;
          }

          ignition::physics::WorldPose wp;
          wp.pose = ignition::math::eigen3::convert(bn->getWorldTransform());
          wp.pose.Pos() = ignition::math::eigen3::convert(bn->getCOM());
          wp.body = id;

          _poses.entries.push_back(wp);
        }

        for(const std::size_t id : cleanup)
          this->dataPtr->mapToBodies.erase(id);
      }

      private: std::unique_ptr<PrivateDARTDoublePendulum> dataPtr;
    };

    using FeaturePolicy3d = ignition::physics::FeaturePolicy3d;
    IGN_PHYSICS_ADD_PLUGIN(
        DARTDoublePendulum,
        FeaturePolicy3d,
        MockDoublePendulumList)
    //void DARTDoublePendulum::SetStateTo(const SetState::State &x)
    //{
    //  this->dataPtr->SetState(x);
    //}
  }
}
