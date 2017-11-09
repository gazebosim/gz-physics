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

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "ignition/common/Console.hh"

#include "OperationalSpaceController.hh"
#include "MathConversions.hh"

namespace ignition
{
  namespace physics
  {
    namespace dart
    {
      struct DartState
      {
        IGN_PHYSICS_DATA_LABEL(ignition::physics::dart::DartState)

        using StateMap =
            std::unordered_map<::dart::dynamics::SkeletonPtr,
                               ::dart::dynamics::Skeleton::Configuration>;

        StateMap states;
      };


      class PrivateOperationalSpaceController
      {
        public: ::dart::simulation::WorldPtr world;

        public: ::dart::dynamics::SkeletonPtr robot;

        public: ::dart::dynamics::BodyNode* endEffector;

        public: ::dart::dynamics::SimpleFramePtr target;

        public: Eigen::Vector3d offset;
        public: Eigen::Matrix3d Kp;
        public: Eigen::MatrixXd Kd;
        public: Eigen::VectorXd forces;

        public: using BodyMap =
            std::unordered_map<std::size_t, ::dart::dynamics::WeakBodyNodePtr>;

        public: BodyMap mapToBodies;

        public: std::size_t id_endEffector;

        public: std::size_t lastId;

        PrivateOperationalSpaceController()
          : world(new ::dart::simulation::World),
            lastId(0)
        {
          ::dart::utils::DartLoader loader;
          this->robot = loader.parseSkeleton(
                "dart://sample/urdf/KR5/KR5 sixx R650.urdf");
          this->world->addSkeleton(this->robot);

          this->robot->getJoint(0)->setTransformFromParentBodyNode(
                Eigen::Isometry3d::Identity());

          this->endEffector = this->robot->getBodyNode(
                this->robot->getNumBodyNodes()-1);

          std::size_t dofs = this->endEffector->getNumDependentGenCoords();
          Kp.setZero();
          for (std::size_t i=0; i < 3; ++i)
            Kp(i, i) = 50.0;

          Kd.setZero(dofs, dofs);
          for (std::size_t i=0; i < dofs; ++i)
            Kd(i, i) = 5.0;

          for (std::size_t i=0; i < robot->getNumJoints(); ++i)
          {
            this->robot->getJoint(i)->setPositionLimitEnforced(false);
            this->robot->getJoint(i)->setDampingCoefficient(0, 0.5);
          }

          this->offset = Eigen::Vector3d(0.0, 0.0, 0.0);

          Eigen::Isometry3d tf = endEffector->getWorldTransform();
          tf.pretranslate(offset);
          this->target = std::make_shared<::dart::dynamics::SimpleFrame>(
                ::dart::dynamics::Frame::World(), "target", tf);

          this->offset = this->endEffector->getWorldTransform()
              .rotation().transpose() * offset;

          this->SetBodyMap();
        }

        void SetBodyMap()
        {
          for (std::size_t i=0; i < this->world->getNumSkeletons(); ++i)
          {
            ::dart::dynamics::SkeletonPtr skel = this->world->getSkeleton(i);
            for (std::size_t j=0; j < skel->getNumBodyNodes(); ++j)
            {
              ::dart::dynamics::BodyNode * const bn = skel->getBodyNode(j);
              if (bn == this->endEffector)
                this->id_endEffector = lastId;

              this->mapToBodies[lastId++] = bn;
            }
          }
        }

        void UpdateTarget(const WorldPose *_target)
        {
          if (!_target)
            return;

          this->target->setTransform(convert(_target->pose));
        }

        void SetState(const SetState::State &x)
        {
          const DartState *state = x.Query<DartState>();
          if (!state)
          {
            ignerr << "[ignition::physics::dart::OperationalSpaceController::"
                   << "SetState] The state provided does not contain a "
                   << "DartState, which this plugins needs in order to go to a "
                   << "specified state!\n";
            return;
          }

          std::unordered_set<::dart::dynamics::SkeletonPtr> allSkels;
          for (const auto &entry : state->states)
          {
            const ::dart::dynamics::SkeletonPtr &skel = entry.first;
            const ::dart::dynamics::SkeletonPtr &worldSkel =
                world->getSkeleton(skel->getName());
            if (skel != worldSkel)
            {
              world->removeSkeleton(worldSkel);
              world->addSkeleton(skel);
            }

            skel->setConfiguration(entry.second);

            allSkels.insert(skel);
          }

          std::unordered_set<::dart::dynamics::SkeletonPtr> removeSkels;
          for (std::size_t i=0; i < world->getNumSkeletons(); ++i)
          {
            const ::dart::dynamics::SkeletonPtr &worldSkel =
                this->world->getSkeleton(i);

            if (allSkels.count(worldSkel) == 0)
              removeSkels.insert(worldSkel);
          }

          for (const auto &skel : removeSkels)
            this->world->removeSkeleton(skel);
        }

        void WriteState(ForwardStep::State &x)
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

        void Simulate()
        {
          Eigen::MatrixXd M = this->robot->getMassMatrix();

          ::dart::math::LinearJacobian J =
              this->endEffector->getLinearJacobian(this->offset);
          Eigen::MatrixXd pinv_J = J.transpose()*(
                J*J.transpose() + 0.0025*Eigen::Matrix3d::Identity()).inverse();

          ::dart::math::LinearJacobian dJ =
              this->endEffector->getLinearJacobianDeriv(this->offset);
          Eigen::MatrixXd pinv_dJ = dJ.transpose()*(
                dJ*dJ.transpose() + 0.0025*Eigen::Matrix3d::Identity())
              .inverse();

          Eigen::Vector3d e = this->target->getWorldTransform().translation()
              - this->endEffector->getWorldTransform()*this->offset;

          Eigen::Vector3d de = - endEffector->getLinearVelocity(this->offset);

          Eigen::VectorXd Cg = robot->getCoriolisAndGravityForces();

          this->forces = M*(pinv_J*Kp*de + pinv_dJ*Kp*e) + Cg + Kd*pinv_J*Kp*e;

          this->robot->setForces(this->forces);

          this->world->step();
        }

        void WriteEndEffectorPose(CompositeData &h)
        {
          EndEffectorPose &pose = h.Get<EndEffectorPose>();
          pose.pose = convert(this->endEffector->getWorldTransform());
          pose.body = this->id_endEffector;
        }
      };

      OperationalSpaceController::~OperationalSpaceController()
      {
        // Do nothing
      }

      OperationalSpaceController::OperationalSpaceController()
        : dataPtr(new PrivateOperationalSpaceController)
      {
        // Do nothing
      }

      void OperationalSpaceController::Step(
          ForwardStep::Output &h, ForwardStep::State &x,
          const ForwardStep::Input &u)
      {
        this->dataPtr->UpdateTarget(u.Query<TargetPose>());

        this->dataPtr->Simulate();

        this->dataPtr->WriteState(x);

        h.ResetQueries();
        this->WriteRequiredData(h);
        this->dataPtr->WriteEndEffectorPose(h);
      }

      void OperationalSpaceController::SetStateTo(const SetState::State &x)
      {
        this->dataPtr->SetState(x);
      }

      void OperationalSpaceController::Write(WorldPoses &poses) const
      {
        poses.entries.reserve(this->dataPtr->mapToBodies.size());

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

          WorldPose wp;
          wp.pose = convert(bn->getWorldTransform());
          wp.body = id;

          poses.entries.push_back(wp);
        }

        for(const std::size_t id : cleanup)
          this->dataPtr->mapToBodies.erase(id);
      }
    }
  }
}
