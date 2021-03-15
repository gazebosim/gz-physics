/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_WORLD_HH_
#define IGNITION_PHYSICS_WORLD_HH_

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FrameID.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE PhysicsOptions : public virtual Feature
    {
      /// \brief The World API for setting physics options.
      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        /// \brief Set the name of the collision detector to use.
        /// \param[in] _collisionDetector Name of collision detector.
        public: void SetCollisionDetector(const std::string &_collisionDetector);

        /// \brief Get the name of the collision detector in use.
        /// \return Name of collision detector.
        public: const std::string &GetCollisionDetector() const;

        /// \brief Set the name of the solver to use.
        /// \param[in] _solver Name of solver.
        public: void SetSolver(const std::string &_solver);

        /// \brief Get the name of the solver in use.
        /// \return Name of solver.
        public: const std::string &GetSolver() const;
      };

      /// \private The implementation API for physics options.
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        /// \brief Implementation API for setting the collision detector.
        /// \param[in] _id Identity of the world.
        /// \param[in] _collisionDetector Name of collision detector.
        public: virtual void SetCollisionDetector(
            const Identity &_id, const std::string &_collisionDetector) = 0;

        /// \brief Implementation API for getting the collision detector.
        /// \param[in] _id Identity of the world.
        /// \return Name of collision detector.
        public: virtual const std::string &GetCollisionDetector(
            const Identity &_id) const = 0;

        /// \brief Implementation API for setting the solver.
        /// \param[in] _id Identity of the world.
        /// \param[in] _collisionDetector Name of solver.
        public: virtual void SetSolver(
            const Identity &_id, const std::string &_solver) = 0;

        /// \brief Implementation API for getting the solver.
        /// \param[in] _id Identity of the world.
        /// \return Name of solver.
        public: virtual const std::string &GetSolver(
            const Identity &_id) const = 0;
      };
    };
  }
}

#include <ignition/physics/detail/World.hh>

#endif
