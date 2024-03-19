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

#ifndef GZ_PHYSICS_WORLD_HH_
#define GZ_PHYSICS_WORLD_HH_

#include <string>

#include <gz/physics/FeatureList.hh>
#include <gz/physics/FrameSemantics.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    class GZ_PHYSICS_VISIBLE CollisionDetector : public virtual Feature
    {
      /// \brief The World API for setting the collision detector.
      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        /// \brief Set the name of the collision detector to use.
        /// \param[in] _collisionDetector Name of collision detector.
        public: void SetCollisionDetector(
            const std::string &_collisionDetector);

        /// \brief Get the name of the collision detector in use.
        /// \return Name of collision detector.
        public: const std::string &GetCollisionDetector() const;
      };

      /// \private The implementation API for the collision detector.
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        /// \brief Implementation API for setting the collision detector.
        /// \param[in] _id Identity of the world.
        /// \param[in] _collisionDetector Name of collision detector.
        public: virtual void SetWorldCollisionDetector(
            const Identity &_id, const std::string &_collisionDetector) = 0;

        /// \brief Implementation API for getting the collision detector.
        /// \param[in] _id Identity of the world.
        /// \return Name of collision detector.
        public: virtual const std::string &GetWorldCollisionDetector(
            const Identity &_id) const = 0;
      };
    };

    /////////////////////////////////////////////////
    using GravityRequiredFeatures = FeatureList<FrameSemantics>;

    /////////////////////////////////////////////////
    /// \brief Get and set the World's gravity vector in a specified frame.
    class GZ_PHYSICS_VISIBLE Gravity
        : public virtual
          FeatureWithRequirements<GravityRequiredFeatures>
    {
      /// \brief The World API for getting and setting the gravity vector.
      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        public: using LinearVectorType =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using RelativeForceType =
            typename FromPolicy<PolicyT>::template Use<RelativeForce>;

        /// \brief Set the World gravity vector.
        /// \param[in] _gravity The desired gravity as a Relative Gravity
        /// (a quantity that contains information about the coordinates
        /// in which it is expressed).
        public: void SetGravity(const RelativeForceType &_gravity);

        /// \brief Set the World gravity vector. Optionally, you may specify
        /// the frame whose coordinates are used to express the gravity vector.
        /// The World frame is used as a default if no frame is specified.
        /// \param[in] _gravity Gravity vector.
        /// \param[in] _inCoordinatesOf Frame whose coordinates are used
        /// to express _gravity.
        public: void SetGravity(
            const LinearVectorType &_gravity,
            const FrameID &_forceInCoordinatesOf = FrameID::World());

        /// \brief Get the World gravity vector. Optionally, you may specify
        /// the frame whose coordinates are used to express the gravity vector.
        /// The World frame is used as a default if no frame is specified.
        /// \param[in] _inCoordinatesOf Frame whose coordinates are used
        /// to express _gravity.
        /// \return Gravity vector in corrdinates of _inCoordinatesOf.
        public: LinearVectorType GetGravity(
            const FrameID &_forceInCoordinatesOf = FrameID::World()) const;
      };

      /// \private The implementation API for the gravity.
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using LinearVectorType =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        /// \brief Implementation API for setting the gravity vector, which is
        /// expressed in the World frame..
        /// \param[in] _id Identity of the world.
        /// \param[in] _gravity Value of gravity.
        public: virtual void SetWorldGravity(
            const Identity &_id, const LinearVectorType &_gravity) = 0;

        /// \brief Implementation API for getting the gravity expressed in the
        /// world frame.
        /// \param[in] _id Identity of the world.
        /// \return Value of gravity.
        public: virtual LinearVectorType GetWorldGravity(
            const Identity &_id) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class GZ_PHYSICS_VISIBLE CollisionPairMaxContacts:
        public virtual Feature
    {
      /// \brief The World API for getting and setting the maximum
      /// number of contacts between two entities.
      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        /// \brief Set the maximum number of contacts allowed between two
        /// entities.
        /// \param[in] _maxContacts Maximum number of contacts.
        public: void SetCollisionPairMaxContacts(std::size_t _maxContacts);

        /// \brief Get the maximum number of contacts allowed between two
        /// entities.
        /// \return Maximum number of contacts.
        public: std::size_t GetCollisionPairMaxContacts() const;
      };

      /// \private The implementation API for getting and setting max contacts.
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        /// \brief Implementation API for setting the maximum number of
        /// contacts between two entities.
        /// \param[in] _id Identity of the world.
        /// \param[in] _maxContacts Maximum number of contacts.
        public: virtual void SetWorldCollisionPairMaxContacts(
            const Identity &_id, std::size_t _maxContacts) = 0;

        /// \brief Implementation API for getting the maximum number of
        /// contacts between two entities.
        /// \param[in] _id Identity of the world.
        /// \return Maximum number of contacts.
        public: virtual std::size_t GetWorldCollisionPairMaxContacts(
            const Identity &_id) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class GZ_PHYSICS_VISIBLE Solver : public virtual Feature
    {
      /// \brief The World API for setting the solver.
      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        /// \brief Set the name of the solver to use.
        /// \param[in] _solver Name of solver.
        public: void SetSolver(const std::string &_solver);

        /// \brief Get the name of the solver in use.
        /// \return Name of solver.
        public: const std::string &GetSolver() const;

        /// \brief Set the number of solver iterations per step.
        /// \param[in] _Iterations Number of solver iterations per step
        public: void SetSolverIterations(std::size_t _iterations);

        /// \brief Get the number of solver iterations per step.
        /// \return Number of solver iterations per step.
        public: std::size_t GetSolverIterations() const;
      };

      /// \private The implementation API for the solver.
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        /// \brief Implementation API for setting the solver.
        /// \param[in] _id Identity of the world.
        /// \param[in] _solver Name of solver.
        public: virtual void SetWorldSolver(
            const Identity &_id, const std::string &_solver) = 0;

        /// \brief Implementation API for getting the solver.
        /// \param[in] _id Identity of the world.
        /// \return Name of solver.
        public: virtual const std::string &GetWorldSolver(
            const Identity &_id) const = 0;

        /// \brief Implementation API for setting the number of solver
        /// iterations.
        /// \param[in] _id Identity of the world.
        /// \param[in] _solver Number of solver iterations.
        public: virtual void SetWorldSolverIterations(
            const Identity &_id, std::size_t _iterations) = 0;

        /// \brief Implementation API for getting the number of solver
        /// iterations.
        /// \param[in] _id Identity of the world.
        /// \return Number of solver iterations.
        public: virtual std::size_t GetWorldSolverIterations(
            const Identity &_id) const = 0;
      };
    };
  }
}

#include <gz/physics/detail/World.hh>

#endif
