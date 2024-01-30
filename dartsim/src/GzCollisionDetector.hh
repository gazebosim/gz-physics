/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_DARTSIM_SRC_GZCOLLISIONDETECTOR_HH_
#define GZ_PHYSICS_DARTSIM_SRC_GZCOLLISIONDETECTOR_HH_

#include <cstdio>
#include <limits>
#include <memory>

#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

namespace dart {
namespace collision {

class GzCollisionDetector
{
  /// \brief Set the maximum number of contacts between a pair of collision
  /// objects
  /// \param[in] _maxContacts Maximum number of contacts between a pair of
  /// collision objects.
  public: virtual void SetCollisionPairMaxContacts(std::size_t _maxContacts);

  /// \brief Get the maximum number of contacts between a pair of collision
  /// objects
  /// \return Maximum number of contacts between a pair of collision objects.
  public: virtual std::size_t GetCollisionPairMaxContacts() const;

  /// Constructor
  protected: GzCollisionDetector();

  /// \brief Limit max number of contacts between a pair of collision objects.
  /// The function modifies the contacts vector inside the CollisionResult
  /// object to cap the number of contacts for each collision pair based on the
  /// maxCollisionPairContacts value
  protected: virtual void LimitCollisionPairMaxContacts(
      CollisionResult *_result);

  /// \brief Maximum number of contacts between a pair of collision objects.
  protected: std::size_t maxCollisionPairContacts =
      std::numeric_limits<std::size_t>::max();
};

class GzOdeCollisionDetector :
    public dart::collision::OdeCollisionDetector,
    public dart::collision::GzCollisionDetector
{
  // Documentation inherited
  public: bool collide(
      CollisionGroup* group,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  // Documentation inherited
  public: bool collide(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  /// \brief Create the GzOdeCollisionDetector
  public: static std::shared_ptr<GzOdeCollisionDetector> create();

  /// Constructor
  protected: GzOdeCollisionDetector();

  private: static Registrar<GzOdeCollisionDetector> mRegistrar;
};

class GzBulletCollisionDetector :
    public dart::collision::BulletCollisionDetector,
    public dart::collision::GzCollisionDetector
{
  // Documentation inherited
  public: bool collide(
      CollisionGroup* group,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  // Documentation inherited
  public: bool collide(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  /// \brief Create the GzBulletCollisionDetector
  public: static std::shared_ptr<GzBulletCollisionDetector> create();

  /// Constructor
  protected: GzBulletCollisionDetector();

  private: static Registrar<GzBulletCollisionDetector> mRegistrar;
};

}
}

#endif
