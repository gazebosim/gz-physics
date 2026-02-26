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
#include <vector>

#include <Eigen/Core>

#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/bullet/BulletCollisionGroup.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

namespace dart {
namespace collision {

/// \brief Single ray query: origin and target in world coordinates.
struct GzRay
{
  Eigen::Vector3d from;
  Eigen::Vector3d to;
};

/// \brief Result of a single ray query.
struct GzRayResult
{
  bool hit{false};
  Eigen::Vector3d point{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  double fraction{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d normal{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
};

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

  /// \brief Cast multiple rays against a collision group.
  /// \param[in] _group The collision group to test against.
  /// \param[in] _rays The rays to cast.
  /// \param[out] _results One result per input ray, in the same order.
  /// \return True if the detector supports batch raycasting, false otherwise.
  ///   When false, _results is left empty and the caller should fall back.
  public: virtual bool BatchRaycast(
      CollisionGroup *_group,
      const std::vector<GzRay> &_rays,
      std::vector<GzRayResult> &_results) const;

  /// Destructor
  public: virtual ~GzCollisionDetector() = default;

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

/// \brief Exposes BulletCollisionGroup::getBulletCollisionWorld() which
/// is protected in the base class.
class GzBulletCollisionGroup : public dart::collision::BulletCollisionGroup
{
  public: explicit GzBulletCollisionGroup(
      const dart::collision::CollisionDetectorPtr &_detector);

  /// \brief Return the underlying btCollisionWorld
  public: const btCollisionWorld *getCollisionWorld() const;
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

  // Documentation inherited
  public: std::unique_ptr<CollisionGroup> createCollisionGroup() override;

  // Documentation inherited
  public: bool BatchRaycast(
      CollisionGroup *_group,
      const std::vector<GzRay> &_rays,
      std::vector<GzRayResult> &_results) const override;

  /// \brief Create the GzBulletCollisionDetector
  public: static std::shared_ptr<GzBulletCollisionDetector> create();

  /// Constructor
  protected: GzBulletCollisionDetector();

  private: static Registrar<GzBulletCollisionDetector> mRegistrar;
};

}
}

#endif
