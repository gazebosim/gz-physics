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

#include <limits>
#include <memory>

#include <dart/collision/ode/OdeCollisionDetector.hpp>

namespace dart {
namespace collision {

class GzOdeCollisionDetector : public dart::collision::OdeCollisionDetector
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

  /// \brief Set the maximum number of contacts between a pair of collision
  /// objects
  /// \param[in] _maxContacts Maximum number of contacts between a pair of
  /// collision objects.
  public: void SetCollisionPairMaxTotalContacts(std::size_t _maxContacts);

  /// \brief Get the maximum number of contacts between a pair of collision
  /// objects
  /// \return Maximum number of contacts between a pair of collision objects.
  public: std::size_t GetCollisionPairMaxTotalContacts() const;


  /// \brief Create the GzOdeCollisionDetector
  public: static std::shared_ptr<GzOdeCollisionDetector> create();

  /// Constructor
  protected: GzOdeCollisionDetector();

  /// \brief Limit max number of contacts between a pair of collision objects.
  /// The function modifies the contacts vector inside the CollisionResult
  /// object to cap the number of contacts for each collision pair based on the
  /// maxCollisionPairContacts value
  private: void LimitCollisionPairMaxTotalContacts(CollisionResult *_result);

  /// \brief Maximum number of contacts between a pair of collision objects.
  private: std::size_t maxCollisionPairContacts =
      std::numeric_limits<std::size_t>::max();

  private: static Registrar<GzOdeCollisionDetector> mRegistrar;
};

}
}
