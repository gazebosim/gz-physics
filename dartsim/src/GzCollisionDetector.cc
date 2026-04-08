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

#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <utility>

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/bullet/BulletCollisionGroup.hpp>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <gz/common/Console.hh>

#include "GzCollisionDetector.hh"

using namespace dart;
using namespace collision;

/////////////////////////////////////////////////
GzCollisionDetector::GzCollisionDetector()
{
}

/////////////////////////////////////////////////
void GzCollisionDetector::SetCollisionPairMaxContacts(
    std::size_t _maxContacts)
{
  this->maxCollisionPairContacts = _maxContacts;
}

/////////////////////////////////////////////////
std::size_t GzCollisionDetector::GetCollisionPairMaxContacts() const
{
  return this->maxCollisionPairContacts;
}

/////////////////////////////////////////////////
void GzCollisionDetector::LimitCollisionPairMaxContacts(
    CollisionResult *_result)
{
  if (this->maxCollisionPairContacts ==
    std::numeric_limits<std::size_t>::max())
    return;

  auto allContacts = _result->getContacts();
  _result->clear();


  if (this->maxCollisionPairContacts == 0u)
    return;

  // A map of collision pairs and their contact info
  // Contact info is stored in std::pair. The elements are:
  // <contact count, index of last contact point (in _result)>
  std::unordered_map<dart::collision::CollisionObject *,
      std::unordered_map<dart::collision::CollisionObject *,
      std::pair<std::size_t, std::size_t>>>
      contactMap;

  for (auto &contact : allContacts)
  {
    auto &[count, lastContactIdx] =
        contactMap[contact.collisionObject1][contact.collisionObject2];
    count++;
    auto &[otherCount, otherLastContactIdx] =
      contactMap[contact.collisionObject2][contact.collisionObject1];

    std::size_t total =  count + otherCount;
    if (total <= this->maxCollisionPairContacts)
    {
      if (total == this->maxCollisionPairContacts)
      {
        lastContactIdx = _result->getNumContacts();
        otherLastContactIdx = lastContactIdx;
      }
      _result->addContact(contact);
    }
    else
    {
      // If too many contacts were generated, replace the last contact point
      // of the collision pair with one that has a larger penetration depth
      auto &c = _result->getContact(lastContactIdx);
      if (contact.penetrationDepth > c.penetrationDepth)
      {
        c = contact;
      }
    }
  }
}

/////////////////////////////////////////////////
std::optional<std::vector<GzRayResult>> GzCollisionDetector::BatchRaycast(
    CollisionGroup */*_group*/,
    const std::vector<GzRay> &/*_rays*/) const
{
  static bool warned = false;
  if (!warned)
  {
    warned = true;
    gzwarn << "BatchRaycast: collision detector does not support batch "
           << "raycasting. All ray results will be NaN." << std::endl;
  }
  return std::nullopt;
}

/////////////////////////////////////////////////
GzOdeCollisionDetector::GzOdeCollisionDetector()
  : OdeCollisionDetector(), GzCollisionDetector()
{
}

/////////////////////////////////////////////////
GzOdeCollisionDetector::Registrar<GzOdeCollisionDetector>
    GzOdeCollisionDetector::mRegistrar{
        GzOdeCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<GzOdeCollisionDetector> {
          return GzOdeCollisionDetector::create();
        }};

/////////////////////////////////////////////////
std::shared_ptr<GzOdeCollisionDetector> GzOdeCollisionDetector::create()
{
  // GzOdeCollisionDetector constructor calls the OdeCollisionDetector
  // constructor, that calls the non-thread safe dInitODE2(0).
  // To mitigate this problem, we use a static mutex to ensure that
  // each GzOdeCollisionDetector constructor is called not at the same time.
  // See https://github.com/gazebosim/gz-sim/issues/18 for more info.
  static std::mutex odeInitMutex;
  std::unique_lock<std::mutex> lock(odeInitMutex);
  return std::shared_ptr<GzOdeCollisionDetector>(new GzOdeCollisionDetector());
}

/////////////////////////////////////////////////
bool GzOdeCollisionDetector::collide(
    CollisionGroup *_group,
    const CollisionOption &_option,
    CollisionResult *_result)
{
  bool ret = OdeCollisionDetector::collide(_group, _option, _result);
  this->LimitCollisionPairMaxContacts(_result);
  return ret;
}

/////////////////////////////////////////////////
bool GzOdeCollisionDetector::collide(
    CollisionGroup *_group1,
    CollisionGroup *_group2,
    const CollisionOption &_option,
    CollisionResult *_result)
{
  bool ret = OdeCollisionDetector::collide(_group1, _group2, _option, _result);
  this->LimitCollisionPairMaxContacts(_result);
  return ret;
}

/// \brief Exposes BulletCollisionGroup::getBulletCollisionWorld() which
/// is protected in the base class.
class GzBulletCollisionGroup : public dart::collision::BulletCollisionGroup
{
  public: explicit GzBulletCollisionGroup(
      const dart::collision::CollisionDetectorPtr &_detector);

  /// \brief Return the underlying btCollisionWorld
  public: const btCollisionWorld *getCollisionWorld() const;
};

/////////////////////////////////////////////////
GzBulletCollisionGroup::GzBulletCollisionGroup(
    const dart::collision::CollisionDetectorPtr &_detector)
  : dart::collision::BulletCollisionGroup(_detector)
{
}

/////////////////////////////////////////////////
const btCollisionWorld *GzBulletCollisionGroup::getCollisionWorld() const
{
  // getBulletCollisionWorld() is protected in BulletCollisionGroup.
  return this->getBulletCollisionWorld();
}

/////////////////////////////////////////////////
GzBulletCollisionDetector::GzBulletCollisionDetector()
  : BulletCollisionDetector(), GzCollisionDetector()
{
}

/////////////////////////////////////////////////
std::unique_ptr<dart::collision::CollisionGroup>
GzBulletCollisionDetector::createCollisionGroup()
{
  return std::make_unique<GzBulletCollisionGroup>(this->shared_from_this());
}

/////////////////////////////////////////////////
GzBulletCollisionDetector::Registrar<GzBulletCollisionDetector>
    GzBulletCollisionDetector::mRegistrar{
        GzBulletCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<GzBulletCollisionDetector> {
          return GzBulletCollisionDetector::create();
        }};

/////////////////////////////////////////////////
std::shared_ptr<GzBulletCollisionDetector> GzBulletCollisionDetector::create()
{
  return std::shared_ptr<GzBulletCollisionDetector>(
      new GzBulletCollisionDetector());
}

/////////////////////////////////////////////////
bool GzBulletCollisionDetector::collide(
    CollisionGroup *_group,
    const CollisionOption &_option,
    CollisionResult *_result)
{
  bool ret = BulletCollisionDetector::collide(_group, _option, _result);
  this->LimitCollisionPairMaxContacts(_result);
  return ret;
}

/////////////////////////////////////////////////
bool GzBulletCollisionDetector::collide(
    CollisionGroup *_group1,
    CollisionGroup *_group2,
    const CollisionOption &_option,
    CollisionResult *_result)
{
  bool ret = BulletCollisionDetector::collide(
      _group1, _group2, _option, _result);
  this->LimitCollisionPairMaxContacts(_result);
  return ret;
}

/////////////////////////////////////////////////
std::optional<std::vector<GzRayResult>> GzBulletCollisionDetector::BatchRaycast(
    CollisionGroup *_group,
    const std::vector<GzRay> &_rays) const
{
  std::vector<GzRayResult> results;
  results.reserve(_rays.size());

  auto *gzGroup = dynamic_cast<GzBulletCollisionGroup *>(_group);
  if (!gzGroup)
    return std::nullopt;

  const btCollisionWorld *btWorld = gzGroup->getCollisionWorld();
  if (!btWorld)
    return std::nullopt;

  for (const auto &ray : _rays)
  {
    const btVector3 btFrom(
      static_cast<btScalar>(ray.origin.x()),
      static_cast<btScalar>(ray.origin.y()),
      static_cast<btScalar>(ray.origin.z()));
    const btVector3 btTo(
      static_cast<btScalar>(ray.target.x()),
      static_cast<btScalar>(ray.target.y()),
      static_cast<btScalar>(ray.target.z()));

    btCollisionWorld::ClosestRayResultCallback rayCallback(btFrom, btTo);
    btWorld->rayTest(btFrom, btTo, rayCallback);

    GzRayResult result;
    if (rayCallback.hasHit())
    {
      const btVector3 &hp = rayCallback.m_hitPointWorld;
      const btVector3 &hn = rayCallback.m_hitNormalWorld;
      result.point << hp.x(), hp.y(), hp.z();
      result.normal << hn.x(), hn.y(), hn.z();
      result.fraction = static_cast<double>(rayCallback.m_closestHitFraction);
    }
    else
    {
      // No object in range: fraction is +INF per REP-117.
      // point and normal are undefined (NaN) when there is no hit.
      constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
      result.point = Eigen::Vector3d::Constant(kNaN);
      result.fraction = std::numeric_limits<double>::infinity();
      result.normal = Eigen::Vector3d::Constant(kNaN);
    }
    results.push_back(std::move(result));
  }

  return results;
}
