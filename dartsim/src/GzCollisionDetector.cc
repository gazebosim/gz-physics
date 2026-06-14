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
#include <unordered_map>
#include <utility>

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/bullet/BulletCollisionGroup.hpp>
#include <dart/collision/ode/OdeCollisionGroup.hpp>
#include <gz/common/Console.hh>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

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
bool GzCollisionDetector::BatchRaycast(
    CollisionGroup */*_group*/,
    const std::vector<GzRay> &/*_rays*/,
    std::vector<GzRayResult> &/*_output*/) const
{
  static bool warned = false;
  if (!warned)
  {
    warned = true;
    gzwarn << "BatchRaycast: collision detector does not support batch "
           << "raycasting. All ray results will be NaN." << std::endl;
  }
  return false;
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

class GzOdeCollisionGroup : public OdeCollisionGroup
{
  friend class GzOdeCollisionDetector;
public:
  /// Constructor
  using OdeCollisionGroup::OdeCollisionGroup;

  using OdeCollisionGroup::getOdeSpaceId;
};

std::unique_ptr<CollisionGroup> GzOdeCollisionDetector::createCollisionGroup()
{
  return std::make_unique<GzOdeCollisionGroup>(shared_from_this());
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

void NearCallbackODE(void *_data, dGeomID _o1, dGeomID _o2)
{
  // Check space
  if (dGeomIsSpace(_o1) || dGeomIsSpace(_o2))
  {
    dSpaceCollide2(_o1, _o2, _data, &NearCallbackODE);
    return;
  }

  // Identify the ray
  dGeomID ray = nullptr;
  dGeomID other = nullptr;

  if (dGeomGetClass(_o1) == dRayClass)
  {
    ray = _o1;
    other = _o2;
  }
  if (dGeomGetClass(_o2) == dRayClass)
  {
    ray = _o2;
    other = _o1;
  }

  if(ray == nullptr)
  {
    // should not happen, but to be safe...
    return;
  }

  dContactGeom contact;

  RaycastResult* result = static_cast<RaycastResult*>(_data);

  auto setResult = [&](dart::collision::RayHit &rayHit)
  {
      auto geomData = dGeomGetData(other);
      rayHit.mNormal = Eigen::Vector3d(contact.normal);
      rayHit.mPoint = Eigen::Vector3d(contact.pos);
      rayHit.mCollisionObject =
        static_cast<dart::collision::CollisionObject*>(geomData);
  };

  // param 3 makes sure that we only generate one collision per call
  if(dCollide(ray, other, 1, &contact, sizeof(dContactGeom)) > 0)
  {
    if(result->mRayHits.empty())
    {
      setResult(result->mRayHits.emplace_back());
    }
    else
    {
      setResult(result->mRayHits.front());
    }
  }
}

static void doSingleRaycastODE(const Eigen::Vector3d& origin,
      const Eigen::Vector3d& target,
      RaycastResult* result,
      const dGeomID &rayId,
      const dSpaceID &spaceId)
{
  const Eigen::Vector3d dirNonNormalized(target - origin);
  const double length = dirNonNormalized.norm();
  result->clear();
  if(length < 1e-7)
  {
    return;
  }

  const Eigen::Vector3d dir(dirNonNormalized / length);

  dGeomRaySet(rayId, origin.x(), origin.y(), origin.z(),
              dir.x(), dir.y(), dir.z());
  dGeomRaySetLength(rayId, length);

  dSpaceCollide2(rayId,
                  reinterpret_cast<dGeomID>(spaceId),
                  result, &NearCallbackODE);

  if(!result->mRayHits.empty())
  {
    // compute fraction, we need to do it here, as we need
    // length and orgin
    RayHit &rayHit(result->mRayHits.front());
    rayHit.mFraction = (rayHit.mPoint - origin).norm() / length;
  }
}

bool GzOdeCollisionDetector::raycast(
      CollisionGroup* group,
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const RaycastOption& option,
      RaycastResult* result)
{
  if(!result)
  {
    return false;
  }

  if(option.mEnableAllHits)
  {
      gzwarn << "raycast multihit support is not"
             << " implemented for ODE" << std::endl;
      return false;
  }

  auto odeGroup = static_cast<GzOdeCollisionGroup *>(group);
  const dSpaceID spaceId = odeGroup->getOdeSpaceId();

  const dGeomID rayId = dCreateRay(spaceId, 1.0);
  dGeomRaySetClosestHit(rayId, 1);

  doSingleRaycastODE(from, to, result, rayId, spaceId);

  dGeomDestroy(rayId);

  // near callback updated our ray hit result now (or not)
  return !result->mRayHits.empty();
}

bool GzOdeCollisionDetector::BatchRaycast(
      CollisionGroup *_group,
      const std::vector<GzRay> &_rays,
      std::vector<GzRayResult> &_results) const
{
  auto odeGroup = static_cast<GzOdeCollisionGroup *>(_group);
  const dSpaceID spaceId = odeGroup->getOdeSpaceId();

  const dGeomID rayId = dCreateRay(spaceId, 1.0);
  dGeomRaySetClosestHit(rayId, 1);

  _results.reserve(_rays.size());
  RaycastResult result;
  for(const GzRay &ray : _rays)
  {
    doSingleRaycastODE(ray.origin, ray.target, &result, rayId, spaceId);

    // near callback updated our ray hit result now (or not)
    if(result.hasHit())
    {
      RayHit &rayHit(result.mRayHits.front());
      _results.emplace_back(GzRayResult{rayHit.mPoint, rayHit.mFraction,
                            rayHit.mNormal});
    }
    else
    {
      _results.emplace_back(GzRayResult{
          Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
          std::numeric_limits<double>::infinity(),
          Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())});
    }
  }

  dGeomDestroy(rayId);

  return true;
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
bool GzBulletCollisionDetector::BatchRaycast(
    CollisionGroup *_group,
    const std::vector<GzRay> &_rays,
    std::vector<GzRayResult> &_output) const
{
  auto *gzGroup = dynamic_cast<GzBulletCollisionGroup *>(_group);
  if (!gzGroup)
    return false;

  const btCollisionWorld *btWorld = gzGroup->getCollisionWorld();
  if (!btWorld)
    return false;

  _output.clear();
  _output.reserve(_rays.size());

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

    GzRayResult &result = _output.emplace_back();
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
  }

  return true;
}
