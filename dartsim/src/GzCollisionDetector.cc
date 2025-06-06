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

/////////////////////////////////////////////////
GzBulletCollisionDetector::GzBulletCollisionDetector()
  : BulletCollisionDetector(), GzCollisionDetector()
{
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
