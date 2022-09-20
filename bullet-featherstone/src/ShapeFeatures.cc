/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "ShapeFeatures.hh"

#include <gz/math/eigen3/Conversions.hh>
#include <memory>
#include <string>
#include <utility>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
AlignedBox3d ShapeFeatures::GetShapeAxisAlignedBoundingBox(
    const Identity &_shapeID) const
{
  const auto *collider = this->ReferenceInterface<CollisionInfo>(_shapeID);
  if (collider)
  {
    btTransform t;
    t.setIdentity();
    btVector3 minBox(0, 0, 0);
    btVector3 maxBox(0, 0, 0);
    btVector3 minBox2(0, 0, 0);
    btVector3 maxBox2(0, 0, 0);
    collider->collider->getAabb(t, minBox, maxBox);
    return math::eigen3::convert(math::AxisAlignedBox(
      math::Vector3d(minBox[0], minBox[1], minBox[2]),
      math::Vector3d(maxBox[0], maxBox[1], maxBox[2])));
  }
  return math::eigen3::convert(math::AxisAlignedBox());
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToBoxShape(
      const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<CollisionInfo>(_shapeID);
  if (shapeInfo != nullptr)
  {
    const auto &shape = shapeInfo->collider;
    if (dynamic_cast<btBoxShape*>(shape.get()))
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetBoxShapeSize(
      const Identity &_boxID) const
{
  // _boxID ~= _collisionID
  auto it = this->collisions.find(_boxID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *box = static_cast<btBoxShape*>(it->second->collider.get());
      btVector3 v = box->getHalfExtentsWithMargin();
      return math::eigen3::convert(math::Vector3d(v[0], v[1], v[2]) * 2);
    }
  }
  // return invalid box shape size if no collision found
  return math::eigen3::convert(math::Vector3d(-1.0, -1.0, -1.0));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachBoxShape(
      const Identity &_linkID,
      const std::string &_name,
      const LinearVector3d &_size,
      const Pose3d &_pose)
{
  const auto size = math::eigen3::convert(_size);
  const btVector3 halfExtents = btVector3(size.X(), size.Y(), size.Z()) * 0.5;
  std::unique_ptr<btCollisionShape> shape =
    std::make_unique<btBoxShape>(halfExtents);

  auto identity = this->AddCollision(
    CollisionInfo{
      _name,
      std::move(shape),
      _linkID,
      _pose});

  return identity;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToCapsuleShape(const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<CollisionInfo>(_shapeID);
  if (shapeInfo != nullptr)
  {
    const auto &shape = shapeInfo->collider;
    if (dynamic_cast<btCapsuleShapeZ*>(shape.get()))
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCapsuleShapeRadius(
    const Identity &_capsuleID) const
{
  auto it = this->collisions.find(_capsuleID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *capsule = static_cast<btCapsuleShapeZ*>(
        it->second->collider.get());
      if (capsule)
      {
        return capsule->getRadius();
      }
    }
  }

  return -1;
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCapsuleShapeLength(
    const Identity &_capsuleID) const
{
  auto it = this->collisions.find(_capsuleID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *capsule = static_cast<btCapsuleShapeZ*>(
        it->second->collider.get());
      if (capsule)
      {
        return capsule->getHalfHeight() * 2;
      }
    }
  }

  return -1;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachCapsuleShape(
    const Identity &_linkID,
    const std::string &_name,
    const double _radius,
    const double _length,
    const Pose3d &_pose)
{
  auto shape =
    std::make_unique<btCapsuleShapeZ>(_radius, _length / 2);

  auto identity = this->AddCollision(
    CollisionInfo{
      _name,
      std::move(shape),
      _linkID,
      _pose});

  return identity;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToCylinderShape(const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<CollisionInfo>(_shapeID);
  if (shapeInfo != nullptr)
  {
    const auto &shape = shapeInfo->collider;
    if (dynamic_cast<btCylinderShape*>(shape.get()))
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCylinderShapeRadius(
    const Identity &_cylinderID) const
{
  auto it = this->collisions.find(_cylinderID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *cylinder = static_cast<btCylinderShape*>(
        it->second->collider.get());
      if (cylinder)
      {
        return cylinder->getHalfExtentsWithMargin()[0];
      }
    }
  }

  return -1;
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCylinderShapeHeight(
    const Identity &_cylinderID) const
{
  auto it = this->collisions.find(_cylinderID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *cylinder = static_cast<btCylinderShape*>(
        it->second->collider.get());
      if (cylinder)
      {
        return cylinder->getHalfExtentsWithMargin()[2] * 2;
      }
    }
  }

  return -1;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachCylinderShape(
    const Identity &_linkID,
    const std::string &_name,
    const double _radius,
    const double _height,
    const Pose3d &_pose)
{
  const auto radius = _radius;
  const auto halfLength = _height * 0.5;
  auto shape =
    std::make_unique<btCylinderShapeZ>(btVector3(radius, radius, halfLength));

  auto identity = this->AddCollision(
    CollisionInfo{
      _name,
      std::move(shape),
      _linkID,
      _pose});

  return identity;
}


/////////////////////////////////////////////////
Identity ShapeFeatures::CastToEllipsoidShape(const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<CollisionInfo>(_shapeID);
  if (shapeInfo != nullptr)
  {
    const auto &shape = shapeInfo->collider;
    if (dynamic_cast<btMultiSphereShape*>(shape.get()))
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Vector3d ShapeFeatures::GetEllipsoidShapeRadii(
    const Identity &_ellipsoidID) const
{
  auto it = this->collisions.find(_ellipsoidID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *ellipsoid = static_cast<btMultiSphereShape*>(
        it->second->collider.get());
      if (ellipsoid)
      {
        auto radii = ellipsoid->getLocalScaling();
        return Vector3d(radii[0], radii[1], radii[2]);
      }
    }
  }

  return Vector3d(-1, -1, -1);
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachEllipsoidShape(
    const Identity &_linkID,
    const std::string &_name,
    const Vector3d _radii,
    const Pose3d &_pose)
{
  btVector3 positions[1];
  btScalar radius[1];
  positions[0] = btVector3();
  radius[0] = 1;

  auto btSphere = std::make_unique<btMultiSphereShape>(
    positions, radius, 1);
  btSphere->setLocalScaling(btVector3(_radii[0], _radii[1], _radii[2]));
  auto shape = std::move(btSphere);

  auto identity = this->AddCollision(
    CollisionInfo{
      _name,
      std::move(shape),
      _linkID,
      _pose});
  return identity;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToSphereShape(
    const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<CollisionInfo>(_shapeID);
  if (shapeInfo != nullptr)
  {
    const auto &shape = shapeInfo->collider;
    if (dynamic_cast<btSphereShape*>(shape.get()))
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetSphereShapeRadius(const Identity &_sphereID) const
{
  auto it = this->collisions.find(_sphereID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *sphere = static_cast<btSphereShape*>(it->second->collider.get());
      if (sphere)
      {
        return sphere->getRadius();
      }
    }
  }

  return -1;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachSphereShape(
    const Identity &_linkID,
    const std::string &_name,
    const double _radius,
    const Pose3d &_pose)
{
  std::unique_ptr<btCollisionShape> shape =
    std::make_unique<btSphereShape>(_radius);

  auto identity = this->AddCollision(
    CollisionInfo{
      _name,
      std::move(shape),
      _linkID,
      _pose});

  return identity;
}

}
}
}
