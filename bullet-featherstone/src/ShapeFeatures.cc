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

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <gz/math/eigen3/Conversions.hh>
#include <cassert>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/physics/heightmap/detail/SampleHeightmap.hh>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
AlignedBox3d ShapeFeatures::GetShapeAxisAlignedBoundingBox(
    const Identity &_shapeID) const
{
  GZ_PROFILE("ShapeFeatures::GetShapeAxisAlignedBoundingBox");
  const auto *collider = this->ReferenceInterface<CollisionInfo>(_shapeID);
  if (collider)
  {
    if (collider->heightmap)
    {
      // Bullet pads the box of a heightfield and does not move it to the
      // heights, so compute it.
      const auto &h = *collider->heightmap;
      return AlignedBox3d(
          Eigen::Vector3d(-h.size.x() / 2, -h.size.y() / 2, h.minHeight),
          Eigen::Vector3d(h.size.x() / 2, h.size.y() / 2,
              h.minHeight + h.size.z()));
    }
    btTransform t;
    t.setIdentity();
    btVector3 minBox(0, 0, 0);
    btVector3 maxBox(0, 0, 0);
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
    if (shape->getShapeType() == BOX_SHAPE_PROXYTYPE)
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetBoxShapeSize(
      const Identity &_boxID) const
{
  GZ_PROFILE("ShapeFeatures::GetBoxShapeSize");
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
  GZ_PROFILE("ShapeFeatures::AttachBoxShape");
  const btVector3 halfExtents = convertVec(_size * 0.5);
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
    if (shape->getShapeType() == CAPSULE_SHAPE_PROXYTYPE &&
        static_cast<const btCapsuleShape*>(shape.get())->getUpAxis() == 2)
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCapsuleShapeRadius(
    const Identity &_capsuleID) const
{
  GZ_PROFILE("ShapeFeatures::GetCapsuleShapeRadius");
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
  GZ_PROFILE("ShapeFeatures::GetCapsuleShapeLength");
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
  GZ_PROFILE("ShapeFeatures::AttachCapsuleShape");
  auto shape = std::make_unique<btCapsuleShapeZ>(
      static_cast<btScalar>(_radius), static_cast<btScalar>(_length / 2));

  auto identity = this->AddCollision(
    CollisionInfo{
      _name,
      std::move(shape),
      _linkID,
      _pose});

  return identity;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToConeShape(const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<CollisionInfo>(_shapeID);
  if (shapeInfo != nullptr)
  {
    const auto &shape = shapeInfo->collider;
    if (shape->getShapeType() == CONE_SHAPE_PROXYTYPE)
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetConeShapeRadius(
    const Identity &_coneID) const
{
  GZ_PROFILE("ShapeFeatures::GetConeShapeRadius");
  auto it = this->collisions.find(_coneID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *cone = static_cast<btConeShape*>(
        it->second->collider.get());
      if (cone)
      {
        return cone->getRadius();
      }
    }
  }

  return -1;
}

/////////////////////////////////////////////////
double ShapeFeatures::GetConeShapeHeight(
    const Identity &_coneID) const
{
  GZ_PROFILE("ShapeFeatures::GetConeShapeHeight");
  auto it = this->collisions.find(_coneID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *cone = static_cast<btConeShape*>(
        it->second->collider.get());
      if (cone)
      {
        return cone->getHeight();
      }
    }
  }

  return -1;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachConeShape(
    const Identity &_linkID,
    const std::string &_name,
    const double _radius,
    const double _height,
    const Pose3d &_pose)
{
  GZ_PROFILE("ShapeFeatures::AttachConeShape");
  const auto radius = static_cast<btScalar>(_radius);
  const auto height = static_cast<btScalar>(_height);
  auto shape =
    std::make_unique<btConeShapeZ>(radius, height);
  shape->setMargin(0.0);

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
    if (shape->getShapeType() == CYLINDER_SHAPE_PROXYTYPE)
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCylinderShapeRadius(
    const Identity &_cylinderID) const
{
  GZ_PROFILE("ShapeFeatures::GetCylinderShapeRadius");
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
  GZ_PROFILE("ShapeFeatures::GetCylinderShapeHeight");
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
  GZ_PROFILE("ShapeFeatures::AttachCylinderShape");
  const auto radius = static_cast<btScalar>(_radius);
  const auto halfLength = static_cast<btScalar>(_height * 0.5);
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
    if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Vector3d ShapeFeatures::GetEllipsoidShapeRadii(
    const Identity &_ellipsoidID) const
{
  GZ_PROFILE("ShapeFeatures::GetEllipsoidShapeRadii");
  auto it = this->collisions.find(_ellipsoidID);
  if (it != this->collisions.end() && it->second != nullptr)
  {
    if (it->second->collider != nullptr)
    {
      auto *ellipsoid = static_cast<btCompoundShape*>(
        it->second->collider.get());
      if (ellipsoid)
      {
        btVector3 aabbMin, aabbMax;
        btTransform tr;
        tr.setIdentity();
        ellipsoid->getAabb(tr, aabbMin, aabbMax);
        return Vector3d(aabbMax[0], aabbMax[1], aabbMax[2]);
      }
    }
  }

  return Vector3d(-1, -1, -1);
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachEllipsoidShape(
    const Identity &_linkID,
    const std::string &_name,
    const Vector3d &_radii,
    const Pose3d &_pose)
{
  GZ_PROFILE("ShapeFeatures::AttachEllipsoidShape");
  btVector3 positions[1];
  btScalar radius[1];
  positions[0] = btVector3(0, 0, 0);
  radius[0] = 1;

  auto btSphere = std::make_unique<btMultiSphereShape>(
    positions, radius, 1);
  btSphere->setLocalScaling(convertVec(_radii));
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
    if (shape->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
      return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  }

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetSphereShapeRadius(const Identity &_sphereID) const
{
  GZ_PROFILE("ShapeFeatures::GetSphereShapeRadius");
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
  GZ_PROFILE("ShapeFeatures::AttachSphereShape");
  std::unique_ptr<btCollisionShape> shape =
    std::make_unique<btSphereShape>(static_cast<btScalar>(_radius));

  auto identity = this->AddCollision(
    CollisionInfo{
      _name,
      std::move(shape),
      _linkID,
      _pose});

  return identity;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToHeightmapShape(const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<CollisionInfo>(_shapeID);
  if (shapeInfo != nullptr && shapeInfo->heightmap)
    return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetHeightmapShapeSize(
    const Identity &_heightmapID) const
{
  const auto *shapeInfo =
      this->ReferenceInterface<CollisionInfo>(_heightmapID);
  if (shapeInfo != nullptr && shapeInfo->heightmap)
    return shapeInfo->heightmap->size;

  return LinearVector3d::Zero();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachHeightmapShape(
    const Identity &_linkID,
    const std::string &_name,
    const common::HeightmapData &_heightmapData,
    const Pose3d &_pose,
    const LinearVector3d &_size,
    int _subSampling)
{
  GZ_PROFILE("ShapeFeatures::AttachHeightmapShape");
  auto *linkInfo = this->ReferenceInterface<LinkInfo>(_linkID);
  auto *modelInfo = this->ReferenceInterface<ModelInfo>(linkInfo->model);
  auto *worldInfo = this->ReferenceInterface<WorldInfo>(modelInfo->world);

  // Heightmaps are only supported on links that don't move: static, fixed to
  // the world or welded to a static link by AttachFixedJoint.
  if (!linkInfo->isStaticOrFixed && !isColliderStatic(linkInfo))
  {
    gzerr << "Unable to attach heightmap [" << _name << "] to link ["
          << linkInfo->name << "]: heightmaps are only supported on links "
          << "that are static or fixed to the world." << std::endl;
    return this->GenerateInvalidId();
  }
  // Every link has a collider once its model is constructed.
  assert(linkInfo->collider);

  // Bullet's row 0 is the -y edge.
  auto sampled = heightmap::detail::SampleHeightmap(
      _heightmapData, _size, _subSampling, true);
  if (!sampled)
  {
    gzerr << "Unable to attach heightmap [" << _name << "] to link ["
          << linkInfo->name << "]: failed to sample the heightmap data."
          << std::endl;
    return this->GenerateInvalidId();
  }
  const int vertSize = static_cast<int>(sampled->vertSize);

  HeightmapInfo heightmap{
      std::vector<btScalar>(sampled->heights.begin(), sampled->heights.end()),
      Eigen::Vector3d(_size.x(), _size.y(),
          static_cast<double>(sampled->maxHeight) - sampled->minHeight),
      static_cast<double>(sampled->minHeight)};

  // The shape keeps a pointer to the heights, which moving the vector into
  // the collision below does not change. This constructor reads PHY_FLOAT
  // data as btScalar.
  auto shape = std::make_unique<btHeightfieldTerrainShape>(
      vertSize, vertSize, heightmap.heights.data(), btScalar(1.0),
      sampled->minHeight, sampled->maxHeight, 2, PHY_FLOAT, false);
  // The samples span the whole size.
  shape->setLocalScaling(btVector3(
      static_cast<btScalar>(_size.x() / (vertSize - 1)),
      static_cast<btScalar>(_size.y() / (vertSize - 1)), btScalar(1.0)));

  // Bullet centers the heightfield vertically on its middle height.
  const Eigen::Isometry3d linkFrameToShape = _pose *
      Eigen::Translation3d(0, 0, heightmap.minHeight + heightmap.size.z() / 2);
  linkInfo->shape->addChildShape(
      convertTf(linkInfo->inertiaToLinkFrame * linkFrameToShape), shape.get());
  worldInfo->world->updateSingleAabb(linkInfo->collider.get());

  btCollisionShape *shapePtr = shape.get();
  CollisionInfo info{_name, std::move(shape), _linkID, _pose};
  info.heightmap = std::move(heightmap);
  const auto identity = this->AddCollision(std::move(info));
  // Used by GetContactsFromLastStep to find the collision
  shapePtr->setUserIndex(static_cast<int>(std::size_t(identity)));

  return identity;
}

}
}
}
