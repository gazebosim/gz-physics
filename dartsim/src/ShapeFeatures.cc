/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include "CustomMeshShape.hh"
#include "ShapeFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
Pose3d ShapeFeatures::GetShapeRelativeTransform(
    const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  return shapeInfo->node->getRelativeTransform() *
         shapeInfo->tf_offset.inverse();
}

/////////////////////////////////////////////////
void ShapeFeatures::SetShapeRelativeTransform(
    const Identity &_shapeID, const Pose3d &_pose)
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  shapeInfo->node->setRelativeTransform(_pose * shapeInfo->tf_offset);
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToBoxShape(const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);

  const dart::dynamics::ShapePtr &shape = shapeInfo->node->getShape();

  if (dynamic_cast<dart::dynamics::BoxShape*>(shape.get()))
    return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetBoxShapeSize(
    const Identity &_boxID) const
{
  const auto *boxInfo = this->ReferenceInterface<ShapeInfo>(_boxID);
  dart::dynamics::BoxShape *box = static_cast<dart::dynamics::BoxShape*>(
        boxInfo->node->getShape().get());

  return box->getSize();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachBoxShape(
    const Identity &_linkID,
    const std::string &_name,
    const LinearVector3d &_size,
    const Pose3d &_pose)
{
  auto box = std::make_shared<dart::dynamics::BoxShape>(_size);

  DartBodyNode *bn = this->ReferenceInterface<LinkInfo>(_linkID)->link.get();
  dart::dynamics::ShapeNode *sn =
      bn->createShapeNodeWith<dart::dynamics::CollisionAspect,
                              dart::dynamics::DynamicsAspect>(
          box, bn->getName() + ":" + _name);

  sn->setRelativeTransform(_pose);
  const std::size_t shapeID = this->AddShape({sn, _name});
  return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToCylinderShape(const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);

  const dart::dynamics::ShapePtr &shape = shapeInfo->node->getShape();

  if (dynamic_cast<dart::dynamics::CylinderShape *>(shape.get()))
    return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCylinderShapeRadius(
    const Identity &_cylinderID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_cylinderID);

  dart::dynamics::CylinderShape *cylinder =
      static_cast<dart::dynamics::CylinderShape *>(
          shapeInfo->node->getShape().get());

  return cylinder->getRadius();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCylinderShapeHeight(
    const Identity &_cylinderID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_cylinderID);
  dart::dynamics::CylinderShape *cylinder =
      static_cast<dart::dynamics::CylinderShape *>(
          shapeInfo->node->getShape().get());

  return cylinder->getHeight();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachCylinderShape(
    const Identity &_linkID,
    const std::string &_name,
    const double _radius,
    const double _height,
    const Pose3d &_pose)
{
  auto cylinder = std::make_shared<dart::dynamics::CylinderShape>(
        _radius, _height);

  auto bn = this->ReferenceInterface<LinkInfo>(_linkID)->link;
  dart::dynamics::ShapeNode *sn =
      bn->createShapeNodeWith<dart::dynamics::CollisionAspect,
                              dart::dynamics::DynamicsAspect>(
          cylinder, bn->getName() + ":" + _name);

  sn->setRelativeTransform(_pose);

  const std::size_t shapeID = this->AddShape({sn, _name});
  return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToSphereShape(
    const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);

  const dart::dynamics::ShapePtr &shape =
      shapeInfo->node->getShape();

  if (dynamic_cast<dart::dynamics::SphereShape *>(shape.get()))
    return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetSphereShapeRadius(const Identity &_sphereID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_sphereID);

  dart::dynamics::SphereShape *sphere =
      static_cast<dart::dynamics::SphereShape*>(
        shapeInfo->node->getShape().get());

  return sphere->getRadius();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachSphereShape(
    const Identity &_linkID,
    const std::string &_name,
    const double _radius,
    const Pose3d &_pose)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(_radius);

  DartBodyNode *bn = this->ReferenceInterface<LinkInfo>(_linkID)->link.get();
  dart::dynamics::ShapeNode *sn =
      bn->createShapeNodeWith<dart::dynamics::CollisionAspect,
                              dart::dynamics::DynamicsAspect>(
          sphere, bn->getName() + ":" + _name);

  sn->setRelativeTransform(_pose);
  const std::size_t shapeID = this->AddShape({sn, _name});
  return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToMeshShape(
    const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);

  const dart::dynamics::ShapePtr &shape =
      shapeInfo->node->getShape();

  if (dynamic_cast<dart::dynamics::MeshShape*>(shape.get()))
    return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetMeshShapeSize(
    const Identity &_meshID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_meshID);

  const dart::dynamics::MeshShape *mesh =
      static_cast<dart::dynamics::MeshShape*>(
        shapeInfo->node->getShape().get());

  return mesh->getBoundingBox().getMax() - mesh->getBoundingBox().getMin();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetMeshShapeScale(
    const Identity &_meshID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_meshID);

  const dart::dynamics::MeshShape *mesh =
      static_cast<dart::dynamics::MeshShape*>(
        shapeInfo->node->getShape().get());

  return mesh->getScale();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachMeshShape(
    const Identity &_linkID,
    const std::string &_name,
    const ignition::common::Mesh &_mesh,
    const Pose3d &_pose,
    const LinearVector3d &_scale)
{
  auto mesh = std::make_shared<CustomMeshShape>(_mesh, _scale);

  DartBodyNode *bn = this->ReferenceInterface<LinkInfo>(_linkID)->link.get();
  dart::dynamics::ShapeNode *sn =
      bn->createShapeNodeWith<dart::dynamics::CollisionAspect,
                              dart::dynamics::DynamicsAspect>(
          mesh, bn->getName() + ":" + _name);

  sn->setRelativeTransform(_pose);
  const std::size_t shapeID = this->AddShape({sn, _name});
  return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToPlaneShape(const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);

  const dart::dynamics::ShapePtr &shape =
      shapeInfo->node->getShape();

  if (dynamic_cast<dart::dynamics::PlaneShape*>(shape.get()))
    return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetPlaneShapeNormal(
    const Identity &_planeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_planeID);

  const dart::dynamics::PlaneShape *plane =
      static_cast<dart::dynamics::PlaneShape*>(
        shapeInfo->node->getShape().get());

  return plane->getNormal();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetPlaneShapePoint(
    const Identity &_planeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_planeID);

  const dart::dynamics::PlaneShape *plane =
      static_cast<dart::dynamics::PlaneShape*>(
        shapeInfo->node->getShape().get());

  return plane->getOffset() * plane->getNormal();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachPlaneShape(
    const Identity &_linkID,
    const std::string &_name,
    const AngularVector3d &_normal,
    const LinearVector3d &_point)
{
  auto plane = std::make_shared<dart::dynamics::PlaneShape>(_normal, _point);

  DartBodyNode *bn = this->ReferenceInterface<LinkInfo>(_linkID)->link.get();
  dart::dynamics::ShapeNode *sn =
      bn->createShapeNodeWith<dart::dynamics::CollisionAspect,
                              dart::dynamics::DynamicsAspect>(
          plane, bn->getName() + ":" + _name);

  const std::size_t shapeID = this->AddShape({sn, _name});
  return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
}

/////////////////////////////////////////////////
AlignedBox3d ShapeFeatures::GetShapeAxisAlignedBoundingBox(
    const Identity &_shapeID) const
{
  const auto &node = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  const dart::math::BoundingBox &box = node->getShape()->getBoundingBox();
  return AlignedBox3d(box.getMin(), box.getMax());
}

#if DART_VERSION_AT_LEAST(6, 10, 0)
/////////////////////////////////////////////////
double ShapeFeatures::GetShapeFrictionPyramidPrimarySlipCompliance(
    const Identity &_shapeID) const
{
  auto &node = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  auto aspect = node->getDynamicsAspect();
  if (nullptr == aspect)
  {
    ignerr
      << "Attempt to get FrictionPyramidPrimarySlipCompliance for a "
      << "ShapeNode that doesn't have a DynamicAspect. "
      << "Returning default value of 0.0."
      << std::endl;
    return 0.0;
  }
  return aspect->getSlipCompliance();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetShapeFrictionPyramidSecondarySlipCompliance(
    const Identity &_shapeID) const
{
  auto &node = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  auto aspect = node->getDynamicsAspect();
  if (nullptr == aspect)
  {
    ignerr
      << "Attempt to get FrictionPyramidSecondarySlipCompliance for a "
      << "ShapeNode that doesn't have a DynamicAspect. "
      << "Returning default value of 0.0."
      << std::endl;
    return 0.0;
  }
  return aspect->getSecondarySlipCompliance();
}

/////////////////////////////////////////////////
bool ShapeFeatures::SetShapeFrictionPyramidPrimarySlipCompliance(
    const Identity &_shapeID, double _value)
{
  auto &node = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  auto aspect = node->getDynamicsAspect();
  if (nullptr == aspect)
  {
    ignerr
      << "Attempt to set FrictionPyramidPrimarySlipCompliance for a "
      << "ShapeNode that doesn't have a DynamicAspect. "
      << "The parameter has not been set."
      << std::endl;
    return false;
  }
  aspect->setSlipCompliance(_value);
  return true;
}

/////////////////////////////////////////////////
bool ShapeFeatures::SetShapeFrictionPyramidSecondarySlipCompliance(
    const Identity &_shapeID, double _value)
{
  auto &node = this->ReferenceInterface<ShapeInfo>(_shapeID)->node;
  auto aspect = node->getDynamicsAspect();
  if (nullptr == aspect)
  {
    ignerr
      << "Attempt to set FrictionPyramidSecondarySlipCompliance for a "
      << "ShapeNode that doesn't have a DynamicAspect. "
      << "The parameter has not been set."
      << std::endl;
    return false;
  }
  aspect->setSecondarySlipCompliance(_value);
  return true;
}
#endif

}
}
}
