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
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include "CustomMeshShape.hh"
#include "ShapeFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
Pose3d ShapeFeatures::GetShapeRelativeTransform(
    const std::size_t _shapeID) const
{
  const auto &shape = this->shapes.at(_shapeID);
  return shape.node->getRelativeTransform() * shape.tf_offset.inverse();
}

/////////////////////////////////////////////////
void ShapeFeatures::SetShapeRelativeTransform(
    const std::size_t _shapeID, const Pose3d &_pose)
{
  const auto &shape = this->shapes.at(_shapeID);
  shape.node->setRelativeTransform(_pose * shape.tf_offset);
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToBoxShape(std::size_t _shapeID) const
{
  const dart::dynamics::ShapePtr &shape =
      this->shapes.at(_shapeID).node->getShape();

  if (dynamic_cast<dart::dynamics::BoxShape*>(shape.get()))
    return this->GenerateIdentity(_shapeID);

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetBoxShapeSize(
    const std::size_t _boxID) const
{
  dart::dynamics::BoxShape *box = static_cast<dart::dynamics::BoxShape*>(
        this->shapes.at(_boxID).node->getShape().get());

  return box->getSize();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachBoxShape(
    const std::size_t _linkID,
    const std::string &_name,
    const LinearVector3d &_size,
    const Pose3d &_pose)
{
  auto box = std::make_shared<dart::dynamics::BoxShape>(_size);

  DartBodyNode *bn = this->links.at(_linkID);
  dart::dynamics::ShapeNode *sn =
      bn->createShapeNodeWith<dart::dynamics::CollisionAspect>(
        box, bn->getName() + ":" + _name);

  sn->setRelativeTransform(_pose);
  return this->GenerateIdentity(this->AddShape({sn, _name}));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToCylinderShape(const std::size_t _shapeID) const
{
  const dart::dynamics::ShapePtr &shape =
      this->shapes.at(_shapeID).node->getShape();

  if (dynamic_cast<dart::dynamics::CylinderShape*>(shape.get()))
    return this->GenerateIdentity(_shapeID);

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCylinderShapeRadius(
    const std::size_t _cylinderID) const
{
  dart::dynamics::CylinderShape *cylinder =
      static_cast<dart::dynamics::CylinderShape*>(
        this->shapes.at(_cylinderID).node->getShape().get());

  return cylinder->getRadius();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCylinderShapeHeight(
    const std::size_t _cylinderID) const
{
  dart::dynamics::CylinderShape *cylinder =
      static_cast<dart::dynamics::CylinderShape*>(
        this->shapes.at(_cylinderID).node->getShape().get());

  return cylinder->getHeight();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachCylinderShape(
    const std::size_t _linkID,
    const std::string &_name,
    const double _radius,
    const double _height,
    const Pose3d &_pose)
{
  auto cylinder = std::make_shared<dart::dynamics::CylinderShape>(
        _radius, _height);

  DartBodyNode *bn = this->links.at(_linkID);
  dart::dynamics::ShapeNode *sn =
      bn->createShapeNodeWith<dart::dynamics::CollisionAspect>(
        cylinder, bn->getName() + ":" + _name);

  sn->setRelativeTransform(_pose);
  return this->GenerateIdentity(this->AddShape({sn, _name}));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToSphereShape(
    const std::size_t _shapeID) const
{
  const dart::dynamics::ShapePtr &shape =
      this->shapes.at(_shapeID).node->getShape();

  if (dynamic_cast<dart::dynamics::SphereShape*>(shape.get()))
    return this->GenerateIdentity(_shapeID);

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetSphereShapeRadius(const std::size_t _sphereID) const
{
  dart::dynamics::SphereShape *sphere =
      static_cast<dart::dynamics::SphereShape*>(
        this->shapes.at(_sphereID).node->getShape().get());

  return sphere->getRadius();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachSphereShape(
    const std::size_t _linkID,
    const std::string &_name,
    const double _radius,
    const Pose3d &_pose)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(_radius);

  DartBodyNode *bn = this->links.at(_linkID);
  dart::dynamics::ShapeNode *sn =
      bn->createShapeNodeWith<dart::dynamics::CollisionAspect>(
        sphere, bn->getName() + ":" + _name);

  sn->setRelativeTransform(_pose);
  return this->GenerateIdentity(this->AddShape({sn, _name}));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToMeshShape(
    const std::size_t _shapeID) const
{
  const dart::dynamics::ShapePtr &shape =
      this->shapes.at(_shapeID).node->getShape();

  if (dynamic_cast<dart::dynamics::MeshShape*>(shape.get()))
    return this->GenerateIdentity(_shapeID);

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetMeshShapeSize(
    const std::size_t _meshID) const
{
  dart::dynamics::MeshShape *mesh =
      static_cast<dart::dynamics::MeshShape*>(
        this->shapes.at(_meshID).node->getShape().get());

  return mesh->getBoundingBox().getMax() - mesh->getBoundingBox().getMin();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetMeshShapeScale(
    const std::size_t _meshID) const
{
  dart::dynamics::MeshShape *mesh =
      static_cast<dart::dynamics::MeshShape*>(
        this->shapes.at(_meshID).node->getShape().get());

  return mesh->getScale();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachMeshShape(
    const std::size_t _linkID,
    const std::string &_name,
    const ignition::common::Mesh &_mesh,
    const Pose3d &_pose)
{
  auto mesh = std::make_shared<CustomMeshShape>(_mesh);

  DartBodyNode *bn = this->links.at(_linkID);
  dart::dynamics::ShapeNode *sn =
      bn->createShapeNodeWith<dart::dynamics::CollisionAspect>(
        mesh, bn->getName() + ":" + _name);

  sn->setRelativeTransform(_pose);
  return this->GenerateIdentity(this->AddShape({sn, _name}));
}

}
}
}
