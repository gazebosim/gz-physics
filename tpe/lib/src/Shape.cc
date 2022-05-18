/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "Shape.hh"

using namespace gz;
using namespace physics;
using namespace tpelib;

//////////////////////////////////////////////////
Shape::Shape()
{
  this->type = ShapeType::EMPTY;
}

//////////////////////////////////////////////////
math::AxisAlignedBox Shape::GetBoundingBox()
{
  if (this->dirty)
  {
    this->UpdateBoundingBox();
    this->dirty = false;
  }
  return this->bbox;
}

//////////////////////////////////////////////////
void Shape::UpdateBoundingBox()
{
  // No op. To be overriden by derived classes
}

//////////////////////////////////////////////////
ShapeType Shape::GetType() const
{
  return this->type;
}

//////////////////////////////////////////////////
BoxShape::BoxShape() : Shape()
{
  this->type = ShapeType::BOX;
}

//////////////////////////////////////////////////
void BoxShape::SetSize(const math::Vector3d &_size)
{
  this->size = _size;
  this->dirty = true;
}

//////////////////////////////////////////////////
math::Vector3d BoxShape::GetSize()
{
  return this->size;
}

//////////////////////////////////////////////////
void BoxShape::UpdateBoundingBox()
{
  math::Vector3d halfSize = this->size * 0.5;
  this->bbox = math::AxisAlignedBox(-halfSize, halfSize);
}

//////////////////////////////////////////////////
CapsuleShape::CapsuleShape() : Shape()
{
  this->type = ShapeType::CAPSULE;
}

//////////////////////////////////////////////////
double CapsuleShape::GetRadius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
void CapsuleShape::SetRadius(double _radius)
{
  this->radius = _radius;
  this->dirty = true;
}

//////////////////////////////////////////////////
double CapsuleShape::GetLength() const
{
  return this->length;
}

//////////////////////////////////////////////////
void CapsuleShape::SetLength(double _length)
{
  this->length = _length;
  this->dirty = true;
}

//////////////////////////////////////////////////
void CapsuleShape::UpdateBoundingBox()
{
  math::Vector3d halfSize(this->radius, this->radius, this->length*0.5 + this->radius);
  this->bbox = math::AxisAlignedBox(-halfSize, halfSize);
}

//////////////////////////////////////////////////
CylinderShape::CylinderShape() : Shape()
{
  this->type = ShapeType::CYLINDER;
}

//////////////////////////////////////////////////
double CylinderShape::GetRadius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
void CylinderShape::SetRadius(double _radius)
{
  this->radius = _radius;
  this->dirty = true;
}

//////////////////////////////////////////////////
double CylinderShape::GetLength() const
{
  return this->length;
}

//////////////////////////////////////////////////
void CylinderShape::SetLength(double _length)
{
  this->length = _length;
  this->dirty = true;
}

//////////////////////////////////////////////////
void CylinderShape::UpdateBoundingBox()
{
  math::Vector3d halfSize(this->radius, this->radius, this->length*0.5);
  this->bbox = math::AxisAlignedBox(-halfSize, halfSize);
}

//////////////////////////////////////////////////
EllipsoidShape::EllipsoidShape() : Shape()
{
  this->type = ShapeType::ELLIPSOID;
}

//////////////////////////////////////////////////
math::Vector3d EllipsoidShape::GetRadii() const
{
  return this->radii;
}

//////////////////////////////////////////////////
void EllipsoidShape::SetRadii(const math::Vector3d &_radii)
{
  this->radii = _radii;
  this->dirty = true;
}

//////////////////////////////////////////////////
void EllipsoidShape::UpdateBoundingBox()
{
  math::Vector3d halfSize(this->radii.X(), this->radii.Y(), this->radii.Z());
  this->bbox = math::AxisAlignedBox(-halfSize, halfSize);
}

//////////////////////////////////////////////////
SphereShape::SphereShape() : Shape()
{
  this->type = ShapeType::SPHERE;
}

//////////////////////////////////////////////////
double SphereShape::GetRadius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
void SphereShape::SetRadius(double _radius)
{
  this->radius = _radius;
  this->dirty = true;
}

//////////////////////////////////////////////////
void SphereShape::UpdateBoundingBox()
{
  math::Vector3d halfSize(this->radius, this->radius, this->radius);
  this->bbox = math::AxisAlignedBox(-halfSize, halfSize);
}

//////////////////////////////////////////////////
MeshShape::MeshShape() : Shape()
{
  this->type = ShapeType::MESH;
}

//////////////////////////////////////////////////
math::Vector3d MeshShape::GetScale() const
{
  return this->scale;
}

//////////////////////////////////////////////////
void MeshShape::SetScale(math::Vector3d _scale)
{
  this->scale = _scale;
  this->dirty = true;
}

//////////////////////////////////////////////////
void MeshShape::SetMesh(const common::Mesh &_mesh)
{
  math::Vector3d center;
  math::Vector3d min;
  math::Vector3d max;
  _mesh.AABB(center, min, max);
  this->meshAABB = math::AxisAlignedBox(min, max);
  this->dirty = true;
}

//////////////////////////////////////////////////
void MeshShape::UpdateBoundingBox()
{
  this->bbox = math::AxisAlignedBox(
      this->scale * this->meshAABB.Min(), this->scale * this->meshAABB.Max());
}
