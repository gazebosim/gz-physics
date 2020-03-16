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

using namespace ignition;
using namespace physics;
using namespace tpe;
using namespace lib;

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
BoxShape::BoxShape(const BoxShape &_other)
  : Shape()
{
  *this = _other;
}

//////////////////////////////////////////////////
BoxShape::~BoxShape()
{
}

//////////////////////////////////////////////////
Shape &BoxShape::operator=(const Shape &_other)
{
  auto other = static_cast<const BoxShape *>(&_other);
  this->size = other->size;
  this->type = ShapeType::BOX;
  return *this;
}

//////////////////////////////////////////////////
void BoxShape::SetSize(const math::Vector3d &_size)
{
  this->size = _size;
  this->dirty = true;
}

//////////////////////////////////////////////////
void BoxShape::UpdateBoundingBox()
{
  math::Vector3d halfSize = this->size * 0.5;
  this->bbox = math::AxisAlignedBox(-halfSize, halfSize);
}

//////////////////////////////////////////////////
CylinderShape::CylinderShape() : Shape()
{
  this->type = ShapeType::CYLINDER;
}

//////////////////////////////////////////////////
CylinderShape::CylinderShape(const CylinderShape &_other)
  : Shape()
{
  *this = _other;
}

//////////////////////////////////////////////////
Shape &CylinderShape::operator=(const Shape &_other)
{
  auto other = static_cast<const CylinderShape *>(&_other);
  this->radius = other->radius;
  this->length = other->length;
  return *this;
}

//////////////////////////////////////////////////
double CylinderShape::GetRadius()
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
double CylinderShape::GetLength()
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
SphereShape::SphereShape() : Shape()
{
  this->type = ShapeType::SPHERE;
}

//////////////////////////////////////////////////
SphereShape::SphereShape(const SphereShape &_other)
  : Shape()
{
  *this = _other;
}

//////////////////////////////////////////////////
Shape &SphereShape::operator=(const Shape &_other)
{
  auto other = static_cast<const SphereShape *>(&_other);
  this->radius = other->radius;
  return *this;
}

//////////////////////////////////////////////////
double SphereShape::GetRadius()
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
MeshShape::MeshShape(const MeshShape &_other)
  : Shape()
{
  *this = _other;
}

//////////////////////////////////////////////////
Shape &MeshShape::operator=(const Shape &_other)
{
  auto other = static_cast<const MeshShape *>(&_other);
//  Shape::operator=(_other);
  this->scale = other->scale;
  this->meshAABB = other->meshAABB;
  return *this;
}

//////////////////////////////////////////////////
math::Vector3d MeshShape::GetScale()
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


