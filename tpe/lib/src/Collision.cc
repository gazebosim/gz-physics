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

#include <ignition/common/Console.hh>

#include "Collision.hh"

/// \brief Private data class for Collision
class ignition::physics::tpelib::CollisionPrivate
{
  /// \brief Collision's geometry shape
  public: std::shared_ptr<Shape> shape = nullptr;

  /// \brief Collide bitmask
  public: uint16_t collideBitmask = 0xFF;
};

using namespace ignition;
using namespace physics;
using namespace tpelib;

//////////////////////////////////////////////////
Collision::Collision()
  : Entity(), dataPtr(new CollisionPrivate)
{
}

//////////////////////////////////////////////////
Collision::Collision(std::size_t _id)
  : Entity(_id), dataPtr(new CollisionPrivate)
{
}

//////////////////////////////////////////////////
Collision::Collision(const Collision &_other)
  : Entity(), dataPtr(new CollisionPrivate)
{
  this->dataPtr->shape = _other.dataPtr->shape;
}

//////////////////////////////////////////////////
Collision &Collision::operator=(const Collision &_other)
{
  this->dataPtr->shape = _other.dataPtr->shape;
  return *this;
}

//////////////////////////////////////////////////
Collision::~Collision()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void Collision::SetShape(const Shape &_shape)
{
  // \todo(anyone) use templates?
  if (_shape.GetType() == ShapeType::BOX)
  {
    const BoxShape *typedShape = static_cast<const BoxShape *>(&_shape);
    this->dataPtr->shape.reset(new BoxShape(*typedShape));
  }
  else if (_shape.GetType() == ShapeType::CAPSULE)
  {
    const CapsuleShape *typedShape =
      static_cast<const CapsuleShape *>(&_shape);
    this->dataPtr->shape.reset(new CapsuleShape(*typedShape));
  }
  else if (_shape.GetType() == ShapeType::CYLINDER)
  {
    const CylinderShape *typedShape =
      static_cast<const CylinderShape *>(&_shape);
    this->dataPtr->shape.reset(new CylinderShape(*typedShape));
  }
  else if (_shape.GetType() == ShapeType::ELLIPSOID)
  {
    const EllipsoidShape *typedShape =
      static_cast<const EllipsoidShape *>(&_shape);
    this->dataPtr->shape.reset(new EllipsoidShape(*typedShape));
  }
  else if (_shape.GetType() == ShapeType::SPHERE)
  {
    const SphereShape *typedShape = dynamic_cast<const SphereShape *>(&_shape);
    this->dataPtr->shape.reset(new SphereShape(*typedShape));
  }
  else if (_shape.GetType() == ShapeType::MESH)
  {
    const MeshShape *typedShape = dynamic_cast<const MeshShape *>(&_shape);
    this->dataPtr->shape.reset(new MeshShape(*typedShape));
  }
  else
  {
    ignwarn << "Failed to set shape." << std::endl;
  }
}

//////////////////////////////////////////////////
Shape *Collision::GetShape() const
{
  return this->dataPtr->shape.get();
}

//////////////////////////////////////////////////
math::AxisAlignedBox Collision::GetBoundingBox(bool /*_force*/) // NOLINT
{
  if (this->dataPtr->shape)
    return this->dataPtr->shape->GetBoundingBox();
  return math::AxisAlignedBox();
}

//////////////////////////////////////////////////
void Collision::SetCollideBitmask(uint16_t _mask)
{
  this->dataPtr->collideBitmask = _mask;
  if (this->GetParent())
    this->GetParent()->ChildrenChanged();
}

//////////////////////////////////////////////////
uint16_t Collision::GetCollideBitmask() const
{
  return this->dataPtr->collideBitmask;
}
