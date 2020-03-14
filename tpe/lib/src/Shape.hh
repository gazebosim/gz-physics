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

#ifndef IGNITION_PHYSICS_TPESIM_TPE_SHAPE_HH_
#define IGNITION_PHYSICS_TPESIM_TPE_SHAPE_HH_

#include <string>
#include <map>
#include <ignition/common/Mesh.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/AxisAlignedBox.hh>

namespace ignition {
namespace physics {
namespace tpe {

/// \enum ShapeType
/// \brief The set of shape types.
enum class ShapeType
{
  /// \brief Empty shpae . This means no shape has been defined.
  EMPTY = 0,

  /// \brief A box shape.
  BOX = 1,

  /// \brief A cylinder shape.
  CYLINDER = 2,

  /// \brief A plane shape.
  PLANE = 3,

  /// \brief A sphere shape.
  SPHERE = 4,

  /// \brief A mesh shape.
  MESH = 5,
};


/// \brief Base shape geometry class
class Shape
{
  /// \brief Constructor
  public: Shape();

  /// \brief Copy Constructor
  /// \param _other shape to copy from
  // public: Shape(const Shape &_other);

  /// \brief Destructor
  public: ~Shape() = default;

  /// \brief Get bounding box of shape
  /// \return Shape's bounding box
  public: virtual math::AxisAlignedBox GetBoundingBox();

  /// \brief Get type of shape
  /// \return Type of shape
  public: virtual ShapeType GetType() const;

  /// \brief Update the shape's bounding box
  protected: virtual void UpdateBoundingBox();

   /// \brief Bounding Box
  protected: math::AxisAlignedBox bbox;

   /// \brief Type of shape
  protected: ShapeType type;

   /// \brief Flag to indicate if dimensions changed
  protected: bool dirty = true;
};

/// \brief Box geometry
class BoxShape : public Shape
{
  /// \brief Constructor
  public: BoxShape();

  /// \brief Copy Constructor
  /// \param _other shape to copy from
  public: BoxShape(const BoxShape &_other);

  /// \brief Destructor
  public: ~BoxShape();

  /// \brief Assignment operator
  /// \param _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Set size of box
  /// \param _size Size of box
  public: void SetSize(const math::Vector3d &_size);

  /// \brief Get size of box
  /// \return _size Size of box
  public: math::Vector3d GetSize();

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  /// \brief Size of box
  private: math::Vector3d size;
};

/// \brief Cylinder geometry
class CylinderShape : public Shape
{
  /// \brief Constructor
  public: CylinderShape();

  /// \brief Copy Constructor
  /// \param _other shape to copy from
  public: CylinderShape(const CylinderShape &_other);

  /// \brief Destructor
  public: ~CylinderShape() = default;

  /// \brief Assignment operator
  /// \param _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Get cylinder radius
  /// \return _radius cylinder radius
  public: double GetRadius();

  /// \brief Set cylinder radius
  /// \param _radius Cylinder radius
  public: void SetRadius(double _radius);

  /// \brief Get cylinder length
  /// \return _length Cylinder length
  public: double GetLength();

  /// \brief Set cylinder length
  /// \param _length Cylinder length
  public: void SetLength(double _length);

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  /// \brief Cylinder radius
  private: double radius = 0.0;

  /// \brief Cylinder length
  private: double length = 0.0;
};

/// \brief Sphere geometry
class SphereShape : public Shape
{
  /// \brief Constructor
  public: SphereShape();

  /// \brief Copy Constructor
  /// \param _other shape to copy from
  public: SphereShape(const SphereShape &_other);

  /// \brief Destructor
  public: ~SphereShape() = default;

  /// \brief Assignment operator
  /// \param _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Get sphere radius
  /// \return _radius Sphere radius
  public: double GetRadius();

  /// \brief Set sphere radius
  /// \param _radius Sphere radius
  public: void SetRadius(double _radius);

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  /// \brief Sphere radius
  private: double radius = 0.0;
};

/// \brief Mesh geometry
class MeshShape : public Shape
{
  /// \brief Constructor
  public: MeshShape();

  /// \brief Copy Constructor
  /// \param _other shape to copy from
  public: MeshShape(const MeshShape &_other);

  /// \brief Destructor
  public: ~MeshShape() = default;

  /// \brief Assignment operator
  /// \param _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Set mesh
  /// \param _mesh Mesh object
  public: void SetMesh(const ignition::common::Mesh &_mesh);

  /// \brief Get mesh scale
  /// \return _scale Mesh scale
  public: math::Vector3d GetScale();

  /// \brief Set mesh scale
  /// \param _scale Mesh scale
  public: void SetScale(math::Vector3d _scale);

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  /// \brief Mesh scale
  private: math::Vector3d scale{1.0, 1.0, 1.0};

  /// \brief Mesh object
  private: math::AxisAlignedBox meshAABB;
};
}
}
}

#endif
