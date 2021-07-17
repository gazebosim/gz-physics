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

#ifndef IGNITION_PHYSICS_TPE_LIB_SRC_SHAPE_HH_
#define IGNITION_PHYSICS_TPE_LIB_SRC_SHAPE_HH_

#include <string>
#include <map>

#include <ignition/common/Mesh.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/utils/SuppressWarning.hh>

#include "ignition/physics/tpelib/Export.hh"

namespace ignition {
namespace physics {
namespace tpelib {

/// \enum ShapeType
/// \brief The set of shape types.
enum class IGNITION_PHYSICS_TPELIB_VISIBLE ShapeType
{
  /// \brief Empty shape. This means no shape has been defined.
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

  /// \brief A capsule shape.
  CAPSULE = 6,

  /// \brief A ellipsoid shape.
  ELLIPSOID = 7,
};


/// \brief Base shape geometry class
class IGNITION_PHYSICS_TPELIB_VISIBLE Shape
{
  /// \brief Constructor
  public: Shape();

  /// \brief Destructor
  public: virtual ~Shape() = default;

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
class IGNITION_PHYSICS_TPELIB_VISIBLE BoxShape : public Shape
{
  /// \brief Constructor
  public: BoxShape();

  /// \brief Destructor
  public: virtual ~BoxShape() = default;

  /// \brief Assignment operator
  /// \param[in] _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Set size of box
  /// \param[in] _size Size of box
  public: void SetSize(const math::Vector3d &_size);

  /// \brief Get size of box
  /// \return Size of box
  public: math::Vector3d GetSize();

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief Size of box
  private: math::Vector3d size;
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

/// \brief Capsule geometry
class IGNITION_PHYSICS_TPELIB_VISIBLE CapsuleShape : public Shape
{
  /// \brief Constructor
  public: CapsuleShape();

  /// \brief Copy Constructor
  /// \param[in] _other shape to copy from
  public: CapsuleShape(const CapsuleShape &_other);

  /// \brief Destructor
  public: ~CapsuleShape() = default;

  /// \brief Assignment operator
  /// \param[in] _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Get capsule radius
  /// \return capsule radius
  public: double GetRadius() const;

  /// \brief Set capsule radius
  /// \param[in] _radius Cylinder radius
  public: void SetRadius(double _radius);

  /// \brief Get capsule length
  /// \return Capsule length
  public: double GetLength() const;

  /// \brief Set capsule length
  /// \param[in] _length Cylinder length
  public: void SetLength(double _length);

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  /// \brief Capsule radius
  private: double radius = 0.0;

  /// \brief Capsule length
  private: double length = 0.0;
};

/// \brief Cylinder geometry
class IGNITION_PHYSICS_TPELIB_VISIBLE CylinderShape : public Shape
{
  /// \brief Constructor
  public: CylinderShape();

  /// \brief Destructor
  public: virtual ~CylinderShape() = default;

  /// \brief Assignment operator
  /// \param[in] _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Get cylinder radius
  /// \return cylinder radius
  public: double GetRadius() const;

  /// \brief Set cylinder radius
  /// \param[in] _radius Cylinder radius
  public: void SetRadius(double _radius);

  /// \brief Get cylinder length
  /// \return Cylinder length
  public: double GetLength() const;

  /// \brief Set cylinder length
  /// \param[in] _length Cylinder length
  public: void SetLength(double _length);

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  /// \brief Cylinder radius
  private: double radius = 0.0;

  /// \brief Cylinder length
  private: double length = 0.0;
};

/// \brief Ellipsoid geometry
class IGNITION_PHYSICS_TPELIB_VISIBLE EllipsoidShape : public Shape
{
  /// \brief Constructor
  public: EllipsoidShape();

  /// \brief Copy Constructor
  /// \param[in] _other shape to copy from
  public: EllipsoidShape(const EllipsoidShape &_other);

  /// \brief Destructor
  public: ~EllipsoidShape() = default;

  /// \brief Assignment operator
  /// \param[in] _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Get ellipsoid radius
  /// \return ellipsoid radius
  public: math::Vector3d GetRadii() const;

  /// \brief Set ellipsoid radius
  /// \param[in] _radius ellipsoid radius
  public: void SetRadii(const math::Vector3d &_radii);

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief ellipsoid radius
  private: math::Vector3d radii = math::Vector3d::Zero;
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

/// \brief Sphere geometry
class IGNITION_PHYSICS_TPELIB_VISIBLE SphereShape : public Shape
{
  /// \brief Constructor
  public: SphereShape();

  /// \brief Destructor
  public: virtual ~SphereShape() = default;

  /// \brief Assignment operator
  /// \param[in] _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Get sphere radius
  /// \return Sphere radius
  public: double GetRadius() const;

  /// \brief Set sphere radius
  /// \param[in] _radius Sphere radius
  public: void SetRadius(double _radius);

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  /// \brief Sphere radius
  private: double radius = 0.0;
};

/// \brief Mesh geometry
class IGNITION_PHYSICS_TPELIB_VISIBLE MeshShape : public Shape
{
  /// \brief Constructor
  public: MeshShape();

  /// \brief Destructor
  public: virtual ~MeshShape() = default;

  /// \brief Assignment operator
  /// \param[in] _other shape to copy from
  public: Shape &operator=(const Shape &_other);

  /// \brief Set mesh
  /// \param[in] _mesh Mesh object
  public: void SetMesh(const ignition::common::Mesh &_mesh);

  /// \brief Get mesh scale
  /// \return Mesh scale
  public: math::Vector3d GetScale() const;

  /// \brief Set mesh scale
  /// \param[in] _scale Mesh scale
  public: void SetScale(math::Vector3d _scale);

  // Documentation inherited
  protected: virtual void UpdateBoundingBox() override;

  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief Mesh scale
  private: math::Vector3d scale{1.0, 1.0, 1.0};
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING

  /// \brief Mesh object
  private: math::AxisAlignedBox meshAABB;
};

}
}
}

#endif
