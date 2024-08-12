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

#ifndef GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_SHAPEFEATURES_HH_
#define GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_SHAPEFEATURES_HH_

#include <gz/physics/Shape.hh>
#include <gz/physics/BoxShape.hh>
#include <gz/physics/CapsuleShape.hh>
#include <gz/physics/ConeShape.hh>
#include <gz/physics/CylinderShape.hh>
#include <gz/physics/EllipsoidShape.hh>
#include <gz/physics/SphereShape.hh>

#include <string>

#include "Base.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {

struct ShapeFeatureList : FeatureList<
  GetShapeBoundingBox,

  GetBoxShapeProperties,
  AttachBoxShapeFeature,

  GetCapsuleShapeProperties,
  AttachCapsuleShapeFeature,

  GetConeShapeProperties,
  AttachConeShapeFeature,

  GetCylinderShapeProperties,
  AttachCylinderShapeFeature,

  GetEllipsoidShapeProperties,
  AttachEllipsoidShapeFeature,

  GetSphereShapeProperties,
  AttachSphereShapeFeature
> { };

class ShapeFeatures :
    public virtual Base,
    public virtual Implements3d<ShapeFeatureList>
{
  // ----- Boundingbox Features -----
  public: AlignedBox3d GetShapeAxisAlignedBoundingBox(
              const Identity &_shapeID) const override;

  // ----- Box Features -----
  public: Identity CastToBoxShape(
      const Identity &_shapeID) const override;

  public: LinearVector3d GetBoxShapeSize(
      const Identity &_boxID) const override;

  public: Identity AttachBoxShape(
      const Identity &_linkID,
      const std::string &_name,
      const LinearVector3d &_size,
      const Pose3d &_pose) override;

  // ----- Capsule Features -----
  public: Identity CastToCapsuleShape(
      const Identity &_shapeID) const override;

  public: double GetCapsuleShapeRadius(
      const Identity &_capsuleID) const override;

  public: double GetCapsuleShapeLength(
      const Identity &_capsuleID) const override;

  public: Identity AttachCapsuleShape(
      const Identity &_linkID,
      const std::string &_name,
      double _radius,
      double _length,
      const Pose3d &_pose) override;

  // ----- Cone Features -----
  public: Identity CastToConeShape(
      const Identity &_shapeID) const override;

  public: double GetConeShapeRadius(
      const Identity &_coneID) const override;

  public: double GetConeShapeHeight(
      const Identity &_coneID) const override;

  public: Identity AttachConeShape(
      const Identity &_linkID,
      const std::string &_name,
      double _radius,
      double _height,
      const Pose3d &_pose) override;

  // ----- Cylinder Features -----
  public: Identity CastToCylinderShape(
      const Identity &_shapeID) const override;

  public: double GetCylinderShapeRadius(
      const Identity &_cylinderID) const override;

  public: double GetCylinderShapeHeight(
      const Identity &_cylinderID) const override;

  public: Identity AttachCylinderShape(
      const Identity &_linkID,
      const std::string &_name,
      double _radius,
      double _height,
      const Pose3d &_pose) override;

  // ----- Ellipsoid Features -----
  public: Identity CastToEllipsoidShape(
      const Identity &_shapeID) const override;

  public: Vector3d GetEllipsoidShapeRadii(
      const Identity &_ellipsoidID) const override;

  public: Identity AttachEllipsoidShape(
      const Identity &_linkID,
      const std::string &_name,
      const Vector3d &_radii,
      const Pose3d &_pose) override;

  // ----- Sphere Features -----
  public: Identity CastToSphereShape(
      const Identity &_shapeID) const override;

  public: double GetSphereShapeRadius(
      const Identity &_sphereID) const override;

  public: Identity AttachSphereShape(
      const Identity &_linkID,
      const std::string &_name,
      double _radius,
      const Pose3d &_pose) override;
};

}
}
}

#endif
