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

#ifndef IGNITION_PHYSICS_TPE_PLUGIN_SRC_SHAPEFEATURES_HH_
#define IGNITION_PHYSICS_TPE_PLUGIN_SRC_SHAPEFEATURES_HH_

#include <string>

#include <ignition/physics/Shape.hh>
#include <ignition/physics/BoxShape.hh>
#include <ignition/physics/CylinderShape.hh>
#include <ignition/physics/mesh/MeshShape.hh>
#include <ignition/physics/SphereShape.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace tpeplugin {

struct ShapeFeatureList : FeatureList<
  GetBoxShapeProperties,
  AttachBoxShapeFeature,
  GetShapeBoundingBox,

  GetCylinderShapeProperties,
  AttachCylinderShapeFeature,

  GetSphereShapeProperties,
  AttachSphereShapeFeature,

  mesh::GetMeshShapeProperties,
  mesh::AttachMeshShapeFeature
> { };

class ShapeFeatures :
    public virtual Base,
    public virtual Implements3d<ShapeFeatureList>
{
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


  // ----- Mesh Features -----
  public: Identity CastToMeshShape(
    const Identity &_shapeID) const override;

  public: LinearVector3d GetMeshShapeSize(
    const Identity &_meshID) const override;

  public: LinearVector3d GetMeshShapeScale(
    const Identity &_meshID) const override;

  public: Identity AttachMeshShape(
    const Identity &_linkID,
    const std::string &_name,
    const ignition::common::Mesh &_mesh,
    const Pose3d &_pose,
    const LinearVector3d &_scale) override;

  // ----- Boundingbox Features -----
  public: AlignedBox3d GetShapeAxisAlignedBoundingBox(
    const Identity &_shapeID) const override;
};

}
}
}

#endif
