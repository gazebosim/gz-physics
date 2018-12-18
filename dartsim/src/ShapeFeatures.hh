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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_SHAPEFEATURES_HH_
#define IGNITION_PHYSICS_DARTSIM_SRC_SHAPEFEATURES_HH_

#include <string>

#include <ignition/physics/Shape.hh>
#include <ignition/physics/BoxShape.hh>
#include <ignition/physics/CylinderShape.hh>
#include <ignition/physics/mesh/MeshShape.hh>
#include <ignition/physics/SphereShape.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using ShapeFeatureList = FeatureList<
  GetShapeKinematicProperties,
  SetShapeKinematicProperties,
  GetBoxShapeProperties,
  // dartsim cannot yet update shape properties without reloading the model into
  // the world
//  SetBoxShapeProperties,
  AttachBoxShapeFeature,
  GetCylinderShapeProperties,
//  SetCylinderShapeProperties,
  AttachCylinderShapeFeature,
  GetSphereShapeProperties,
//  SetSphereShapeProperties,
  AttachSphereShapeFeature,
  mesh::GetMeshShapeProperties,
//  mesh::SetMeshShapeProperties,
  mesh::AttachMeshShapeFeature
>;

class ShapeFeatures :
    public virtual Base,
    public virtual Implements3d<ShapeFeatureList>
{
  // ----- Kinematic Properties -----
  public: Pose3d GetShapeRelativeTransform(
      std::size_t _shapeID) const override;

  public: void SetShapeRelativeTransform(
      std::size_t _shapeID, const Pose3d &_pose) override;


  // ----- Box Features -----
  public: Identity CastToBoxShape(
      std::size_t _shapeID) const override;

  public: LinearVector3d GetBoxShapeSize(
      std::size_t _boxID) const override;

  public: Identity AttachBoxShape(
      std::size_t _linkID,
      const std::string &_name,
      const LinearVector3d &_size,
      const Pose3d &_pose) override;


  // ----- Cylinder Features -----
  public: Identity CastToCylinderShape(
      std::size_t _shapeID) const override;

  public: double GetCylinderShapeRadius(
      std::size_t _cylinderID) const override;

  public: double GetCylinderShapeHeight(
      std::size_t _cylinderID) const override;

  public: Identity AttachCylinderShape(
      std::size_t _linkID,
      const std::string &_name,
      double _radius,
      double _height,
      const Pose3d &_pose) override;


  // ----- Sphere Features -----
  public: Identity CastToSphereShape(
      std::size_t _shapeID) const override;

  public: double GetSphereShapeRadius(
      std::size_t _sphereID) const override;

  public: Identity AttachSphereShape(
      std::size_t _linkID,
      const std::string &_name,
      double _radius,
      const Pose3d &_pose) override;


  // ----- Mesh Features -----
  public: Identity CastToMeshShape(
      std::size_t _shapeID) const override;

  public: LinearVector3d GetMeshShapeSize(
      std::size_t _meshID) const override;

  public: LinearVector3d GetMeshShapeScale(
      std::size_t _meshID) const override;

  public: Identity AttachMeshShape(
      std::size_t _linkID,
      const std::string &_name,
      const ignition::common::Mesh &_mesh,
      const Pose3d &_pose) override;
};

}
}
}

#endif
