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

#ifndef IGNITION_PHYSICS_BULLET_SRC_SHAPEFEATURES_HH_
#define IGNITION_PHYSICS_BULLET_SRC_SHAPEFEATURES_HH_

#include <ignition/physics/mesh/MeshShape.hh>
#include <string>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

using ShapeFeatureList = FeatureList<
  mesh::AttachMeshShapeFeature
>;

class ShapeFeatures :
    public virtual Base,
    public virtual Implements3d<ShapeFeatureList>
{
  public: Identity AttachMeshShape(
      const Identity &_linkID,
      const std::string &_name,
      const ignition::common::Mesh &_mesh,
      const Pose3d &_pose,
      const LinearVector3d &_scale) override;

  public: Identity CastToMeshShape(
      const Identity &_shapeID) const override;
};

}  // namespace bullet
}  // namespace physics
}  // namespace ignition

#endif
