
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

}
}
}

#endif
