
#ifndef IGNITION_PHYSICS_BULLET_SRC_SDFFEATURES_HH_
#define IGNITION_PHYSICS_BULLET_SRC_SDFFEATURES_HH_

#include <ignition/physics/sdf/ConstructJoint.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>
#include <ignition/physics/sdf/ConstructCollision.hh>

#include <ignition/physics/Implements.hh>

#include "EntityManagementFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

using SDFFeatureList = FeatureList<
  sdf::ConstructSdfJoint,
  sdf::ConstructSdfLink,
  sdf::ConstructSdfModel,
  sdf::ConstructSdfCollision,
  sdf::ConstructSdfWorld
>;

class SDFFeatures :
    public virtual EntityManagementFeatures,
    public virtual Implements3d<SDFFeatureList>
{
  public: Identity ConstructSdfWorld(
      const Identity &/*_engine*/,
      const ::sdf::World &_sdfWorld) override;

  public: Identity ConstructSdfModel(
      const Identity &_worldID,
      const ::sdf::Model &_sdfModel) override;

  private: Identity ConstructSdfLink(
      const Identity &_modelID,
      const ::sdf::Link &_sdfLink) override;

  private: Identity ConstructSdfCollision(
      const Identity &_linkID,
      const ::sdf::Collision &_collision) override;

  private: Identity ConstructSdfJoint(
      const Identity &_modelID,
      const ::sdf::Joint &_sdfJoint) override;

  private: std::size_t FindOrConstructSdfLink(
      const Identity &_modelID,
      const ::sdf::Link &_sdfLink);
};

}
}
}

#endif
