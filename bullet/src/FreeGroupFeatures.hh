#ifndef IGNITION_PHYSICS_BULLET_SRC_FREEGROUPFEATURES_HH_
#define IGNITION_PHYSICS_BULLET_SRC_FREEGROUPFEATURES_HH_

#include <ignition/physics/FreeGroup.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

using FreeGroupFeatureList = FeatureList<
  FindFreeGroupFeature,
  SetFreeGroupWorldPose
>;

class FreeGroupFeatures
    : public virtual Base,
      public virtual Implements3d<FreeGroupFeatureList>
{
  // ----- FindFreeGroupFeature -----
  Identity FindFreeGroupForModel(const Identity &_modelID) const override;

  Identity FindFreeGroupForLink(const Identity &/* _linkID */) const override
      { return this->GenerateInvalidId(); };

  Identity GetFreeGroupCanonicalLink(
      const Identity &/* _groupID */) const override
      { return this->GenerateInvalidId(); };

  // ----- SetFreeGroupWorldPose -----
  void SetFreeGroupWorldPose(
      const Identity &_groupID,
      const PoseType &_pose) override;
};

}
}
}

#endif
