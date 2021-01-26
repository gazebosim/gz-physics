#ifndef IGNITION_PHYSICS_BULLET_SRC_LINKFEATURES_HH_
#define IGNITION_PHYSICS_BULLET_SRC_LINKFEATURES_HH_

#include <ignition/physics/Link.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

struct LinkFeatureList : FeatureList<
  AddLinkExternalForceTorque
> { };

class LinkFeatures :
    public virtual Base,
    public virtual Implements3d<LinkFeatureList>
{
  // ----- Add Link Force/Torque -----
  public: void AddLinkExternalForceInWorld(
      const Identity &_id,
      const LinearVectorType &_force,
      const LinearVectorType &_position) override;

  public: void AddLinkExternalTorqueInWorld(
      const Identity &_id, const AngularVectorType &_torque) override;
};

}
}
}

#endif
