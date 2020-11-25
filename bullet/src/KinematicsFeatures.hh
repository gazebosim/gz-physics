#ifndef IGNITION_PHYSICS_BULLET_SRC_KINEMATICSFEATURES_HH_
#define IGNITION_PHYSICS_BULLET_SRC_KINEMATICSFEATURES_HH_

#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/FreeGroup.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

using KinematicsFeatureList = FeatureList<
  LinkFrameSemantics
>;

class KinematicsFeatures :
    public virtual Base,
    public virtual Implements3d<KinematicsFeatureList>
{
  public: FrameData3d FrameDataRelativeToWorld(const FrameID &_id) const;
};

}
}
}

#endif
