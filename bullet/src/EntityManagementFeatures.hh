#ifndef IGNITION_PHYSICS_BULLET_SRC_GETENTITIESFEATURE_HH_
#define IGNITION_PHYSICS_BULLET_SRC_GETENTITIESFEATURE_HH_

#include <string>

#include <ignition/physics/ConstructEmpty.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/RemoveEntities.hh>
#include <ignition/physics/Implements.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

using EntityManagementFeatureList = FeatureList<
  ConstructEmptyWorldFeature
>;

class EntityManagementFeatures :
    public virtual Base,
    public virtual Implements3d<EntityManagementFeatureList>
{
  // ----- Construct empty entities -----
  public: Identity ConstructEmptyWorld(
      const Identity &/*_engineID*/, const std::string &/* _name */) override;
};

}
}
}

#endif
