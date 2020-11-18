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
  RemoveEntities,
  ConstructEmptyWorldFeature
>;

class EntityManagementFeatures :
    public virtual Base,
    public virtual Implements3d<EntityManagementFeatureList>
{

  // ----- Remove entities -----
  public: bool RemoveModelByIndex(
      const Identity &/* _worldID */, std::size_t _modelIndex) override;

  public: bool RemoveModelByName(
      const Identity &/* _worldID */, const std::string &/* _modelName */) override;

  public: bool RemoveModel(const Identity &_modelID) override;

  public: bool ModelRemoved(const Identity &_modelID) const override;

  // ----- Construct empty entities -----
  public: Identity ConstructEmptyWorld(
      const Identity &/*_engineID*/, const std::string &/* _name */) override;
};

}
}
}

#endif
