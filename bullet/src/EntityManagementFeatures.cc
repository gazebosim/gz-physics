#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <btBulletDynamicsCommon.h>

#include <string>

#include "EntityManagementFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  ignerr << "ConstructEmptyWorld" << std::endl;
  // Create bullet empty multibody dynamics world
  auto collisionConfiguration = new btDefaultCollisionConfiguration();
  auto dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  auto solver = new btMultiBodyConstraintSolver;
  auto world = new btMultiBodyDynamicsWorld(
      dispatcher, broadphase, solver, collisionConfiguration);

  world->getSolverInfo().m_globalCfm = 0;

  return this->AddWorld(
      {world, _name, collisionConfiguration, dispatcher, broadphase, solver});
}

/////////////////////////////////////////////////
bool EntityManagementFeatures::RemoveModel(const Identity &_modelID)
{
  ignerr << "RemoveModel " << _modelID.id << std::endl;
  // Check if the model exists
  if (this->models.find(_modelID) == this->models.end()){
    return false;
  }

  // Current implementation does not include collisions nor joints
  // Those should be removed here before removing the model
  std::unordered_map<std::size_t, LinkInfoPtr>::iterator it = this->links.begin();
  while (it != this->links.end())
  {
    const auto &linkInfo = it->second;
    if (linkInfo->model.id == _modelID.id)
    {
      ignerr << "Erasing links " <<  it->first << std::endl;
      it = this->links.erase(it);
      continue;
    }
    it++;
  }

  // (To-DO blast545): test if its required to make an extra call to
  // world->removeMultiBody(model) to remove it from the bullet system
  // or is it enough just by deleting its pointed memory

  // Clean up model
  ignerr << "Removing BT" << std::endl;
  this->worlds.at(this->models.at(_modelID)->world)->world->removeMultiBody(this->models.at(_modelID)->model);
  ignerr << "deleting pointer" << this->models.at(_modelID)->model << std::endl;
  delete this->models.at(_modelID)->model;   //  todo(Lobotuerk) causes "free(): invalid next size (fast)"
  // Adding and removing objects one by one gets you "corrupted size vs. prev_size"
  // Adding more than 3, deleting 1 and adding 1, gives you "malloc_consolidate(): invalid chunk size"
  ignerr << "erasing from map models" << std::endl;
  this->models.erase(_modelID);

  ignerr << "erased from map models" << std::endl;
  return true;
}

bool EntityManagementFeatures::ModelRemoved(
  const Identity &_modelID) const
{
  ignerr << "ModelRemoved" << std::endl;
  return this->models.find(_modelID) == this->models.end();
}

bool EntityManagementFeatures::RemoveModelByIndex(
  const Identity &/* _worldID */, std::size_t _modelIndex)
{
  ignerr << "RemoveModelByIndex" << std::endl;
  // Check if the model exists
  if (this->models.find(_modelIndex) == this->models.end()){
    return false;
  }

  // Current method ignores the worldID, check if this causes
  // an API problem with the gazebo simulator
  const auto &modelInfo = this->models.at(_modelIndex);

  // Current implementation does not include collisions nor joints
  // Those should be removed here before removing the model

  // Clean up links
  for (const auto &linkEntry : this->links)
  {
    const auto &linkInfo = linkEntry.second;
    if (linkInfo->model.id == _modelIndex)
    {
      this->links.erase(linkEntry.first);
    }
  }

  // (To-DO blast545): test if its required to make an extra call to
  // world->removeMultiBody(model) to remove it from the bullet system
  // or is it enough just by deleting its pointed memory

  // Clean up model
  delete modelInfo->model;
  this->models.erase(_modelIndex);

  return true;
}

bool EntityManagementFeatures::RemoveModelByName(
  const Identity & _worldID, const std::string & _modelName )
{
  ignerr << "RemoveModelByName" << std::endl;

  // Check if there is a model with the requested name
  bool found = false;
  size_t index_id = 0;
  for (const auto &model : this->models)
  {
    const auto &modelInfo = model.second;
    if (modelInfo->name == _modelName)
    {
      found = true;
      index_id = model.first;
      break;
    }
  }

  if (found) {
    // Use a not valid Identity, not used
    return this->RemoveModelByIndex(_worldID, index_id);
  }

  return false;
}

}
}
}
