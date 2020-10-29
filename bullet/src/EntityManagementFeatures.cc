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
  // Check if the model exists
  if (this->models.find(_modelID.id) == this->models.end()){
    return false;
  }
  // Current implementation does not include collisions nor joints
  // Those should be removed here before removing the model

  // Clean up links
  std::unordered_map<std::size_t, LinkInfoPtr>::iterator it = this->links.begin();
  while (it != this->links.end())
  {
    const auto &linkInfo = it->second;
    if (linkInfo->model.id == _modelID.id)
    {
      it = this->links.erase(it);
      continue;
    }
    it++;
  }

  // Clean up model
  auto model = this->models.at(_modelID.id);
  this->worlds.at(model->world)->world->removeMultiBody(model->model);
  delete this->models.at(_modelID.id)->model;
  this->models.erase(_modelID.id);

  return true;
}

bool EntityManagementFeatures::ModelRemoved(
  const Identity &_modelID) const
{
  return this->models.find(_modelID) == this->models.end();
}

bool EntityManagementFeatures::RemoveModelByIndex(
  const Identity & _worldID, std::size_t _modelIndex)
{
  // Check if the model exists
  if (this->models.find(_modelIndex) == this->models.end() ||
      this->models.at(_modelIndex)->world.id != _worldID.id) {
    return false;
  }
  // Current implementation does not include collisions nor joints
  // Those should be removed here before removing the model

  // Clean up links
  std::unordered_map<std::size_t, LinkInfoPtr>::iterator it = this->links.begin();
  while (it != this->links.end())
  {
    const auto &linkInfo = it->second;
    if (linkInfo->model.id == _modelIndex)
    {
      it = this->links.erase(it);
      continue;
    }
    it++;
  }

  // Clean up model
  auto model = this->models.at(_modelIndex);
  this->worlds.at(model->world)->world->removeMultiBody(model->model);
  delete this->models.at(_modelIndex)->model;
  this->models.erase(_modelIndex);

  return true;
}

bool EntityManagementFeatures::RemoveModelByName(
  const Identity & _worldID, const std::string & _modelName )
{

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
