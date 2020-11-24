#include "SDFFeatures.hh"
#include <ignition/math/eigen3/Conversions.hh>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const Identity &_engine,
    const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  const WorldInfoPtr &worldInfo = this->worlds.at(worldID);

  auto gravity = _sdfWorld.Gravity();
  worldInfo->world->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
  return worldID;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
  const Identity &_worldID,
  const ::sdf::Model &_sdfModel)
{
  // Read sdf params
  const std::string name = _sdfModel.Name();
  const auto pose = _sdfModel.RawPose();
  // const bool isStatic = _sdfModel.Static();
  // const bool selfCollide = _sdfModel.SelfCollide();

  // const auto &world = this->worlds.at(_worldID)->world;
  const auto modelIdentity = this->AddModel({name, _worldID});

  // Build links
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i)
  {
    this->ConstructSdfLink(modelIdentity, *_sdfModel.LinkByIndex(i));
  }

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfLink(
  const Identity &_modelID,
  const ::sdf::Link &_sdfLink)
{
  // Read sdf params
  const std::string name = _sdfLink.Name();
  const auto pose = _sdfLink.RawPose();
  const auto inertial = _sdfLink.Inertial();
  const auto mass = inertial.MassMatrix().Mass();
  const auto diagonalMoments = inertial.MassMatrix().DiagonalMoments();

  // Get link properties
  // const btScalar linkMass = mass;
  const btVector3 linkInertiaDiag =
      convertVec(ignition::math::eigen3::convert(diagonalMoments));

  // (TO-DO: do those calculation need to take into account in some way the base model?)
  const auto poseIsometry = ignition::math::eigen3::convert(pose);
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));
  
  // Create link
  // (TO-DO: do we want to use MotionState?)
  // First zero is motionState, the second one is the colision shape
  btRigidBody* body = new btRigidBody(mass, 0, 0, linkInertiaDiag);
  body->setWorldTransform(baseTransform);

  // Add it to the internal world:
  const auto _worldID = this->models.at(_modelID)->world;
  const auto &world = this->worlds.at(_worldID)->world;  
  world->addRigidBody(body);

  // Generate an identity for it
  const auto linkIdentity = this->AddLink({name, body, _modelID});

  return linkIdentity;
}

}
}
}
