#include "SDFFeatures.hh"
#include <ignition/math/eigen3/Conversions.hh>

#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Plane.hh>

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
  const math::Pose3d pose = _sdfModel.RawPose();
  const bool isStatic = _sdfModel.Static();
  // const bool selfCollide = _sdfModel.SelfCollide();

  // const auto &world = this->worlds.at(_worldID)->world;
  const auto modelIdentity = this->AddModel({name, _worldID, isStatic, pose});

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
  auto mass = inertial.MassMatrix().Mass();
  const auto diagonalMoments = inertial.MassMatrix().DiagonalMoments();

  // Get link properties
  // const btScalar linkMass = mass;
  btVector3 linkInertiaDiag =
      convertVec(ignition::math::eigen3::convert(diagonalMoments));

  math::Pose3d base_pose = this->models.at(_modelID)->pose;
  const auto poseIsometry = ignition::math::eigen3::convert(base_pose * pose);
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // Create link
  // (TO-DO: do we want to use MotionState?) 2nd part: Do motion state use the same transformation?
  btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
	if (this->models.at(_modelID)->fixed)
  {
    mass = 0;
    linkInertiaDiag = btVector3(0,0,0);
  }
  btDefaultMotionState* myMotionState = new btDefaultMotionState(baseTransform);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, linkInertiaDiag);
  btRigidBody* body = new btRigidBody(rbInfo);
  body->setWorldTransform(baseTransform);

  // Generate an identity for it
  const auto linkIdentity = this->AddLink({name, body, _modelID});
  return linkIdentity;
}

Identity SDFFeatures::ConstructSdfCollision(
      const Identity &_linkID,
      const ::sdf::Collision &_collision)
{
  if (!_collision.Geom())
  {
    ignerr << "The geometry element of collision [" << _collision.Name() << "] "
           << "was a nullptr\n";
    return this->GenerateInvalidId();
  }

  const auto &geom = _collision.Geom();
  btCollisionShape* shape = nullptr;

  if (geom->BoxShape())
  {
    const auto box = geom->BoxShape();
    const auto size = math::eigen3::convert(box->Size());
    const auto halfExtents = convertVec(size)*0.5;
    shape = new btBoxShape(halfExtents);
  }
  else if (geom->SphereShape())
  {
    const auto sphere = geom->SphereShape();
    const auto radius = sphere->Radius();
    shape = new btSphereShape(radius);
  }
  else if (geom->CylinderShape())
  {
    const auto cylinder = geom->CylinderShape();
    const auto radius = cylinder->Radius();
    const auto halfLength = cylinder->Length()*0.5;
    shape = new btCylinderShapeZ(btVector3(radius, 0, halfLength));
  }
  else if (geom->PlaneShape())
  {
    const auto plane = geom->PlaneShape();
    const auto normal = convertVec(math::eigen3::convert(plane->Normal()));
    shape = new btStaticPlaneShape(normal, 0);
  }

  // TODO(lobotuerk) find if there's a  way to use friction and related
  // info in bullet dynamics

  // const auto &surfaceElement = _collision.Element()->GetElement("surface");
  //
  // // Get friction
  // const auto &odeFriction = surfaceElement->GetElement("friction")
  //                             ->GetElement("ode");
  // const auto mu = odeFriction->Get<btScalar>("mu");
  //
  // // Get restitution
  // const auto restitution = surfaceElement->GetElement("bounce")
  //                             ->Get<btScalar>("restitution_coefficient");
  if (shape != nullptr)
  {
    const auto &linkInfo = this->links.at(_linkID);
    const auto &modelID = linkInfo->model;
    const auto &modelInfo = this->models.at(modelID);
    const auto &link = linkInfo->link;
    const auto &world = this->worlds.at(modelInfo->world)->world;

    delete link->getCollisionShape();
    link->setCollisionShape(shape);

    // We add the rigidbody to the world after it has collision, as
    // non collision bodies don't get simulated on a dynamics world
    world->addRigidBody(link);

    return this->AddCollision({_collision.Name(), shape, _linkID,
                               modelID});
  }
  return this->GenerateInvalidId();
}

}
}
}
