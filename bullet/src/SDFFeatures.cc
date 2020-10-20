#include "SDFFeatures.hh"
#include <ignition/math/eigen3/Conversions.hh>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
inline btMatrix3x3 convertMat(Eigen::Matrix3d mat)
{
  return btMatrix3x3(mat(0, 0), mat(0, 1), mat(0, 2),
                     mat(1, 0), mat(1, 1), mat(1, 2),
                     mat(2, 0), mat(2, 1), mat(2, 2));
}

/////////////////////////////////////////////////
inline btVector3 convertVec(Eigen::Vector3d vec)
{
  return btVector3(vec(0), vec(1), vec(2));
}

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
  const bool isStatic = _sdfModel.Static();
  const bool selfCollide = _sdfModel.SelfCollide();

  // Set multibody params
  const btScalar baseMass = 0;
  // TODO(lobotuerk) update mass calculation to something like
  // https://github.com/bulletphysics/bullet3/blob/master/examples/MultiBody/MultiDofDemo.cpp#L289
  const btVector3 baseInertiaDiag(0, 0, 0); // This has to be tested, Im not sure if 0 0 0 is normal
  // To Do: experiment with it
  const bool canSleep = false;

  // Set base transform
  const auto poseIsometry = ignition::math::eigen3::convert(pose);
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // Check if floating base
  bool fixedBase = isStatic;
  const bool hasWorldLink = _sdfModel.LinkNameExists("world");
  if (!hasWorldLink)
  {
    fixedBase = true;
  }

  // Initialize model with zero links and increase the number as links are added
  const int numLinks = 0;

  // Build model
  btMultiBody* model = new btMultiBody(numLinks,
                                       baseMass,
                                       baseInertiaDiag,
                                       fixedBase,
                                       canSleep);

  model->setBaseWorldTransform(baseTransform);

  model->setHasSelfCollision(selfCollide);

  const auto &world = this->worlds.at(_worldID)->world;
  world->addMultiBody(model);
  const auto modelIdentity = this->AddModel({model, name, _worldID});

  // Build links
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i)
  {
    this->ConstructSdfLink(modelIdentity, *_sdfModel.LinkByIndex(i));
  }

  // Finalize model
  model->finalizeMultiDof();

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
  const btScalar linkMass = mass;
  const btVector3 linkInertiaDiag =
      convertVec(ignition::math::eigen3::convert(diagonalMoments));
  const auto poseIsometry = ignition::math::eigen3::convert(pose);

  // Add default fixed joints to links unless replaced by other joints
  // Find translation
  const btVector3 parentComToCurrentCom = convertVec(
    poseIsometry.translation());
  const btVector3 currentPivotToCurrentCom(0, 0, 0);
  const btVector3 parentComToCurrentPivot = parentComToCurrentCom -
    currentPivotToCurrentCom;
  // Find rotation
  btQuaternion rotParentToThis;
  const btMatrix3x3 mat = convertMat(poseIsometry.linear());
  mat.getRotation(rotParentToThis);

  // Set up fixed joints
  const int parentIndex = -1;
  const auto &model = this->models.at(_modelID)->model;

  // Update number of links in the model
  auto linkIndex = model->getNumLinks();
  model->setNumLinks(linkIndex + 1);

  // Create link
  model->setupFixed(linkIndex, linkMass, linkInertiaDiag, parentIndex,
                     rotParentToThis, parentComToCurrentPivot,
                     currentPivotToCurrentCom);

  const auto linkIdentity = this->AddLink({name, linkIndex, linkMass,
                                  linkInertiaDiag, poseIsometry, _modelID});

  return linkIdentity;
}



}
}
}
