#include "SDFFeatures.hh"
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/math/Helpers.hh>

#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Plane.hh>
#include <sdf/JointAxis.hh>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
// static Eigen::Isometry3d ResolveSdfPose(const ::sdf::SemanticPose &_semPose)
// {
//   math::Pose3d pose;
//   ::sdf::Errors errors = _semPose.Resolve(pose);
//   if (!errors.empty())
//   {
//     if (!_semPose.RelativeTo().empty())
//     {
//       ignerr << "There was an error in SemanticPose::Resolve\n";
//       for (const auto &err : errors)
//       {
//         ignerr << err.Message() << std::endl;
//       }
//       ignerr << "There is no optimal fallback since the relative_to attribute["
//              << _semPose.RelativeTo() << "] of the pose is not empty. "
//              << "Falling back to using the raw Pose.\n";
//     }
//     pose = _semPose.RawPose();
//   }
//
//   return math::eigen3::convert(pose);
// }

/////////////////////////////////////////////////
// This function was taken directly from dartsim
// Might need geometry fixes
// static Eigen::Vector3d ConvertJointAxis(
//     const ::sdf::JointAxis *_sdfAxis,
//     const ModelInfo &_modelInfo,
//     const Eigen::Isometry3d &_T_joint)
// {
//   (void) _modelInfo;
//   (void) _T_joint;
//   math::Vector3d resolvedAxis;
//   ::sdf::Errors errors = _sdfAxis->ResolveXyz(resolvedAxis);
//   if (errors.empty())
//     return math::eigen3::convert(resolvedAxis);
//
//   // Error while Resolving xyz. Fallback sdformat 1.6 behavior but treat
//   // xyz_expressed_in = "__model__" as the old use_parent_model_frame
//
//   const Eigen::Vector3d axis = ignition::math::eigen3::convert(_sdfAxis->Xyz());
//   return axis;
//   /*
//
//   if (_sdfAxis->XyzExpressedIn().empty())
//     return axis;
//
//   if (_sdfAxis->XyzExpressedIn() == "__model__")
//   {
//     ignwarn << "Xyz expressed in model frame is not currently supported, returning raw axis\n";
//     return axis;
//   }
//
//   // xyz expressed in a frame other than the joint frame or the parent model
//   // frame is not supported
//   ignerr << "There was an error in JointAxis::ResolveXyz\n";
//   for (const auto &err : errors)
//   {
//     ignerr << err.Message() << std::endl;
//   }
//   ignerr << "There is no optimal fallback since the expressed_in attribute["
// 	 << _sdfAxis->XyzExpressedIn() << "] of the axis's xyz is neither empty"
// 	 << "nor '__model__'. Falling back to using the raw xyz vector "
// 	 << "expressed in the joint frame.\n";
//
//   return axis;
//   */
// }

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

  // After creating all the links, join the ones that have joints
  for (std::size_t i=0; i < _sdfModel.JointCount(); ++i)
  {
    igndbg << "Loop adding joints.\n";

    const ::sdf::Joint *sdfJoint = _sdfModel.JointByIndex(i);
    if (!sdfJoint)
    {
      ignerr << "The joint with index [" << i << "] in model ["
             << _sdfModel.Name() << "] is a nullptr. It will be skipped.\n";
      continue;
    }

    // Find the child and parent rigid bodies
    // dart::dynamics::BodyNode * const parent = this->FindOrConstructLink(
    //       model, modelIdentity, _sdfModel, sdfJoint->ParentLinkName());

    // dart::dynamics::BodyNode * const child = this->FindOrConstructLink(
    //       model, modelIdentity, _sdfModel, sdfJoint->ChildLinkName());

    // this->ConstructSdfJoint(modelInfo, *sdfJoint, parent, child);
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
  auto mass = inertial.MassMatrix().Mass();
  const auto diagonalMoments = inertial.MassMatrix().DiagonalMoments();

  // Get link properties
  // const btScalar linkMass = mass;
  btVector3 linkInertiaDiag =
      convertVec(ignition::math::eigen3::convert(diagonalMoments));

  // Create link
  // (TO-DO: do we want to use MotionState?) 2nd part: Do motion state use the same transformation?
  if (this->models.at(_modelID)->fixed)
  {
    mass = 0;
    linkInertiaDiag = btVector3(0,0,0);
  }

  // Generate an identity for it
  const auto linkIdentity = this->AddLink({name, body, _modelID, pose});
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

  const auto &surfaceElement = _collision.Element()->GetElement("surface");

  // Get friction
  const auto &odeFriction = surfaceElement->GetElement("friction")
                              ->GetElement("ode");
  const auto mu = odeFriction->Get<btScalar>("mu");

  // Get restitution
  // const auto restitution = surfaceElement->GetElement("bounce")
  //                             ->Get<btScalar>("restitution_coefficient");
  if (shape != nullptr)
  {
    const auto &linkInfo = this->links.at(_linkID);
    const auto &modelID = linkInfo->model;
    const auto &modelInfo = this->models.at(modelID);
    const auto &world = this->worlds.at(modelInfo->world)->world;
    const auto &mass = linkInfo->mass;
    const auto &inertia = linkInfo->inertia;

    if (!modelInfo->fixed && this->link_to_collision.find(_linkID.id) != this->link_to_collision.end()){
      return this->GenerateInvalidId();
    }

    delete link->getCollisionShape();
    shape->setMargin(btScalar(0.001));
    link->setCollisionShape(shape);
    link->setFriction(mu);

    // We add the rigidbody to the world after it has collision, as
    // non collision bodies don't get simulated on a dynamics world
    world->addRigidBody(body);

    auto identity = this->AddCollision({_collision.Name(), shape, _linkID,
                               modelID, pose});

    this->link_to_collision[_linkID] = identity;
    return identity;
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(
  const Identity &_modelID,
  const ::sdf::Joint &_sdfJoint)
{
  // const auto &parentModelInfo = *this->ReferenceInterface<ModelInfo>(_modelID);

  // Check supported Joints
  const ::sdf::JointType type = _sdfJoint.Type();
  if( type != ::sdf::JointType::REVOLUTE && type != ::sdf::JointType::FIXED ){
    ignerr << "Asked to construct a joint of sdf::JointType ["
           << static_cast<int>(type) << "], but that is not supported yet.\n";
    return this->GenerateInvalidId();
  }

  // Get the parent and child ids
  const std::string parentLinkName = _sdfJoint.ParentLinkName();
  std::size_t parentId = this->FindSdfLink(_modelID, parentLinkName);

  const std::string childLinkName = _sdfJoint.ChildLinkName();
  std::size_t childId = this->FindSdfLink(_modelID, childLinkName);

  // Check if chilId and parentId are valid values
  const auto invalidEntity = this->GenerateInvalidId().id;
  if (parentId == invalidEntity || childId == invalidEntity) {
    ignerr << "There was a problem finding/creating parent/child links\n";
    return this->GenerateInvalidId();
  }

  // Handle the case where either parent or child is the world
  const std::size_t worldId = this->models.at(_modelID)->world;
  if (parentId == worldId || childId == worldId){
    ignwarn << "Not implemented joints using world as parent/child\n";
    return this->GenerateInvalidId();
  }

  // Get axis unit vector (expressed in world frame).
  // IF fixed joint, use UnitZ, if revolute use the Axis given by the joint
  // Eigen::Vector3d axis;
  //ignition::math::Vector3d axis = ignition::math::Vector3d::UnitZ;
  ignition::math::Vector3d axis;
  if(type == ::sdf::JointType::FIXED ){
    axis = ignition::math::Vector3d::UnitZ;
  }
  else {
    axis = _sdfJoint.Axis(0)->Xyz();
  }

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  Eigen::Vector3d pivotParent, pivotChild, axisParent, axisChild;
  math::Pose3d pose;
  const math::Pose3d base_pose = this->models.at(_modelID)->pose;
  // const Eigen::Isometry3d T_joint = ignition::math::eigen3::convert(base_pose) * ResolveSdfPose(_sdfJoint.SemanticPose());

  // Initialize pivots to anchorPos, which is expressed in the
  // world coordinate frame.
  // anchorPos is the position of the joint in gazebo11, replacing with equivalent for ignition
  pivotParent = ignition::math::eigen3::convert(_sdfJoint.RawPose().Pos() + this->links.at(childId)->pose.Pos());
  pivotChild = ignition::math::eigen3::convert(_sdfJoint.RawPose().Pos() + this->links.at(childId)->pose.Pos());
  // pivotParent = ConvertJointAxis(_sdfJoint.Axis(0), parentModelInfo, T_joint);
  // pivotChild = ConvertJointAxis(_sdfJoint.Axis(0), parentModelInfo, T_joint);

  // Assumming at this part of the code already that both parent and child are defined
  // And none of those is world

  // <<Consider: I think the pose has to be referenced to the base pose of the model>>
  // const auto poseIsometry = ignition::math::eigen3::convert(base_pose * pose);
  // The following part of the code was taken from gazebo11 implementation almost without changes

  // Compute relative pose between joint anchor and inertial frame of parent.
  pose = this->links.at(parentId)->pose;
  // Subtract CoG position from anchor position, both in world frame.
  pivotParent -= ignition::math::eigen3::convert(pose).translation();

  // Rotate pivot offset and axis into body-fixed inertial frame of parent.
  math::Vector3 pivotParent_vect(pivotParent(0), pivotParent(1), pivotParent(2));
  pivotParent_vect = pose.Rot().RotateVectorReverse(pivotParent_vect);
  math::Vector3 axisParent_vect = pose.Rot().RotateVectorReverse(axis);
  axisParent_vect = axisParent_vect.Normalize();

  // Compute relative pose between joint anchor and inertial frame of child.
  pose = this->links.at(childId)->pose;
  // Subtract CoG position from anchor position, both in world frame.
  pivotChild -= ignition::math::eigen3::convert(pose).translation();
  // Rotate pivot offset and axis into body-fixed inertial frame of child.

  math::Vector3 pivotChild_vect(pivotChild(0), pivotChild(1), pivotChild(2));
  pivotChild_vect = pose.Rot().RotateVectorReverse(pivotChild_vect);
  math::Vector3 axisChild_vect = pose.Rot().RotateVectorReverse(axis);
  axisChild_vect = axisChild_vect.Normalize();

  // At this part, save a reference to the object created, to delete it later
  // If both links exist, then create a joint between the two links.
  pivotParent(0) = pivotParent_vect[0];
  pivotParent(1) = pivotParent_vect[1];
  pivotParent(2) = pivotParent_vect[2];
  axisParent(0) = axisParent_vect[0];
  axisParent(1) = axisParent_vect[1];
  axisParent(2) = axisParent_vect[2];
  pivotChild(0) = pivotChild_vect[0];
  pivotChild(1) = pivotChild_vect[1];
  pivotChild(2) = pivotChild_vect[2];
  axisChild(0) = axisChild_vect[0];
  axisChild(1) = axisChild_vect[1];
  axisChild(2) = axisChild_vect[2];

  btTypedConstraint* joint = new btHingeConstraint(
    *this->links.at(childId)->link,
    *this->links.at(parentId)->link,
    convertVec(pivotChild),
    convertVec(pivotParent),
    convertVec(axisChild),
    convertVec(axisParent),
    true);

  const auto &modelInfo = this->models.at(_modelID);
  const auto &world = this->worlds.at(modelInfo->world)->world;
  world->addConstraint(joint, true);
  joint->enableFeedback(true);

  // Generate an identity for it and return it
  return this->AddJoint({_sdfJoint.Name(), joint, childId, parentId});
}

/////////////////////////////////////////////////
std::size_t SDFFeatures::FindSdfLink(
  const Identity &_modelID,
  const std::string &_sdfLinkName)
{
  for (const auto &link : this->links)
  {
    const auto &linkInfo = link.second;
    if (linkInfo->name == _sdfLinkName)
    {
      // A link was previously created with that name,
      // Return its entity value
      return link.first;
    }
  }

  // Link wasn't found, check if the requested link is "world"
  if (_sdfLinkName == "world") {
    // Return the ID of the parent world of the model
    return this->models.at(_modelID)->world;
  }
  else{
    ignerr << "Model does not contain a link named [" << _sdfLinkName << "].\n";
    return this->GenerateInvalidId();
  }
}

}
}
}
