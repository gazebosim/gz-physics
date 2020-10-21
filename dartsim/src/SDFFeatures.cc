/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "SDFFeatures.hh"

#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ScrewJoint.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/UniversalJoint.hpp>
#include <dart/constraint/WeldJointConstraint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <cmath>

#include <ignition/common/Console.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/math/Helpers.hh>

#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Sphere.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

namespace ignition {
namespace physics {
namespace dartsim {

namespace {
/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
static Eigen::Isometry3d ResolveSdfPose(const ::sdf::SemanticPose &_semPose)
{
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  if (!errors.empty())
  {
    if (!_semPose.RelativeTo().empty())
    {
      ignerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto &err : errors)
      {
        ignerr << err.Message() << std::endl;
      }
      ignerr << "There is no optimal fallback since the relative_to attribute["
             << _semPose.RelativeTo() << "] of the pose is not empty. "
             << "Falling back to using the raw Pose.\n";
    }
    pose = _semPose.RawPose();
  }

  return math::eigen3::convert(pose);
}

/////////////////////////////////////////////////
double infIfNeg(const double _value)
{
  if (_value < 0.0)
    return std::numeric_limits<double>::infinity();

  return _value;
}

/// \brief Invert thread pitch to match the different definitions of
/// thread pitch in Gazebo and DART.
///
/// [Definitions of thread pitch]
/// Gazebo: NEGATIVE angular motion per linear motion.
/// DART  : linear motion per single rotation.
static double InvertThreadPitch(double _pitch)
{
  if (math::equal(std::abs(_pitch), 0.0))
  {
    ignerr << "Zero thread pitch is not allowed.\n";
    assert(false);
  }

  return -2.0 * IGN_PI / _pitch;
}

/////////////////////////////////////////////////
template <typename Properties>
static void CopyStandardJointAxisProperties(
    const int _index, Properties &_properties,
    const ::sdf::JointAxis *_sdfAxis)
{
  _properties.mInitialPositions[_index] = _sdfAxis->InitialPosition();
  _properties.mDampingCoefficients[_index] = _sdfAxis->Damping();
  _properties.mFrictions[_index] = _sdfAxis->Friction();
  _properties.mRestPositions[_index] = _sdfAxis->SpringReference();
  _properties.mSpringStiffnesses[_index] = _sdfAxis->SpringStiffness();
  _properties.mPositionLowerLimits[_index] = _sdfAxis->Lower();
  _properties.mPositionUpperLimits[_index] = _sdfAxis->Upper();
  _properties.mIsPositionLimitEnforced = true;
  _properties.mForceLowerLimits[_index] = -infIfNeg(_sdfAxis->Effort());
  _properties.mForceUpperLimits[_index] =  infIfNeg(_sdfAxis->Effort());
  _properties.mVelocityLowerLimits[_index] = -infIfNeg(_sdfAxis->MaxVelocity());
  _properties.mVelocityUpperLimits[_index] =  infIfNeg(_sdfAxis->MaxVelocity());

  // TODO(MXG): Can dartsim support "Stiffness" and "Dissipation"?
}

/////////////////////////////////////////////////
static Eigen::Isometry3d GetParentModelFrame(
    const ModelInfo &_modelInfo)
{
  return _modelInfo.frame->getWorldTransform();
}

/////////////////////////////////////////////////
static Eigen::Vector3d ConvertJointAxis(
    const ::sdf::JointAxis *_sdfAxis,
    const ModelInfo &_modelInfo,
    const Eigen::Isometry3d &_T_joint)
{
  math::Vector3d resolvedAxis;
  ::sdf::Errors errors = _sdfAxis->ResolveXyz(resolvedAxis);
  if (errors.empty())
    return math::eigen3::convert(resolvedAxis);

  // Error while Resolving xyz. Fallback sdformat 1.6 behavior but treat
  // xyz_expressed_in = "__model__" as the old use_parent_model_frame

  const Eigen::Vector3d axis = ignition::math::eigen3::convert(_sdfAxis->Xyz());

  if (_sdfAxis->XyzExpressedIn().empty())
    return axis;

  if (_sdfAxis->XyzExpressedIn() == "__model__")
  {
    const Eigen::Quaterniond O_R_J{_T_joint.rotation()};
    const Eigen::Quaterniond O_R_M{GetParentModelFrame(_modelInfo).rotation()};
    const Eigen::Quaterniond J_R_M = O_R_J.inverse() * O_R_M;
    return J_R_M * axis;
  }

  // xyz expressed in a frame other than the joint frame or the parent model
  // frame is not supported
  ignerr << "There was an error in JointAxis::ResolveXyz\n";
  for (const auto &err : errors)
  {
    ignerr << err.Message() << std::endl;
  }
  ignerr << "There is no optimal fallback since the expressed_in attribute["
         << _sdfAxis->XyzExpressedIn() << "] of the axis's xyz is neither empty"
         << "nor '__model__'. Falling back to using the raw xyz vector "
         << "expressed in the joint frame.\n";

  return axis;
}

/////////////////////////////////////////////////
template <typename JointType>
static JointType *ConstructSingleAxisJoint(
    const ModelInfo &_modelInfo,
    const ::sdf::Joint &_sdfJoint,
    dart::dynamics::BodyNode * const _parent,
    dart::dynamics::BodyNode * const _child,
    const Eigen::Isometry3d &_T_joint)
{
  typename JointType::Properties properties;

  const ::sdf::JointAxis * const sdfAxis = _sdfJoint.Axis(0);

  // use the default properties if sdfAxis is not set
  if (sdfAxis)
  {
    properties.mAxis = ConvertJointAxis(sdfAxis, _modelInfo, _T_joint);
    CopyStandardJointAxisProperties(0, properties, sdfAxis);
  }

  return _child->moveTo<JointType>(_parent, properties);
}

/////////////////////////////////////////////////
static dart::dynamics::UniversalJoint *ConstructUniversalJoint(
    const ModelInfo &_modelInfo,
    const ::sdf::Joint &_sdfJoint,
    dart::dynamics::BodyNode * const _parent,
    dart::dynamics::BodyNode * const _child,
    const Eigen::Isometry3d &_T_joint)
{
  dart::dynamics::UniversalJoint::Properties properties;

  for (const std::size_t index : {0u, 1u})
  {
    const ::sdf::JointAxis * const sdfAxis = _sdfJoint.Axis(index);
    // use the default properties if sdfAxis is not set
    if (sdfAxis)
    {
      properties.mAxis[index] = ConvertJointAxis(sdfAxis, _modelInfo, _T_joint);
      CopyStandardJointAxisProperties(index, properties, sdfAxis);
    }
  }

  return _child->moveTo<dart::dynamics::UniversalJoint>(_parent, properties);
}

/////////////////////////////////////////////////
struct ShapeAndTransform
{
  dart::dynamics::ShapePtr shape;
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
};

/////////////////////////////////////////////////
static ShapeAndTransform ConstructBox(
    const ::sdf::Box &_box)
{
  return {std::make_shared<dart::dynamics::BoxShape>(
        math::eigen3::convert(_box.Size()))};
}

/////////////////////////////////////////////////
static ShapeAndTransform ConstructCylinder(
    const ::sdf::Cylinder &_cylinder)
{
  return {std::make_shared<dart::dynamics::CylinderShape>(
        _cylinder.Radius(), _cylinder.Length())};
}

/////////////////////////////////////////////////
static ShapeAndTransform ConstructSphere(
    const ::sdf::Sphere &_sphere)
{
  return {std::make_shared<dart::dynamics::SphereShape>(_sphere.Radius())};
}

/////////////////////////////////////////////////
static ShapeAndTransform ConstructPlane(
    const ::sdf::Plane &_plane)
{
  // TODO(anyone): We use BoxShape until PlaneShape is completely supported in
  // DART. Please see: https://github.com/dartsim/dart/issues/114
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d axis = z.cross(math::eigen3::convert(_plane.Normal()));
  const double norm = axis.norm();
  const double angle = std::asin(norm/(_plane.Normal().Length()));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

  // We check that the angle isn't too close to zero, because otherwise
  // axis/norm would be undefined.
  if (angle > 1e-12)
    tf.rotate(Eigen::AngleAxisd(angle, axis/norm));

  // This number was taken from osrf/gazebo. Seems arbitrary.
  const double planeDim = 2100;
  tf.translate(Eigen::Vector3d(0.0, 0.0, -planeDim*0.5));

  return {std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(planeDim, planeDim, planeDim)), tf};
}

/////////////////////////////////////////////////
static ShapeAndTransform ConstructMesh(
    const ::sdf::Mesh & /*_mesh*/)
{
  // TODO(MXG): Look into what kind of mesh URI we get here. Will it just be
  // a local file name, or do we need to resolve the URI?
  ignerr << "Mesh construction from an SDF has not been implemented yet for "
         << "dartsim.\n";
  return {nullptr};
}

/////////////////////////////////////////////////
static ShapeAndTransform ConstructGeometry(
    const ::sdf::Geometry &_geometry)
{
  if (_geometry.BoxShape())
    return ConstructBox(*_geometry.BoxShape());
  else if (_geometry.CylinderShape())
    return ConstructCylinder(*_geometry.CylinderShape());
  else if (_geometry.SphereShape())
    return ConstructSphere(*_geometry.SphereShape());
  else if (_geometry.PlaneShape())
    return ConstructPlane(*_geometry.PlaneShape());
  else if (_geometry.MeshShape())
    return ConstructMesh(*_geometry.MeshShape());

  return {nullptr};
}
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const Identity &_engine,
    const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  const dart::simulation::WorldPtr &world = this->worlds.at(worldID);

  world->setGravity(ignition::math::eigen3::convert(_sdfWorld.Gravity()));

  // TODO(MXG): Add a Physics class to the SDFormat DOM and then parse that
  // information here. For now, we'll just use dartsim's default physics
  // parameters.

  for (std::size_t i=0; i < _sdfWorld.ModelCount(); ++i)
  {
    const ::sdf::Model *model = _sdfWorld.ModelByIndex(i);

    if (!model)
      continue;

    this->ConstructSdfModel(worldID, *model);
  }

  return worldID;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
    const Identity &_worldID,
    const ::sdf::Model &_sdfModel)
{
  // check if parent is a world
  if (!this->worlds.HasEntity(_worldID))
  {
    ignerr << "Unable to construct model: " << _sdfModel.Name() << ". "
           << "Parent of model is not a world. " << std::endl;
    return this->GenerateInvalidId();
  }

  dart::dynamics::SkeletonPtr model =
      dart::dynamics::Skeleton::create(_sdfModel.Name());

  dart::dynamics::SimpleFramePtr modelFrame =
      dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        _sdfModel.Name()+"_frame",
        ResolveSdfPose(_sdfModel.SemanticPose()));

  // Set canonical link name
  auto [modelID, modelInfo] = this->AddModel( // NOLINT
      {model, modelFrame, _sdfModel.CanonicalLinkName()}, _worldID);

  model->setMobile(!_sdfModel.Static());
  model->setSelfCollisionCheck(_sdfModel.SelfCollide());

  auto modelIdentity =
      this->GenerateIdentity(modelID, this->models.at(modelID));

  // First, construct all links
  for (std::size_t i=0; i < _sdfModel.LinkCount(); ++i)
  {
    this->FindOrConstructLink(
          model, modelIdentity, _sdfModel, _sdfModel.LinkByIndex(i)->Name());
  }


  // Next, join all links that have joints
  for (std::size_t i=0; i < _sdfModel.JointCount(); ++i)
  {
    const ::sdf::Joint *sdfJoint = _sdfModel.JointByIndex(i);
    if (!sdfJoint)
    {
      ignerr << "The joint with index [" << i << "] in model ["
             << _sdfModel.Name() << "] is a nullptr. It will be skipped.\n";
      continue;
    }

    dart::dynamics::BodyNode * const parent = this->FindOrConstructLink(
          model, modelIdentity, _sdfModel, sdfJoint->ParentLinkName());

    dart::dynamics::BodyNode * const child = this->FindOrConstructLink(
          model, modelIdentity, _sdfModel, sdfJoint->ChildLinkName());

    this->ConstructSdfJoint(modelInfo, *sdfJoint, parent, child);
  }

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfLink(
    const Identity &_modelID,
    const ::sdf::Link &_sdfLink)
{
  const auto &modelInfo = *this->ReferenceInterface<ModelInfo>(_modelID);
  dart::dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = _sdfLink.Name();

  const ignition::math::Inertiald &sdfInertia = _sdfLink.Inertial();
  bodyProperties.mInertia.setMass(sdfInertia.MassMatrix().Mass());

  // TODO(addisu) Resolve the pose of inertials when frame information is
  // availabile for ignition::math::Inertial
  const Eigen::Matrix3d R_inertial{
        math::eigen3::convert(sdfInertia.Pose().Rot())};

  const Eigen::Matrix3d I_link =
      R_inertial
      * math::eigen3::convert(sdfInertia.Moi())
      * R_inertial.inverse();

  bodyProperties.mInertia.setMoment(I_link);

  const Eigen::Vector3d localCom =
      math::eigen3::convert(sdfInertia.Pose().Pos());

  bodyProperties.mInertia.setLocalCOM(localCom);

  dart::dynamics::FreeJoint::Properties jointProperties;
  jointProperties.mName = bodyProperties.mName + "_FreeJoint";
  // TODO(MXG): Consider adding a UUID to this joint name in order to avoid any
  // potential (albeit unlikely) name collisions.

  // Note: When constructing a link from this function, we always instantiate
  // it as a standalone free body within the model. If it should have any joint
  // constraints, those will be added later.
  const auto result = modelInfo.model->createJointAndBodyNodePair<
      dart::dynamics::FreeJoint>(nullptr, jointProperties, bodyProperties);

  dart::dynamics::FreeJoint * const joint = result.first;
  const Eigen::Isometry3d tf =
      GetParentModelFrame(modelInfo) * ResolveSdfPose(_sdfLink.SemanticPose());

  joint->setTransform(tf);

  dart::dynamics::BodyNode * const bn = result.second;

  const std::size_t linkID = this->AddLink(bn);
  this->AddJoint(joint);

  auto linkIdentity = this->GenerateIdentity(linkID, this->links.at(linkID));

  if (_sdfLink.Name() == modelInfo.canonicalLinkName ||
      (modelInfo.canonicalLinkName.empty() &&
       modelInfo.model->getNumBodyNodes() == 1))
  {
    // We just added the first link, so this is now the canonical link. We
    // should therefore move the "model frame" from the world onto this new
    // link, while preserving its location in the world frame.
    const dart::dynamics::SimpleFramePtr &modelFrame = modelInfo.frame;
    const Eigen::Isometry3d tf_frame = modelFrame->getWorldTransform();
    modelFrame->setParentFrame(bn);
    modelFrame->setTransform(tf_frame);
  }

  for (std::size_t i = 0; i < _sdfLink.CollisionCount(); ++i)
  {
    const auto collision = _sdfLink.CollisionByIndex(i);
    if (collision)
      this->ConstructSdfCollision(linkIdentity, *collision);
  }

  // ign-physics is currently ignoring visuals, so we won't parse them from the
  // SDF
//  for (std::size_t i = 0; i < _sdfLink.VisualCount(); ++i)
//  {
//    const auto visual = _sdfLink.VisualByIndex(i);
//    if (visual)
//      this->ConstructSdfVisual(linkID, *visual);
//  }

  return linkIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(
    const Identity &_modelID,
    const ::sdf::Joint &_sdfJoint)
{
  const auto &modelInfo = *this->ReferenceInterface<ModelInfo>(_modelID);
  dart::dynamics::BodyNode * const parent =
      modelInfo.model->getBodyNode(_sdfJoint.ParentLinkName());

  dart::dynamics::BodyNode * const child =
      modelInfo.model->getBodyNode(_sdfJoint.ChildLinkName());

  return ConstructSdfJoint(modelInfo, _sdfJoint, parent, child);
}

/////////////////////////////////////////////////
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

  const ShapeAndTransform st = ConstructGeometry(*_collision.Geom());
  const dart::dynamics::ShapePtr shape = st.shape;
  const Eigen::Isometry3d tf_shape = st.tf;

  if (!shape)
  {
    // The geometry element was empty, or the shape type is not supported
    return this->GenerateInvalidId();
  }

  dart::dynamics::BodyNode *const bn =
      this->ReferenceInterface<LinkInfo>(_linkID)->link.get();

  // NOTE(MXG): Gazebo requires unique collision shape names per Link, but
  // dartsim requires unique ShapeNode names per Skeleton, so we decorate the
  // Collision name for uniqueness sake.
  const std::string internalName =
      bn->getName() + ":" + _collision.Name();

  dart::dynamics::ShapeNode * const node =
      bn->createShapeNodeWith<
        dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(
          shape, internalName);

  // Calling GetElement creates a new element with default values if the element
  // doesn't exist, so the following is okay.
  // TODO(addisu) We are using the coefficient specified in the <ode> tag.
  // Either add parameters specific to DART or generic to all physics engines.
  uint16_t collideBitmask = 0xFF;
  if (_collision.Element())
  {
    const auto &odeFriction = _collision.Element()
                                  ->GetElement("surface")
                                  ->GetElement("friction")
                                  ->GetElement("ode");

#if DART_VERSION_AT_LEAST(6, 10, 0)
    auto aspect = node->getDynamicsAspect();
    aspect->setFrictionCoeff(odeFriction->Get<double>("mu"));
    if (odeFriction->HasElement("mu2"))
    {
      aspect->setSecondaryFrictionCoeff(odeFriction->Get<double>("mu2"));
    }
    if (odeFriction->HasElement("slip1"))
    {
      aspect->setSlipCompliance(odeFriction->Get<double>("slip1"));
    }
    if (odeFriction->HasElement("slip2"))
    {
      aspect->setSecondarySlipCompliance(odeFriction->Get<double>("slip2"));
    }
    if (odeFriction->HasElement("fdir1"))
    {
      math::Vector3d fdir1 = odeFriction->Get<math::Vector3d>("fdir1");
      aspect->setFirstFrictionDirection(math::eigen3::convert(fdir1));
    }

    const auto &surfaceBounce = _collision.Element()
                                    ->GetElement("surface")
                                    ->GetElement("bounce");

    if (surfaceBounce->HasElement("restitution_coefficient"))
    {
      aspect->setRestitutionCoeff(
          surfaceBounce->Get<double>("restitution_coefficient"));
    }
#else
    // We are setting the friction coefficient of a collision element
    // to be the coefficient for the whole link. If there are multiple collision
    // elements, the value of the last one will be the coefficient for the link.
    // TODO(addisu) Assign the coefficient to the shape node when support is
    // added in DART.
    bn->setFrictionCoeff(odeFriction->Get<double>("mu"));
#endif
    // TODO(anyone) add category_bitmask as well
    const auto bitmaskElement = _collision.Element()
                                     ->GetElement("surface")
                                     ->GetElement("contact");
    if (bitmaskElement->HasElement("collide_bitmask"))
      collideBitmask = bitmaskElement->Get<int>("collide_bitmask");
  }

  node->setRelativeTransform(ResolveSdfPose(_collision.SemanticPose()) *
                             tf_shape);

  const std::size_t shapeID =
      this->AddShape({node, _collision.Name(), tf_shape});
  auto identity = this->GenerateIdentity(shapeID, this->shapes.at(shapeID));

  this->SetCollisionFilterMask(identity, collideBitmask);
  return identity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfVisual(
    const Identity &_linkID,
    const ::sdf::Visual &_visual)
{
  if (!_visual.Geom())
  {
    ignerr << "The geometry element of visual [" << _visual.Name() << "] was a "
           << "nullptr\n";
    return this->GenerateInvalidId();
  }

  const ShapeAndTransform st = ConstructGeometry(*_visual.Geom());
  const dart::dynamics::ShapePtr shape = st.shape;
  const Eigen::Isometry3d tf_shape = st.tf;

  if (!shape)
  {
    // The geometry element was empty, or the shape type is not supported
    return this->GenerateInvalidId();
  }

  dart::dynamics::BodyNode *const bn =
      this->ReferenceInterface<LinkInfo>(_linkID)->link.get();

  // NOTE(MXG): Gazebo requires unique collision shape names per Link, but
  // dartsim requires unique ShapeNode names per Skeleton, so we decorate the
  // Collision name for uniqueness sake.
  const std::string internalName = bn->getName() + ":visual:" + _visual.Name();

  dart::dynamics::ShapeNode * const node =
      bn->createShapeNodeWith<dart::dynamics::VisualAspect>(
        shape, internalName);

  node->setRelativeTransform(ResolveSdfPose(_visual.SemanticPose()) * tf_shape);

  // TODO(MXG): Are there any other visual parameters that we can do anything
  // with? Do these visual parameters even matter, since dartsim is only
  // intended for the physics?
  if (_visual.Material())
  {
    const ignition::math::Color &color = _visual.Material()->Ambient();
    node->getVisualAspect()->setColor(
          Eigen::Vector4d(color.R(), color.G(), color.B(), color.A()));
  }

  const std::size_t shapeID = this->AddShape({node, _visual.Name(), tf_shape});
  return this->GenerateIdentity(shapeID, this->shapes.at(shapeID));
}

/////////////////////////////////////////////////
dart::dynamics::BodyNode *SDFFeatures::FindOrConstructLink(
    const dart::dynamics::SkeletonPtr &_model,
    const Identity &_modelID,
    const ::sdf::Model &_sdfModel,
    const std::string &_linkName)
{
  dart::dynamics::BodyNode * link = _model->getBodyNode(_linkName);
  if (link)
    return link;

  const ::sdf::Link * const sdfLink = _sdfModel.LinkByName(_linkName);
  if (!sdfLink)
  {
    if (_linkName != "world")
    {
      ignerr << "Model [" << _sdfModel.Name() << "] does not contain a Link "
             << "with the name [" << _linkName << "].\n";
    }
    return nullptr;
  }

  return this->links.at(this->ConstructSdfLink(_modelID, *sdfLink))->link.get();
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(
    const ModelInfo &_modelInfo,
    const ::sdf::Joint &_sdfJoint,
    dart::dynamics::BodyNode * const _parent,
    dart::dynamics::BodyNode * const _child)
{
  // if a specified link is named "world" but cannot be found, we'll assume the
  // joint is connected to the world
  bool worldParent = (!_parent && _sdfJoint.ParentLinkName() == "world");
  bool worldChild = (!_child && _sdfJoint.ChildLinkName() == "world");

  if (worldChild)
  {
    ignerr << "Asked to create a joint with the world as the child in model "
           << "[" << _modelInfo.model->getName() << "]. This is currently not "
           << "supported\n";

    return this->GenerateInvalidId();
  }

  // If either parent or child is null, it's only an error if the link is not
  // "world"
  if ((!_parent && !worldParent) || !_child)
  {
    {
      std::stringstream msg;
      msg << "Asked to create a joint from link [" << _sdfJoint.ParentLinkName()
          << "] to link [" << _sdfJoint.ChildLinkName() << "] in the model "
          << "[" << _modelInfo.model->getName() << "], but ";

      if (!_parent && !worldParent)
      {
        msg << "the parent link ";
        if (!_child)
          msg << " and ";
      }

      if (!_child)
        msg << "the child link ";

      msg << "could not be found in that model!\n";
      ignerr << msg.str();

      return this->GenerateInvalidId();
    }
  }

  {
    auto childsParentJoint = _child->getParentJoint();
    if (childsParentJoint->getType() != "FreeJoint")
    {
      ignerr << "Asked to create a joint between links "
             << "[" << _parent->getName() << "] as parent and ["
             << _child->getName() << "] as child, but the child link already "
             << "has a parent joint of type [" << childsParentJoint->getType()
             << "].\n";
      return this->GenerateInvalidId();
    }
    else if (_parent && _parent->descendsFrom(_child))
    {
      // TODO(MXG): Add support for non-tree graph structures
      ignerr << "Asked to create a closed kinematic chain between links "
             << "[" << _parent->getName() << "] and [" << _child->getName()
             << "], but that is not supported by the dartsim wrapper yet.\n";
      return this->GenerateInvalidId();
    }
  }

  // Save the current transforms of the links so we remember it later
  const Eigen::Isometry3d T_parent =
      _parent ? _parent->getWorldTransform() : Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d T_child = _child->getWorldTransform();

  const Eigen::Isometry3d T_joint =
      _child->getWorldTransform() * ResolveSdfPose(_sdfJoint.SemanticPose());

  const ::sdf::JointType type = _sdfJoint.Type();
  dart::dynamics::Joint *joint = nullptr;

  if (::sdf::JointType::BALL == type)
  {
    // SDF does not support any of the properties for ball joint, besides the
    // name and relative transforms to its parent and child, which will be taken
    // care of below. All other properties like joint limits, stiffness, etc,
    // will be the default values of +/- infinity or 0.0.
    joint = _child->moveTo<dart::dynamics::BallJoint>(_parent);
  }
  // TODO(MXG): Consider adding dartsim support for a CONTINUOUS joint type.
  // Alternatively, support the CONTINUOUS joint type by wrapping the
  // RevoluteJoint joint type.
  // TODO(MXG): Consider adding dartsim support for a GEARBOX joint type. It's
  // unclear to me whether it would be possible to get the same effect by
  // wrapping a RevoluteJoint type.
  else if (::sdf::JointType::PRISMATIC == type)
  {
    joint = ConstructSingleAxisJoint<dart::dynamics::PrismaticJoint>(
          _modelInfo, _sdfJoint, _parent, _child, T_joint);
  }
  else if (::sdf::JointType::REVOLUTE == type)
  {
    joint = ConstructSingleAxisJoint<dart::dynamics::RevoluteJoint>(
          _modelInfo, _sdfJoint, _parent, _child, T_joint);
  }
  // TODO(MXG): Consider adding dartsim support for a REVOLUTE2 joint type.
  // Alternatively, support the REVOLUTE2 joint type by wrapping two
  // RevoluteJoint objects into one.
  else if (::sdf::JointType::SCREW == type)
  {
    auto *screw = ConstructSingleAxisJoint<dart::dynamics::ScrewJoint>(
          _modelInfo, _sdfJoint, _parent, _child, T_joint);

    screw->setPitch(InvertThreadPitch(_sdfJoint.ThreadPitch()));
    joint = screw;
  }
  else if (::sdf::JointType::UNIVERSAL == type)
  {
    joint = ConstructUniversalJoint(
          _modelInfo, _sdfJoint, _parent, _child, T_joint);
  }
  else
  {
    // The joint type is either fixed or unsupported. If it's unsupported, print
    // out an error message and fall back to a fixed joint
    if (::sdf::JointType::FIXED != type)
    {
      ignerr << "Asked to construct a joint of sdf::JointType ["
             << static_cast<int>(type) << "], but that is not supported yet. "
             << "Creating a FIXED joint instead\n";
    }

    // A fixed joint does not have any properties besides the name and relative
    // transforms to its parent and child, which will be taken care of below.
    joint = _child->moveTo<dart::dynamics::WeldJoint>(_parent);
  }

  joint->setName(_sdfJoint.Name());

  // When initial positions are provided for joints, we need to correct the
  // parent transform:
  const Eigen::Isometry3d child_T_postjoint = T_child.inverse() * T_joint;
  const Eigen::Isometry3d parent_T_prejoint_init = T_parent.inverse() * T_joint;
  joint->setTransformFromParentBodyNode(parent_T_prejoint_init);
  joint->setTransformFromChildBodyNode(child_T_postjoint);

  // This is the transform inside the joint produced by whatever the current
  // joint position happens to be.
  const Eigen::Isometry3d T_child_parent_postjoint =
      _parent ? _child->getTransform(_parent) : _child->getTransform();

  const Eigen::Isometry3d prejoint_T_postjoint =
      parent_T_prejoint_init.inverse()
      * T_child_parent_postjoint
      * child_T_postjoint;

  // This is the corrected transform needed to get the child link to its
  // correct pose (as specified by the loaded SDF) for the current initial
  // position
  const Eigen::Isometry3d T_parent_postjoint =
      _parent ? _parent->getWorldTransform() : Eigen::Isometry3d::Identity();

  const Eigen::Isometry3d parent_T_prejoint_final =
      T_parent_postjoint.inverse()
      * T_child
      * child_T_postjoint
      * prejoint_T_postjoint.inverse();

  joint->setTransformFromParentBodyNode(parent_T_prejoint_final);

  const std::size_t jointID = this->AddJoint(joint);

  return this->GenerateIdentity(jointID, this->joints.at(jointID));
}

/////////////////////////////////////////////////
Eigen::Isometry3d SDFFeatures::ResolveSdfLinkReferenceFrame(
    const std::string &_frame,
    const ModelInfo &_modelInfo) const
{
  if (_frame.empty())
    return GetParentModelFrame(_modelInfo);

  ignerr << "Requested a reference frame of [" << _frame << "] but currently "
         << "only the model frame is supported as a reference frame for link "
         << "poses.\n";

  // TODO(MXG): Implement this when frame specifications are nailed down
  return Eigen::Isometry3d::Identity();
}

/////////////////////////////////////////////////
Eigen::Isometry3d SDFFeatures::ResolveSdfJointReferenceFrame(
    const std::string &_frame,
    const dart::dynamics::BodyNode *_child) const
{
  if (_frame.empty())
  {
    // This means the joint pose is expressed relative to the child link pose
    return _child->getWorldTransform();
  }

  ignerr << "Requested a reference frame of [" << _frame << "] but currently "
         << "only the child link frame is supported as a reference frame for "
         << "joint poses.\n";

  // TODO(MXG): Implement this when frame specifications are nailed down
  return Eigen::Isometry3d::Identity();
}

}
}
}
