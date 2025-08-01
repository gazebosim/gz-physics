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

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/ConeShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/HeightmapShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ScrewJoint.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/UniversalJoint.hpp>
#include <dart/constraint/WeldJointConstraint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/math/Helpers.hh>

#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Sphere.hh>
#include <sdf/Types.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "AddedMassFeatures.hh"
#include "CustomConeMeshShape.hh"
#include "CustomMeshShape.hh"

namespace gz {
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
      gzerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto &err : errors)
      {
        gzerr << err.Message() << std::endl;
      }
      gzerr << "There is no optimal fallback since the relative_to attribute["
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
    gzerr << "Zero thread pitch is not allowed.\n";
    assert(false);
  }

  return -2.0 * GZ_PI / _pitch;
}

/////////////////////////////////////////////////
template <typename Properties>
static void CopyStandardJointAxisProperties(
    const int _index, Properties &_properties,
    const ::sdf::JointAxis *_sdfAxis)
{
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

  const Eigen::Vector3d axis = math::eigen3::convert(_sdfAxis->Xyz());

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
  gzerr << "There was an error in JointAxis::ResolveXyz\n";
  for (const auto &err : errors)
  {
    gzerr << err.Message() << std::endl;
  }
  gzerr << "There is no optimal fallback since the expressed_in attribute["
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
template <typename JointType>
static JointType *ConstructBallJoint(
    const ModelInfo &/*_modelInfo*/,
    const ::sdf::Joint &_sdfJoint,
    dart::dynamics::BodyNode * const _parent,
    dart::dynamics::BodyNode * const _child,
    const Eigen::Isometry3d &/*_T_joint*/)
{
  // SDF does not support any of the properties for ball joint, besides the
  // name and relative transforms to its parent and child.
  //
  // To set other properties like joint limits, stiffness, etc,
  // apply values in <axis> to all 3 DoF.
  typename JointType::Properties properties;

  const ::sdf::JointAxis * const sdfAxis = _sdfJoint.Axis(0);

  // use default properties if sdfAxis is not set, otherwise apply to all DoF
  if (sdfAxis)
  {
    for (const std::size_t index : {0u, 1u, 2u})
    {
      CopyStandardJointAxisProperties(index, properties, sdfAxis);
    }
  }

  return _child->moveTo<JointType>(_parent, properties);
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
static ShapeAndTransform ConstructCapsule(
    const ::sdf::Capsule &_capsule)
{
  return {std::make_shared<dart::dynamics::CapsuleShape>(
        _capsule.Radius(), _capsule.Length())};
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
static ShapeAndTransform ConstructHeightmap(
    const ::sdf::Heightmap & /*_heightmap*/)
{
  // TODO(mjcarroll) Allow dartsim to construct heightmaps internally rather
  // than relying on the physics consumer constructing and attaching:
  // https://github.com/gazebosim/gz-physics/issues/451
  gzdbg << "Heightmap construction from an SDF has not been implemented yet "
        << "for dartsim. Use AttachHeightmapShapeFeature to use heightmaps.\n";
  return {nullptr};
}

/////////////////////////////////////////////////
static ShapeAndTransform ConstructMesh(
    const ::sdf::Mesh & /*_mesh*/)
{
  // TODO(MXG): Look into what kind of mesh URI we get here. Will it just be
  // a local file name, or do we need to resolve the URI?
  // TODO(mjcarroll) Allow dartsim to construct meshes internally rather
  // than relying on the physics consumer constructing and attaching:
  // https://github.com/gazebosim/gz-physics/issues/451
  gzdbg << "Mesh construction from an SDF has not been implemented yet for "
        << "dartsim. Use AttachMeshShapeFeature to use mesh shapes.\n";
  return {nullptr};
}

/////////////////////////////////////////////////
static ShapeAndTransform ConstructGeometry(
    const ::sdf::Geometry &_geometry)
{
  if (_geometry.BoxShape())
    return ConstructBox(*_geometry.BoxShape());
  else if (_geometry.CapsuleShape())
  {
    return ConstructCapsule(*_geometry.CapsuleShape());
  }
  else if (_geometry.ConeShape())
  {
    // TODO(anyone): Replace this code when Cone is supported by DART
    gzwarn << "DART: Cone is not a supported collision geomerty"
           << " primitive, using generated mesh of a cone instead"
           << std::endl;
    auto mesh =
      std::make_shared<CustomConeMeshShape>(_geometry.ConeShape()->Shape());
    auto mesh2 = std::dynamic_pointer_cast<dart::dynamics::MeshShape>(mesh);
    return {mesh2};
  }
  else if (_geometry.CylinderShape())
  {
    return ConstructCylinder(*_geometry.CylinderShape());
  }
  else if (_geometry.EllipsoidShape())
  {
    // TODO(anyone): Replace this code when Ellipsoid is supported by DART
    gzwarn << "DART: Ellipsoid is not a supported collision geomerty"
           << " primitive, using generated mesh of an ellipsoid instead"
           << std::endl;
    common::MeshManager *meshMgr = common::MeshManager::Instance();
    std::string ellipsoidMeshName = std::string("ellipsoid_mesh")
      + "_" + std::to_string(_geometry.EllipsoidShape()->Radii().X())
      + "_" + std::to_string(_geometry.EllipsoidShape()->Radii().Y())
      + "_" + std::to_string(_geometry.EllipsoidShape()->Radii().Z());
    meshMgr->CreateEllipsoid(
      ellipsoidMeshName,
      _geometry.EllipsoidShape()->Radii(),
      6, 12);
    const gz::common::Mesh * _mesh =
      meshMgr->MeshByName(ellipsoidMeshName);

    auto mesh = std::make_shared<CustomMeshShape>(*_mesh, Vector3d(1, 1, 1));
    auto mesh2 = std::dynamic_pointer_cast<dart::dynamics::MeshShape>(mesh);
    return {mesh2};
  }
  else if (_geometry.SphereShape())
    return ConstructSphere(*_geometry.SphereShape());
  else if (_geometry.PlaneShape())
    return ConstructPlane(*_geometry.PlaneShape());
  else if (_geometry.MeshShape())
    return ConstructMesh(*_geometry.MeshShape());
  else if (_geometry.HeightmapShape())
    return ConstructHeightmap(*_geometry.HeightmapShape());

  return {nullptr};
}

}  // namespace

/////////////////////////////////////////////////
dart::dynamics::BodyNode *SDFFeatures::FindBodyNode(
    const std::string &_worldName, const std::string &_jointModelName,
    const std::string &_linkRelativeName) const
{
  if (_linkRelativeName == "world")
    return nullptr;

  const auto fullName = ::sdf::JoinName(
      _worldName, ::sdf::JoinName(_jointModelName, _linkRelativeName));
  auto it = this->linksByName.find(fullName);
  if (it != this->linksByName.end())
  {
    return it->second;
  }
  gzerr << "Could not find link " << _linkRelativeName << " in model "
         << _jointModelName << std::endl;
  return nullptr;
}

std::optional<std::pair<dart::dynamics::BodyNode *, dart::dynamics::BodyNode *>>
SDFFeatures::FindParentAndChildOfJoint(std::size_t _worldID,
                                       const ::sdf::Joint *_sdfJoint,
                                       const std::string &_parentName,
                                       const std::string &_parentType) const
{

  // Resolve parent and child frames to links
  std::string parentLinkName;
  ::sdf::Errors errors = _sdfJoint->ResolveParentLink(parentLinkName);
  if (!errors.empty())
  {
    gzerr << "The link of the parent frame [" << _sdfJoint->ParentName()
      << "] of joint [" << _sdfJoint->Name() << "] in " << _parentType
      << " [" << _parentName
      << "] could not be resolved. The joint will not be constructed\n";
    for (const auto &error : errors)
    {
      gzerr << error << std::endl;
    }
    return {};
  }
  std::string childLinkName;
  errors = _sdfJoint->ResolveChildLink(childLinkName);
  if (!errors.empty())
  {
    gzerr << "The link of the child frame [" << _sdfJoint->ChildName()
      << "] of joint [" << _sdfJoint->Name() << "] in " << _parentType
      << " [" << _parentName
      << "] could not be resolved. The joint will not be constructed\n";
    for (const auto &error : errors)
    {
      gzerr << error << std::endl;
    }
    return {};
  }

  // When calling `FindBodyNode`, we need to check wheter the parent entity
  // (different from parent link/frame) of the joint is a model or world. If it
  // is a world, relative name we provide to `FindBodyNode` should *not* be
  // prefixed by the world name since is that step is done in `FindBodyNode`
  // itself.

  auto *const parent = this->FindBodyNode(
      this->worlds.at(_worldID)->getName(),
      _parentType == "model" ? _parentName : "", parentLinkName);

  if (nullptr == parent && parentLinkName != "world")
  {
    gzerr << "The parent [" << _sdfJoint->ParentName() << "] of joint ["
      << _sdfJoint->Name() << "] in " << _parentType << " [" << _parentName
      << "] was not found. The joint will not be constructed\n";
    return {};
  }

  auto *const child = this->FindBodyNode(
      this->worlds.at(_worldID)->getName(),
      _parentType == "model" ? _parentName : "", childLinkName);
  if (nullptr == child)
  {
    gzerr << "The child of joint [" << _sdfJoint->Name() << "] in "
          << _parentType << " [" << _parentName
          << "] was not found. The joint will not be constructed\n";
    return {};
  }

  return std::make_pair(parent, child);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const Identity &_engine,
    const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  const dart::simulation::WorldPtr &world = this->worlds.at(worldID);

  world->setGravity(math::eigen3::convert(_sdfWorld.Gravity()));

  // TODO(MXG): Add a Physics class to the SDFormat DOM and then parse that
  // information here. For now, we'll just use dartsim's default physics
  // parameters.

  for (std::size_t i = 0; i < _sdfWorld.ModelCount(); ++i)
  {
    const ::sdf::Model *model = _sdfWorld.ModelByIndex(i);

    if (!model)
      continue;

    this->ConstructSdfNestedModel(worldID, *model);
  }

  auto modelProxy = this->modelProxiesToWorld.MaybeAt(worldID);
  if (modelProxy)
  {
    for (std::size_t i = 0; i < _sdfWorld.JointCount(); ++i)
    {
      const ::sdf::Joint *sdfJoint = _sdfWorld.JointByIndex(i);
      if (!sdfJoint)
      {
        gzerr << "The joint with index [" << i << "] in world ["
              << _sdfWorld.Name() << "] is a nullptr. It will be skipped.\n";
        continue;
      }
      auto parentAndChild = this->FindParentAndChildOfJoint(
          worldID, sdfJoint, _sdfWorld.Name(), "world");
      if (parentAndChild)
      {
        auto [parent, child] = *parentAndChild;
        this->ConstructSdfJoint(this->GetWorldModel(worldID), *sdfJoint, parent,
                                child);
      }
    }
  }
  return worldID;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
    const Identity &_parentID,
    const ::sdf::Model &_sdfModel)
{
  return this->ConstructSdfModelImpl(_parentID, _sdfModel);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfNestedModel(const Identity &_parentID,
                                              const ::sdf::Model &_sdfModel)
{
  return this->ConstructSdfModelImpl(_parentID, _sdfModel);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModelImpl(
    std::size_t _parentID,
    const ::sdf::Model &_sdfModel)
{
  auto worldID = _parentID;
  std::string modelName = _sdfModel.Name();
  const bool isNested = this->models.HasEntity(_parentID);
  // If this is a nested model, find the world assocated with the model
  if (isNested)
  {
    worldID = this->GetWorldOfModelImpl(_parentID);

    const auto parentModelInfo = this->models.at(_parentID);
    const auto &skel = parentModelInfo->model;
    modelName = ::sdf::JoinName(skel->getName(), _sdfModel.Name());

    // Check to see if the nested model has already been constructed.
    // This can happen in the case that it was recursively created as
    // part of the parent model.
    // In this case, return the identity, rather than duplicating.
    for (const auto &nestedModelID : parentModelInfo->nestedModels)
    {
      auto nestedModel = this->models.at(nestedModelID);
      if (nestedModel->localName == _sdfModel.Name())
      {
        return this->GenerateIdentity(nestedModelID, nestedModel);
      }
    }
  }

  dart::dynamics::Frame *parentFrame = this->frames.at(_parentID);

  dart::dynamics::SkeletonPtr model =
      dart::dynamics::Skeleton::create(modelName);

  dart::dynamics::SimpleFramePtr modelFrame =
      dart::dynamics::SimpleFrame::createShared(
          parentFrame, modelName + "::__model__",
          ResolveSdfPose(_sdfModel.SemanticPose()));

  // Set canonical link name
  // TODO(anyone) This may not work correctly with nested models and will need
  // to be updated once multiple canonical links can exist in a nested model
  // https://github.com/gazebosim/gz-physics/issues/209
  auto [modelID, modelInfo] = [&] {
    if (isNested)
    {
      return this->AddNestedModel(  // NOLINT
          {model, _sdfModel.Name(), modelFrame, _sdfModel.CanonicalLinkName()},
          _parentID, worldID);
    }
    else
    {
      return this->AddModel(  // NOLINT
          {model, _sdfModel.Name(), modelFrame, _sdfModel.CanonicalLinkName()},
          worldID);
    }
  }();
  model->setMobile(!_sdfModel.Static());
  model->setSelfCollisionCheck(_sdfModel.SelfCollide());

  auto modelIdentity =
      this->GenerateIdentity(modelID, this->models.at(modelID));

  // First, recursively construct nested models
  for (std::size_t i = 0; i < _sdfModel.ModelCount(); ++i)
  {
    this->ConstructSdfModelImpl(modelID, *_sdfModel.ModelByIndex(i));
  }
  // then, construct all links
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
      gzerr << "The joint with index [" << i << "] in model ["
        << modelName << "] is a nullptr. It will be skipped.\n";
      continue;
    }
    auto parentAndChild =
        this->FindParentAndChildOfJoint(worldID, sdfJoint, modelName, "model");
    if (parentAndChild)
    {
      auto [parent, child] = *parentAndChild;
      this->ConstructSdfJoint(modelIdentity, *sdfJoint, parent, child);
    }
  }

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfLink(
    const Identity &_modelID,
    const ::sdf::Link &_sdfLink)
{
  // Return early if model is a proxy to the world.
  if (this->modelProxiesToWorld.MaybeAt(_modelID))
  {
    return this->GenerateInvalidId();
  }
  const auto &modelInfo = *this->ReferenceInterface<ModelInfo>(_modelID);
  dart::dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = _sdfLink.Name();

  const math::Inertiald &sdfInertia = _sdfLink.Inertial();
  bodyProperties.mInertia.setMass(sdfInertia.MassMatrix().Mass());

  const Eigen::Matrix3d I_link = math::eigen3::convert(sdfInertia.Moi());

  bodyProperties.mInertia.setMoment(I_link);

  const Eigen::Vector3d localCom =
      math::eigen3::convert(sdfInertia.Pose().Pos());

  bodyProperties.mInertia.setLocalCOM(localCom);

  bodyProperties.mGravityMode = _sdfLink.EnableGravity();

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

  auto worldID = this->GetWorldOfModelImpl(_modelID);
  if (worldID == INVALID_ENTITY_ID)
  {
    gzerr << "World of model [" << modelInfo.model->getName()
           << "] could not be found when creating link [" << _sdfLink.Name()
           << "]\n";
    return this->GenerateInvalidId();
  }

  auto world = this->worlds.at(worldID);
  const std::string fullName = ::sdf::JoinName(
      world->getName(),
      ::sdf::JoinName(modelInfo.model->getName(), bn->getName()));
  const std::size_t linkID = this->AddLink(bn, fullName, _modelID, sdfInertia);

  auto linkIdentity = this->GenerateIdentity(linkID, this->links.at(linkID));

  if (sdfInertia.FluidAddedMass().has_value())
  {
    auto* amf = dynamic_cast<AddedMassFeatures*>(this);

    if (nullptr == amf)
    {
      gzwarn << "Link [" << _sdfLink.Name() << "] in model ["
        << modelInfo.model->getName() <<
        "] has added mass specified in SDF, but AddedMassFeatures" <<
        "was not available on this engine.  Added mass will not be applied.\n";
    }
    else
    {
      amf->SetLinkAddedMass(linkIdentity, sdfInertia.FluidAddedMass().value());
    }
  }

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

  // gz-physics is currently ignoring visuals, so we won't parse them from the
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

  if (_sdfJoint.ChildName() == "world")
  {
    gzerr << "Asked to create a joint with the world as the child in model "
           << "[" << modelInfo.model->getName() << "]. This is currently not "
           << "supported\n";

    return this->GenerateInvalidId();
  }

  // If we get an error here, one of two things has occured:
  // 1. The Model _sdfJoint came from is invalid, i.e, had errors during
  //    sdf::Root::Load and ConstructSdfJoint was called regardless of the
  //    errors. Resolving for the parent link may fail for any number of reasons
  //    and should be reported.
  // 2. ConstructSdfJoint is being called with an sdf::Joint that was
  //    constructed by the user instead of one obtained from an sdf::Model
  //    object. In this case, the joint does not have a valid frame graph and it
  //    cannot be used to resolve for the parent link name. However, this can
  //    still be a valid use case for gz-sim that sends DOM objects
  //    piecemeal with all links and poses resolved.
  // Since (1) is an error that should be reported and (2) might be a valid use
  // case, the solution is, when an error is encountered, we first assume (2)
  // and set the parent and child link names to whatever returned from
  // sdf::Joint::ParentName() and sdf::Joint::ChildName respectively.
  // Then we check if a body node with the same relative name exists in DART. If
  // the link is nested inside a child model, it will be necessary to split the
  // name to identify the correct parent skeleton. If the corresponding body
  // node is found, an error will not be printed.
  //
  const std::size_t worldID = this->GetWorldOfModelImpl(_modelID);
  auto & world = this->worlds.at(worldID);

  std::string parentLinkName;
  const auto resolveParentErrors = _sdfJoint.ResolveParentLink(parentLinkName);
  if (!resolveParentErrors.empty()) {
    // It's possible this wasn't created from an sdf::Model object, like
    // SDFFeatures_TEST.WorldIsParentOrChild. Try using raw ParentName() in
    // that case
    parentLinkName = _sdfJoint.ParentName();
  }

  dart::dynamics::BodyNode * const parent =
    FindBodyNode(world->getName(), modelInfo.model->getName(), parentLinkName);

  std::string childLinkName;
  const auto childResolveErrors = _sdfJoint.ResolveChildLink(childLinkName);
  if (!childResolveErrors.empty()) {
    childLinkName = _sdfJoint.ChildName();
  }

  dart::dynamics::BodyNode * const child =
    FindBodyNode(world->getName(), modelInfo.model->getName(), childLinkName);

  if (nullptr == parent && parentLinkName != "world")
  {
    gzerr << "The link of the parent frame [" << _sdfJoint.ParentName()
           << "] with resolved link name [" << parentLinkName
           << "] of joint [" << _sdfJoint.Name()
           << "] could not be resolved. The joint will not be constructed\n";
    return this->GenerateInvalidId();
  }
  if (nullptr == child)
  {
    gzerr << "The link of the child frame [" << _sdfJoint.ChildName()
           << "] with resolved link name [" << childLinkName
           << "] of joint [" << _sdfJoint.Name() << "] in model ["
           << modelInfo.model->getName()
           << "] could not be resolved. The joint will not be constructed\n";
    return this->GenerateInvalidId();
  }

  return ConstructSdfJoint(_modelID, _sdfJoint, parent, child);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfCollision(
    const Identity &_linkID,
    const ::sdf::Collision &_collision)
{
  if (!_collision.Geom())
  {
    gzerr << "The geometry element of collision [" << _collision.Name() << "] "
           << "was a nullptr\n";
    return this->GenerateInvalidId();
  }

  const ShapeAndTransform st = ConstructGeometry(*_collision.Geom());
  const dart::dynamics::ShapePtr shape = st.shape;
  const Eigen::Isometry3d tf_shape = st.tf;

  if (!shape)
  {
    // The geometry element was empty, or the shape type is not supported
    gzdbg << "The geometry element of collision [" << _collision.Name() << "] "
          << "couldn't be created\n";
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
      aspect->setPrimarySlipCompliance(odeFriction->Get<double>("slip1"));
    }
    if (odeFriction->HasElement("slip2"))
    {
      aspect->setSecondarySlipCompliance(odeFriction->Get<double>("slip2"));
    }
    if (odeFriction->HasElement("fdir1"))
    {
      auto frictionDirectionElem = odeFriction->GetElement("fdir1");
      math::Vector3d fdir1 = frictionDirectionElem->Get<math::Vector3d>();
      aspect->setFirstFrictionDirection(math::eigen3::convert(fdir1));

      std::string expressedIn = "gz:expressed_in";

      if (frictionDirectionElem->HasAttribute(expressedIn))
      {
        auto skeleton = bn->getSkeleton();
        auto directionFrameBodyNode = skeleton->getBodyNode(
          frictionDirectionElem->Get<std::string>(expressedIn));
        if (nullptr != directionFrameBodyNode)
        {
          aspect->setFirstFrictionDirectionFrame(directionFrameBodyNode);
        }
        else
        {
          gzwarn << "Failed to get body node for [" << _collision.Name()
                  << "], not setting friction direction frame." << std::endl;
        }
      }
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
    const auto &surfaceBounce = _collision.Element()
                                    ->GetElement("surface")
                                    ->GetElement("bounce");

    if (surfaceBounce->HasElement("restitution_coefficient"))
    {
      bn->setRestitutionCoeff(
          surfaceBounce->Get<double>("restitution_coefficient"));
    }
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
    gzerr << "The geometry element of visual [" << _visual.Name() << "] was a "
           << "nullptr\n";
    return this->GenerateInvalidId();
  }

  const ShapeAndTransform st = ConstructGeometry(*_visual.Geom());
  const dart::dynamics::ShapePtr shape = st.shape;
  const Eigen::Isometry3d tf_shape = st.tf;

  if (!shape)
  {
    // The geometry element was empty, or the shape type is not supported
    gzerr << "The geometry element of visual [" << _visual.Name() << "] "
           << "couldn't be created\n";
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
    const math::Color &color = _visual.Material()->Ambient();
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
      gzerr << "Model [" << _sdfModel.Name() << "] does not contain a Link "
             << "with the name [" << _linkName << "].\n";
    }
    return nullptr;
  }

  return this->links.at(this->ConstructSdfLink(_modelID, *sdfLink))->link.get();
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(
    const Identity &_modelID,
    const ::sdf::Joint &_sdfJoint,
    dart::dynamics::BodyNode * const _parent,
    dart::dynamics::BodyNode * const _child)
{
  const auto &_modelInfo = *this->ReferenceInterface<ModelInfo>(_modelID);
  // if a specified link is named "world" but cannot be found, we'll assume the
  // joint is connected to the world
  bool worldParent = (!_parent && _sdfJoint.ParentName() == "world");
  bool worldChild = (!_child && _sdfJoint.ChildName() == "world");

  if (worldChild)
  {
    gzerr << "Asked to create a joint with the world as the child in model "
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
      msg << "Asked to create a joint from link [" << _sdfJoint.ParentName()
          << "] to link [" << _sdfJoint.ChildName() << "] in the model "
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
      gzerr << msg.str();

      return this->GenerateInvalidId();
    }
  }

  {
    auto childsParentJoint = _child->getParentJoint();
    std::string parentName = worldParent? "world" : _parent->getName();
    if (childsParentJoint->getType() != "FreeJoint")
    {
      gzerr << "Asked to create a joint between links "
             << "[" << parentName << "] as parent and ["
             << _child->getName() << "] as child, but the child link already "
             << "has a parent joint of type [" << childsParentJoint->getType()
             << "].\n";
      return this->GenerateInvalidId();
    }
    else if (_parent && _parent->descendsFrom(_child))
    {
      // TODO(MXG): Add support for non-tree graph structures
      gzerr << "Asked to create a closed kinematic chain between links "
             << "[" << parentName << "] and [" << _child->getName()
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
    joint = ConstructBallJoint<dart::dynamics::BallJoint>(
          _modelInfo, _sdfJoint, _parent, _child, T_joint);
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
      gzerr << "Asked to construct a joint of sdf::JointType ["
             << static_cast<int>(type) << "], but that is not supported yet. "
             << "Creating a FIXED joint instead\n";
    }

    // A fixed joint does not have any properties besides the name and relative
    // transforms to its parent and child, which will be taken care of below.
    joint = _child->moveTo<dart::dynamics::WeldJoint>(_parent);
  }

  const std::string jointName = _sdfJoint.Name();
  joint->setName(jointName);

  const Eigen::Isometry3d child_T_postjoint = T_child.inverse() * T_joint;
  const Eigen::Isometry3d parent_T_prejoint_init = T_parent.inverse() * T_joint;
  joint->setTransformFromParentBodyNode(parent_T_prejoint_init);
  joint->setTransformFromChildBodyNode(child_T_postjoint);

  const std::string fullJointName =
      this->FullyScopedJointName(_modelID, jointName);

  const std::size_t jointID = this->AddJoint(joint, fullJointName, _modelID);
  // Increment BodyNode version since the child could be moved to a new skeleton
  // when a joint is created.
  // TODO(azeey) Remove incrementVersion once DART has been updated to
  // internally increment the BodyNode's version after Joint::moveTo.
  _child->incrementVersion();

  return this->GenerateIdentity(jointID, this->joints.at(jointID));
}

/////////////////////////////////////////////////
Eigen::Isometry3d SDFFeatures::ResolveSdfLinkReferenceFrame(
    const std::string &_frame,
    const ModelInfo &_modelInfo) const
{
  if (_frame.empty())
    return GetParentModelFrame(_modelInfo);

  gzerr << "Requested a reference frame of [" << _frame << "] but currently "
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

  gzerr << "Requested a reference frame of [" << _frame << "] but currently "
         << "only the child link frame is supported as a reference frame for "
         << "joint poses.\n";

  // TODO(MXG): Implement this when frame specifications are nailed down
  return Eigen::Isometry3d::Identity();
}

}
}
}
