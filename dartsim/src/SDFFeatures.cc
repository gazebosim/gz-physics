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

#include <ignition/math/eigen3/Conversions.hh>

#include <dart/dynamics/BallJoint.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ScrewJoint.hpp>
#include <dart/dynamics/UniversalJoint.hpp>
#include <dart/constraint/WeldJointConstraint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>

#include "SDFFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

namespace {
/////////////////////////////////////////////////
template <typename Properties>
static void CopyStandardJointAxisProperties(
    const int _index, Properties &_properties, const ::sdf::JointAxis *_sdfAxis)
{
  _properties.mInitialPositions[_index] = _sdfAxis->InitialPosition();
  _properties.mDampingCoefficients[_index] = _sdfAxis->Damping();
  _properties.mFrictions[_index] = _sdfAxis->Friction();
  _properties.mRestPositions[_index] = _sdfAxis->SpringReference();
  _properties.mSpringStiffnesses[_index] = _sdfAxis->SpringStiffness();
  _properties.mPositionLowerLimits[_index] = _sdfAxis->Lower();
  _properties.mPositionUpperLimits[_index] = _sdfAxis->Upper();
  _properties.mForceLowerLimits[_index] = -_sdfAxis->Effort();
  _properties.mForceUpperLimits[_index] =  _sdfAxis->Effort();
  _properties.mVelocityLowerLimits[_index] = -_sdfAxis->MaxVelocity();
  _properties.mVelocityUpperLimits[_index] =  _sdfAxis->MaxVelocity();

  // TODO(MXG): Can dartsim support "Stiffness" and "Dissipation"?
}

/////////////////////////////////////////////////
static Eigen::Vector3d ConvertJointAxis(
    const ::sdf::JointAxis *_sdfAxis,
    const ::sdf::Joint &_sdfJoint,
    const dart::dynamics::SkeletonPtr &/*_model*/,
    const dart::dynamics::BodyNode * const /*_parent*/)
{
  Eigen::Vector3d axis = ignition::math::eigen3::convert(_sdfAxis->Xyz());

  if (_sdfAxis->UseParentModelFrame())
  {
    // TODO(MXG): Account for "model frame" here using the _model argument
    std::cerr << "[dartsim::ConstructSdfJoint] Error: Asked to use parent "
              << "model frame for joint axis of [" << _sdfJoint.Name()
              << "], but this is not supported yet. Parent link frame will "
              << "be used instead.\n";
  }

  return axis;
}

/////////////////////////////////////////////////
template <typename JointType>
static JointType *ConstructSingleAxisJoint(
    const dart::dynamics::SkeletonPtr &_model,
    const ::sdf::Joint &_sdfJoint,
    dart::dynamics::BodyNode * const _parent,
    dart::dynamics::BodyNode * const _child)
{
  typename JointType::Properties properties;

  const ::sdf::JointAxis * const sdfAxis = _sdfJoint.Axis(0);
  properties.mAxis = ConvertJointAxis(sdfAxis, _sdfJoint, _model, _parent);
  CopyStandardJointAxisProperties(0, properties, sdfAxis);

  return _child->moveTo<JointType>(_parent, properties);
}

/////////////////////////////////////////////////
static dart::dynamics::UniversalJoint *ConstructUniversalJoint(
    const dart::dynamics::SkeletonPtr &_model,
    const ::sdf::Joint &_sdfJoint,
    dart::dynamics::BodyNode * const _parent,
    dart::dynamics::BodyNode * const _child)
{
  dart::dynamics::UniversalJoint::Properties properties;

  for (const int index : {0, 1})
  {
    const ::sdf::JointAxis * const sdfAxis = _sdfJoint.Axis(index);
    properties.mAxis[index] =
        ConvertJointAxis(sdfAxis, _sdfJoint, _model, _parent);

    CopyStandardJointAxisProperties(index, properties, sdfAxis);
  }

  return _child->moveTo<dart::dynamics::UniversalJoint>(_parent, properties);
}
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const std::size_t /*_engine*/,
    const ::sdf::World &_sdfWorld)
{
  dart::simulation::WorldPtr world =
      std::make_shared<dart::simulation::World>();

  const std::size_t worldID = this->GetNextEntity();
  worlds[worldID] = world;

  world->setName(_sdfWorld.Name());
  world->setGravity(ignition::math::eigen3::convert(_sdfWorld.Gravity()));

  for (std::size_t i=0; i < _sdfWorld.ModelCount(); ++i)
  {
    const ::sdf::Model *model = _sdfWorld.ModelByIndex(i);

    if (!model)
      continue;

    this->ConstructSdfModel(worldID, *model);
  }

  return this->GenerateIdentity(worldID, world);
}

/////////////////////////////////////////////////
dart::dynamics::BodyNode *SDFFeatures::FindOrConstructLink(
    const dart::dynamics::SkeletonPtr &_model,
    const std::size_t _modelID,
    const ::sdf::Model &_sdfModel,
    const std::string &_linkName,
    const std::string &_jointName)
{
  dart::dynamics::BodyNode * link = _model->getBodyNode(_linkName);
  if (link)
    return link;

  const ::sdf::Link * const sdfLink = _sdfModel.LinkByName(_linkName);
  if (!sdfLink)
  {
    std::cerr << "[dartsim::ConstructSdfModel] Error: Model ["
              << _sdfModel.Name() << "] contains a nullptr Link with the "
              << "name [" << _linkName << "]. We will skip constructing "
              << "the joint [" << _jointName << "].\n";
    return nullptr;
  }

  return this->links.at(this->ConstructSdfLink(_modelID, *sdfLink));
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
    const std::size_t _worldID,
    const ::sdf::Model &_sdfModel)
{
  dart::dynamics::SkeletonPtr model =
      dart::dynamics::Skeleton::create(_sdfModel.Name());
  const std::size_t modelID = this->GetNextEntity();
  models[modelID] = model;

  for (std::size_t i=0; i < _sdfModel.JointCount(); ++i)
  {
    const ::sdf::Joint *sdfJoint = _sdfModel.JointByIndex(i);
    if (!sdfJoint)
    {
      std::cerr << "[dartsim::ConstructSdfModel] Error: The joint with "
                << "index [" << i << "] in model [" << _sdfModel.Name()
                << "] is a nullptr. It will be skipped.\n";
      continue;
    }

    const std::string &jointName = sdfJoint->Name();
    dart::dynamics::BodyNode * const parent = this->FindOrConstructLink(
          model, modelID, _sdfModel, sdfJoint->ParentLinkName(), jointName);

    dart::dynamics::BodyNode * const child = this->FindOrConstructLink(
          model, modelID, _sdfModel, sdfJoint->ChildLinkName(), jointName);

    if (!parent || !child)
      continue;

    this->ConstructSdfJoint(model, *sdfJoint, parent, child);
  }

  const dart::simulation::WorldPtr &world = worlds[_worldID];
  world->addSkeleton(model);

  return this->GenerateIdentity(modelID, model);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfLink(
    const std::size_t _modelID,
    const ::sdf::Link &_sdfLink)
{
  const dart::dynamics::SkeletonPtr model = models.at(_modelID);
  dart::dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = _sdfLink.Name();

  // TODO(MXG): Add visuals and collision shapes

  const ignition::math::Inertiald &sdfInertia = _sdfLink.Inertial();
  bodyProperties.mInertia.setMass(sdfInertia.MassMatrix().Mass());

  const Eigen::Matrix3d R_inertial{
        math::eigen3::convert(sdfInertia.Pose().Rot())};

  const Eigen::Matrix3d I_link =
      R_inertial
      * math::eigen3::convert(sdfInertia.Moi())
      * R_inertial.inverse();

  bodyProperties.mInertia.setMoment(I_link);

  bodyProperties.mInertia.setLocalCOM(
        math::eigen3::convert(sdfInertia.Pose().Pos()));

  dart::dynamics::FreeJoint::Properties jointProperties;
  jointProperties.mName = bodyProperties.mName + "_FreeJoint";
  // TODO(MXG): Consider adding a UUID to this joint name in order to avoid any
  // potential (albeit unlikely) name collisions.

  // Note: When constructing a link from this function, we always instantiate
  // it as a standalone free body within the model. If it should have any
  // joint constraints, those will be added later.
  const auto result = model->createJointAndBodyNodePair<
      dart::dynamics::FreeJoint>(nullptr, jointProperties, bodyProperties);

  dart::dynamics::FreeJoint * const joint = result.first;
  const Eigen::Isometry3d tf = math::eigen3::convert(_sdfLink.Pose());
  joint->setTransform(tf, dart::dynamics::Frame::World());

  dart::dynamics::BodyNode * const bn = result.second;

  const std::size_t linkID = this->GetNextEntity();
  links[linkID] = bn;
  const std::size_t jointID = this->GetNextEntity();
  joints[jointID] = joint;

  return this->GenerateIdentity(linkID);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(
    const std::size_t _modelID,
    const ::sdf::Joint &_sdfJoint)
{
  const dart::dynamics::SkeletonPtr &model = models[_modelID];
  dart::dynamics::BodyNode * const parent =
      model->getBodyNode(_sdfJoint.ParentLinkName());

  dart::dynamics::BodyNode * const child =
      model->getBodyNode(_sdfJoint.ChildLinkName());

  if(!parent || !child)
  {
    std::stringstream msg;
    msg << "[dartsim::ConstructSdfJoint] Error: Asked to create a joint from "
        << "link [" << _sdfJoint.ParentLinkName() << "] to link ["
        << _sdfJoint.ChildLinkName() << "] in the model [" << model->getName()
        << "], but ";

    if(!parent)
    {
      msg << "the parent link ";
      if(!child)
        msg << " and ";
    }

    if(!child)
      msg << "the child link ";

    msg << "could not be found in that model!\n";
    std::cerr << msg.str();

    return this->GenerateInvalidId();
  }

  return ConstructSdfJoint(model, _sdfJoint, parent, child);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(
    const dart::dynamics::SkeletonPtr &_model,
    const ::sdf::Joint &_sdfJoint,
    dart::dynamics::BodyNode * const _parent,
    dart::dynamics::BodyNode * const _child)
{
  if (_parent->descendsFrom(_child))
  {
    // TODO(MXG): Add support for non-tree graph structures
    std::cerr << "[dartsim::ConstructSdfJoint] Error: Asked to create a "
              << "closed kinematic chain between links ["
              << _parent->getName() << "] and [" << _child->getName()
              << "], but that is not supported by the dartsim wrapper yet.\n";
    return this->GenerateInvalidId();
  }

  // Save the current transform of the child link so we remember it later
  const Eigen::Isometry3d T_child_desired = _child->getTransform();

  const ::sdf::JointType type = _sdfJoint.Type();
  dart::dynamics::Joint *joint = nullptr;

  if(::sdf::JointType::BALL == type)
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
  else if(::sdf::JointType::FIXED == type)
  {
    // A fixed joint does not have any properties besides the name and relative
    // transforms to its parent and child, which will be taken care of below.
    joint = _child->moveTo<dart::dynamics::WeldJoint>(_parent);
  }
  // TODO(MXG): Consider adding dartsim support for a GEARBOX joint type. It's
  // unclear to me whether it would be possible to get the same effect by
  // wrapping a RevoluteJoint type.
  else if(::sdf::JointType::PRISMATIC == type)
  {
    joint = ConstructSingleAxisJoint<dart::dynamics::PrismaticJoint>(
          _model, _sdfJoint, _parent, _child);
  }
  else if (::sdf::JointType::REVOLUTE == type)
  {
    joint = ConstructSingleAxisJoint<dart::dynamics::RevoluteJoint>(
          _model, _sdfJoint, _parent, _child);
  }
  // TODO(MXG): Consider adding dartsim support for a REVOLUTE2 joint type.
  // Alternatively, support the REVOLUTE2 joint type by wrapping two
  // RevoluteJoint objects into one.
  else if(::sdf::JointType::SCREW == type)
  {
    auto *screw = ConstructSingleAxisJoint<dart::dynamics::ScrewJoint>(
          _model, _sdfJoint, _parent, _child);

    ::sdf::ElementPtr element = _sdfJoint.Element();
    if (element->HasElement("thread_pitch"))
    {
      screw->setPitch(element->GetElement("thread_pitch")->Get<double>());
    }

    joint = screw;
  }
  else if(::sdf::JointType::UNIVERSAL == type)
  {
    joint = ConstructUniversalJoint(_model, _sdfJoint, _parent, _child);
  }
  else
  {
    std::cerr << "[dartsim::ConstructSdfJoint] Error: Asked to construct a "
              << "joint of sdf::JointType [" << static_cast<int>(type)
              << "], but that is not supported yet.\n";
    return this->GenerateInvalidId();
  }

  joint->setName(_sdfJoint.Name());

  // Determine the transform from the parent link to the joint
  Eigen::Isometry3d T_parentToJoint =
      ignition::math::eigen3::convert(_sdfJoint.Pose());

  if (_sdfJoint.PoseFrame() != "")
  {
    // TODO(MXG): Account for the reference frame of the pose. It would be
    // good if SDF provided a way to identify which entity this is or at least
    // what type of entity. For example "link://model_name/name_of_link"
    std::cerr << "[dartsim::ConstructSdfJoint] Error: A reference frame has "
              << "been specified for [" << _sdfJoint.Name() << "], but this "
              << "is not supported yet.\n";
    // TODO(MXG): Also, the sdf/Joint.hh documentation says that the default
    // frame is the child frame rather than the parent frame. That makes very
    // little sense to me, so for now we'll assume it actually meant to say
    // "parent frame".
  }

  joint->setTransformFromParentBodyNode(T_parentToJoint);

  // Alter the transform from the joint to the child link so that the child
  // link returns to its original pose. This should be done after all other
  // changes to joint properties are finished.
  const Eigen::Isometry3d T_child_actual = _child->getTransform();
  joint->setTransformFromChildBodyNode(
        T_child_desired.inverse() * T_child_actual
        * joint->getTransformFromChildBodyNode());

  const std::size_t jointID = GetNextEntity();
  joints[jointID] = joint;

  return this->GenerateIdentity(jointID);
}

}
}
}
