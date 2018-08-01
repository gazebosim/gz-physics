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

#include <unordered_map>

#include <dart/constraint/WeldJointConstraint.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ScrewJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/simulation/World.hpp>

#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/Implements.hh>

#include <ignition/physics/sdf/ConstructWorld.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructJoint.hh>

#include <ignition/math/eigen3/Conversions.hh>

#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>

namespace ignition {
namespace physics {
namespace dartsim {

using DartsimFeatures = FeatureList<
  sdf::ConstructSdfWorld,
  sdf::ConstructSdfModel,
  sdf::ConstructSdfLink,
  sdf::ConstructSdfJoint
  // TODO(MXG): Implement these other features
/*  LinkFrameSemantics,
  GetBasicJointState,
  GetBasicJointProperties,
  SetBasicJointState,
  SetJointTransformFromParentFeature,
  SetJointTransformToChildFeature, */
>;

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

class Plugin : public Implements<FeaturePolicy3d, DartsimFeatures>
{
  public: Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    this->GetNextEntity();
    // dartsim does not have multiple "engines"
    return this->GenerateIdentity(0);
  }

  public: std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: Identity ConstructSdfWorld(
      const std::size_t /*_engine*/,
      const ::sdf::World &_sdfWorld) override
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

  public: dart::dynamics::BodyNode *FindOrConstructLink(
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

  public: Identity ConstructSdfModel(
      const std::size_t _worldID,
      const ::sdf::Model &_sdfModel) override
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

  public: Identity ConstructSdfLink(
      const std::size_t _modelID,
      const ::sdf::Link &_sdfLink) override
  {
    const dart::dynamics::SkeletonPtr model = models.at(_modelID);
    dart::dynamics::BodyNode::Properties properties;
    properties.mName = _sdfLink.Name();

    // TODO(MXG): Add visuals and collision shapes

    // TODO(MXG): Examine other "inertial" properties and make sure that they
    // are passed into dartsim correctly.
    const ignition::math::Inertiald &sdfInertia = _sdfLink.Inertial();
    properties.mInertia.setMass(sdfInertia.MassMatrix().Mass());
    properties.mInertia.setLocalCOM(
          math::eigen3::convert(sdfInertia.Pose().Pos()));

    // Note: When constructing a link from this function, we always instantiate
    // it as a standalone free body within the model. If it should have any
    // joint constraints, those will be added later.
    const auto result = model->createJointAndBodyNodePair<
        dart::dynamics::FreeJoint>(
          nullptr, dart::dynamics::FreeJoint::Properties(), properties);

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

  public: Identity ConstructSdfJoint(
      const std::size_t _modelID,
      const ::sdf::Joint &_sdfJoint) override
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

  public: Identity ConstructSdfJoint(
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

    if(::sdf::JointType::PRISMATIC == type)
    {
      joint = this->ConstructSimpleJoint<dart::dynamics::PrismaticJoint>(
            _model, _sdfJoint, _parent, _child);
    }
    else if (::sdf::JointType::REVOLUTE == type)
    {
      joint = this->ConstructSimpleJoint<dart::dynamics::RevoluteJoint>(
            _model, _sdfJoint, _parent, _child);
    }
    else if(::sdf::JointType::SCREW == type)
    {
      auto *screw = this->ConstructSimpleJoint<dart::dynamics::ScrewJoint>(
            _model, _sdfJoint, _parent, _child);

      ::sdf::ElementPtr element = _sdfJoint.Element();
      if (element->HasElement("thread_pitch"))
      {
        screw->setPitch(element->GetElement("thread_pitch")->Get<double>());
      }

      joint = screw;
    }
    else if(::sdf::JointType::FIXED == type)
    {
      // A fixed joint does not have any properties besides the relative
      // transforms to its parent and child, which will be taken care of below.
      joint = _child->moveTo<dart::dynamics::WeldJoint>(_parent);
    }
    else
    {
      // TODO(MXG): Finish constructing BALL and UNIVERSAL joint types. The
      // other joint types specified in SDF might be possible in dartsim by
      // taking liberties, but they are not natively supported.
      std::cerr << "[dartsim::ConstructSdfJoint] Error: Asked to construct a "
                << "joint of sdf::JointType [" << static_cast<int>(type)
                << "], but that is not supported yet.\n";
      return this->GenerateInvalidId();
    }

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
                << "is not supported yet.";
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

  template <typename JointType>
  JointType *ConstructSimpleJoint(
      const dart::dynamics::SkeletonPtr &/*_model*/,
      const ::sdf::Joint &_sdfJoint,
      dart::dynamics::BodyNode * const _parent,
      dart::dynamics::BodyNode * const _child)
  {
    typename JointType::Properties properties;
    properties.mName = _sdfJoint.Name();

    const ::sdf::JointAxis * const sdfAxis = _sdfJoint.Axis(0);
    properties.mAxis = ignition::math::eigen3::convert(sdfAxis->Xyz());

    if (sdfAxis->UseParentModelFrame())
    {
      // TODO(MXG): Account for "model frame" here using the _model argument
      std::cerr << "[dartsim::ConstructSdfJoint] Error: Asked to use parent "
                << "model frame for joint axis of [" << _sdfJoint.Name()
                << "], but this is not supported yet. Parent link frame will "
                << "be used instead.\n";
    }

    CopyStandardJointAxisProperties(0, properties, sdfAxis);
    return _child->moveTo<JointType>(_parent, properties);
  }

  std::size_t entityCount = 0;

  std::unordered_map<std::size_t, dart::simulation::WorldPtr> worlds;
  std::unordered_map<std::size_t, dart::dynamics::SkeletonPtr> models;
  std::unordered_map<std::size_t, dart::dynamics::BodyNodePtr> links;
  std::unordered_map<std::size_t, dart::dynamics::JointPtr> joints;
};

IGN_PHYSICS_ADD_PLUGIN(Plugin, FeaturePolicy3d, DartsimFeatures)

}
}
}
