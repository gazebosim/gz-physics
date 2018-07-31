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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>

#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/Implements.hh>

#include <ignition/physics/sdf/ConstructWorld.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructLink.hh>

#include <ignition/math/eigen3/Conversions.hh>

#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Joint.hh>
#include <sdf/Link.hh>

namespace ignition {
namespace physics {
namespace dartsim {

using DartsimFeatures = FeatureList<
  sdf::ConstructSdfWorld,
  sdf::ConstructSdfModel,
  sdf::ConstructSdfLink
  // TODO(MXG): Implement these other features
/*  LinkFrameSemantics,
  GetBasicJointState,
  GetBasicJointProperties,
  SetBasicJointState,
  SetJointTransformFromParentFeature,
  SetJointTransformToChildFeature, */
>;

class Plugin : public Implements<FeaturePolicy3d, DartsimFeatures>
{
  public: Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    ++entityCount;
    // dartsim does not have multiple "engines"
    return this->GenerateIdentity(0);
  }

  public: Identity ConstructSdfWorld(
      const std::size_t /*_engine*/,
      const ::sdf::World &_sdfWorld) override
  {
    dart::simulation::WorldPtr world =
        std::make_shared<dart::simulation::World>();

    const std::size_t worldID = entityCount++;
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

  public: Identity ConstructSdfModel(
      const std::size_t _worldID,
      const ::sdf::Model &_sdfModel)
  {
    dart::dynamics::SkeletonPtr model =
        dart::dynamics::Skeleton::create(_sdfModel.Name());
    const std::size_t modelID = entityCount++;
    models[modelID] = model;

    for (std::size_t i=0; i < _sdfModel.JointCount(); ++i)
    {
      const ::sdf::Joint *sdfJoint = _sdfModel.JointByIndex(i);

    }

    return this->GenerateIdentity(modelID, model);
  }

  public: Identity ConstructSdfLink(
      const std::size_t _modelID,
      const ::sdf::Link &_sdfLink)
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

    const auto result = model->createJointAndBodyNodePair<
        dart::dynamics::FreeJoint>(
          nullptr, dart::dynamics::FreeJoint::Properties(), properties);

    dart::dynamics::FreeJoint * const joint = result.first;
    const Eigen::Isometry3d tf = math::eigen3::convert(_sdfLink.Pose());
    joint->setTransform(tf, dart::dynamics::Frame::World());

    dart::dynamics::BodyNode * const bn = result.second;

    const std::size_t linkID = entityCount++;
    links[linkID] = bn;
    const std::size_t jointID = entityCount++;
    joints[jointID] = joint;

    return this->GenerateIdentity(linkID);
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
