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

#include <type_traits>

#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ScrewJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <gtest/gtest.h>

#include <tuple>

#include <gz/plugin/Loader.hh>

#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RevoluteJoint.hh>

#include <gz/physics/sdf/ConstructCollision.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructNestedModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/Utils.hh>
#include "test/common_test/Worlds.hh"
#include "Worlds.hh"

#include "World.hh"

using namespace gz;

struct TestFeatureList : physics::FeatureList<
    physics::GetEntities,
    physics::GetBasicJointState,
    physics::SetBasicJointState,
    physics::LinkFrameSemantics,
    physics::dartsim::RetrieveWorld,
    physics::sdf::ConstructSdfCollision,
    physics::sdf::ConstructSdfJoint,
    physics::sdf::ConstructSdfLink,
    physics::sdf::ConstructSdfModel,
    physics::sdf::ConstructSdfNestedModel,
    physics::sdf::ConstructSdfWorld
> { };

using World = physics::World3d<TestFeatureList>;
using WorldPtr = physics::World3dPtr<TestFeatureList>;
using ModelPtr = physics::Model3dPtr<TestFeatureList>;
using LinkPtr = physics::Link3dPtr<TestFeatureList>;

/////////////////////////////////////////////////
auto LoadEngine()
{
  plugin::Loader loader;
  loader.LoadLib(dartsim_plugin_LIB);

  plugin::PluginPtr dartsim =
      loader.Instantiate("gz::physics::dartsim::Plugin");

  auto engine =
      physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  return engine;
}

enum class LoaderType
{
  Whole,
  Piecemeal
};

/////////////////////////////////////////////////
WorldPtr LoadWorldWhole(const std::string &_world)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size());
  for (const auto & error : errors) {
    std::cout << error << std::endl;
  }

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);

  return world;
}

/////////////////////////////////////////////////
static gz::math::Pose3d ResolveSdfPose(
    const ::sdf::SemanticPose &_semPose)
{
  gz::math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  EXPECT_TRUE(errors.empty()) << errors;
  return pose;
}

static sdf::JointAxis ResolveJointAxis(const sdf::JointAxis &_unresolvedAxis)
{
  gz::math::Vector3d axisXyz;
  const sdf::Errors resolveAxisErrors = _unresolvedAxis.ResolveXyz(axisXyz);
  EXPECT_TRUE(resolveAxisErrors.empty()) << resolveAxisErrors;

  sdf::JointAxis resolvedAxis = _unresolvedAxis;

  const sdf::Errors setXyzErrors = resolvedAxis.SetXyz(axisXyz);
  EXPECT_TRUE(setXyzErrors.empty()) << setXyzErrors;

  resolvedAxis.SetXyzExpressedIn("");
  return resolvedAxis;
}

/////////////////////////////////////////////////
/// Downstream applications, like gz-sim, use this way of world construction
WorldPtr LoadWorldPiecemeal(const std::string &_world)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);
  if (nullptr == engine)
    return nullptr;

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size()) << errors;

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);
  if (nullptr == sdfWorld)
    return nullptr;

  sdf::World newWorld;
  newWorld.SetName(sdfWorld->Name());
  newWorld.SetGravity(sdfWorld->Gravity());
  auto world = engine->ConstructWorld(newWorld);
  if (nullptr == world)
    return nullptr;

  std::unordered_map<const sdf::Model *, ModelPtr> modelMap;
  std::unordered_map<const sdf::Link *, LinkPtr> linkMap;

  auto createModel = [&](const sdf::Model *_model,
                         const sdf::Model *_parentModel = nullptr) {
    ASSERT_NE(nullptr, _model);
    sdf::Model newSdfModel;
    newSdfModel.SetName(_model->Name());
    newSdfModel.SetRawPose(ResolveSdfPose(_model->SemanticPose()));
    newSdfModel.SetStatic(_model->Static());
    newSdfModel.SetSelfCollide(_model->SelfCollide());

    ModelPtr newModel;
    if (nullptr != _parentModel)
    {
      auto it = modelMap.find(_parentModel);
      ASSERT_TRUE(it != modelMap.end());
      newModel = it->second->ConstructNestedModel(newSdfModel);
    }
    else
    {
      newModel = world->ConstructModel(newSdfModel);
    }

    EXPECT_NE(nullptr, newModel);
    if (nullptr != newModel)
    {
      modelMap[_model] = newModel;
    }
  };

  for (uint64_t i = 0; i < sdfWorld->ModelCount(); ++i)
  {
    const auto *model = sdfWorld->ModelByIndex(i);
    createModel(model);
    for (uint64_t nestedInd = 0; nestedInd < model->ModelCount(); ++nestedInd)
    {
      createModel(model->ModelByIndex(nestedInd), model);
    }
  }

  for (auto [sdfModel, physModel] : modelMap)
  {
    for (uint64_t li = 0; li < sdfModel->LinkCount(); ++li)
    {
      const auto link = sdfModel->LinkByIndex(li);
      EXPECT_NE(nullptr, link);
      if (nullptr == link)
        return nullptr;

      sdf::Link newSdfLink;
      newSdfLink.SetName(link->Name());
      newSdfLink.SetRawPose(ResolveSdfPose(link->SemanticPose()));
      newSdfLink.SetInertial(link->Inertial());

      auto newLink = physModel->ConstructLink(newSdfLink);
      EXPECT_NE(nullptr, newLink);
      if (nullptr == newLink)
        return nullptr;

      linkMap[link] = newLink;
    }
  }

  for (auto [sdfLink, physLink] : linkMap)
  {
    for (uint64_t ci = 0; ci < sdfLink->CollisionCount(); ++ci)
    {
      physLink->ConstructCollision(*sdfLink->CollisionByIndex(ci));
    }
  }

  for (auto [sdfModel, physModel] : modelMap)
  {
    for (uint64_t ji = 0; ji < sdfModel->JointCount(); ++ji)
    {
      const auto sdfJoint = sdfModel->JointByIndex(ji);
      EXPECT_NE(nullptr, sdfJoint);
      if (nullptr == sdfJoint)
        return nullptr;

      std::string resolvedParentLinkName;
      const auto resolveParentErrors =
          sdfJoint->ResolveParentLink(resolvedParentLinkName);
      EXPECT_TRUE(resolveParentErrors.empty()) << resolveParentErrors;

      std::string resolvedChildLinkName;
      const auto resolveChildErrors =
        sdfJoint->ResolveChildLink(resolvedChildLinkName);
      EXPECT_TRUE (resolveChildErrors.empty()) << resolveChildErrors;

      sdf::Joint newSdfJoint;
      newSdfJoint.SetName(sdfJoint->Name());
      if (sdfJoint->Axis(0))
      {
        newSdfJoint.SetAxis(0, ResolveJointAxis(*sdfJoint->Axis(0)));
      }
      if (sdfJoint->Axis(1))
      {
        newSdfJoint.SetAxis(1, ResolveJointAxis(*sdfJoint->Axis(1)));
      }
      newSdfJoint.SetType(sdfJoint->Type());
      newSdfJoint.SetRawPose(ResolveSdfPose(sdfJoint->SemanticPose()));
      newSdfJoint.SetThreadPitch(sdfJoint->ThreadPitch());

      newSdfJoint.SetParentName(resolvedParentLinkName);
      newSdfJoint.SetChildName(resolvedChildLinkName);

      physModel->ConstructJoint(newSdfJoint);
    }
  }

  return world;
}

/////////////////////////////////////////////////
class SDFFeatures_TEST : public ::testing::TestWithParam<LoaderType>
{
  public: WorldPtr LoadWorld(const std::string &_world)
  {
    switch(this->GetParam())
    {
      case LoaderType::Whole:
        return LoadWorldWhole(_world);
      case LoaderType::Piecemeal:
        return LoadWorldPiecemeal(_world);
      default:
        std::cout << "Unknown LoaderType "
                  << std::underlying_type_t<LoaderType>(this->GetParam())
                  << " Using LoadWorldWhole" << std::endl;
        return LoadWorldWhole(_world);
    }
  }
};

// Run with different load world functions
INSTANTIATE_TEST_SUITE_P(LoadWorld, SDFFeatures_TEST,
                        ::testing::Values(LoaderType::Whole,
                                          LoaderType::Piecemeal));

/////////////////////////////////////////////////
// Test that the dartsim plugin loaded all the relevant information correctly.
TEST_P(SDFFeatures_TEST, CheckDartsimData)
{
  WorldPtr world = this->LoadWorld(common_test::worlds::kTestWorld);
  ASSERT_NE(nullptr, world);

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  ASSERT_EQ(9u, dartWorld->getNumSkeletons());

  const dart::dynamics::SkeletonPtr skeleton = dartWorld->getSkeleton(1);
  ASSERT_NE(nullptr, skeleton);
  EXPECT_EQ("double_pendulum_with_base", skeleton->getName());
  ASSERT_EQ(3u, skeleton->getNumBodyNodes());

  auto verify = [](const dart::dynamics::DegreeOfFreedom * dof,
                   double damping, double friction,
                   double springRest, double stiffness, double lower,
                   double upper, double maxForce, double maxVelocity)
  {
    EXPECT_DOUBLE_EQ(damping, dof->getDampingCoefficient());
    EXPECT_DOUBLE_EQ(friction, dof->getCoulombFriction());
    EXPECT_DOUBLE_EQ(springRest, dof->getRestPosition());
    EXPECT_DOUBLE_EQ(stiffness, dof->getSpringStiffness());
    EXPECT_DOUBLE_EQ(lower, dof->getPositionLowerLimit());
    EXPECT_DOUBLE_EQ(upper, dof->getPositionUpperLimit());
    EXPECT_DOUBLE_EQ(-maxForce, dof->getForceLowerLimit());
    EXPECT_DOUBLE_EQ(maxForce, dof->getForceUpperLimit());
    EXPECT_DOUBLE_EQ(-maxVelocity, dof->getVelocityLowerLimit());
    EXPECT_DOUBLE_EQ(maxVelocity, dof->getVelocityUpperLimit());
  };

  // Test that things were parsed correctly. These values are either stated or
  // implied in the test.world SDF file.
  verify(skeleton->getJoint(1)->getDof(0),
         3.0, 0.0, 0.0, 0.0,
         -std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity());

  verify(skeleton->getJoint(2)->getDof(0),
         3.0, 0.0, 0.0, 0.0,
         -std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity());

  /// \todo (anyone) getBodyNode("blah")->getFrictionCoeff is deprecated,
  /// disabling these tests.
  /*
  EXPECT_DOUBLE_EQ(1.1, skeleton->getBodyNode("base")->getFrictionCoeff());
  // The last collision element overwrites the value set by previous collision
  // elements. We expect mu=1, the default value, instead of 0.1.
  EXPECT_DOUBLE_EQ(1, skeleton->getBodyNode("upper_link")->getFrictionCoeff());
  // Gets the default value when the <surface> tag is missing
  EXPECT_DOUBLE_EQ(1, skeleton->getBodyNode("lower_link")->getFrictionCoeff());
  */

  for (const auto * joint : {skeleton->getJoint(1), skeleton->getJoint(2)})
  {
    const auto * revolute =
        dynamic_cast<const dart::dynamics::RevoluteJoint*>(joint);
    ASSERT_NE(nullptr, revolute);

    const Eigen::Vector3d &axis = revolute->getAxis();
    EXPECT_DOUBLE_EQ(1.0, axis[0]);
    EXPECT_DOUBLE_EQ(0.0, axis[1]);
    EXPECT_DOUBLE_EQ(0.0, axis[2]);
  }

  {
    const dart::dynamics::SkeletonPtr freeBody =
      dartWorld->getSkeleton("free_body");
    ASSERT_NE(nullptr, freeBody);
    ASSERT_EQ(1u, freeBody->getNumBodyNodes());
    const dart::dynamics::BodyNode *bn = freeBody->getBodyNode(0);
    ASSERT_NE(nullptr, bn);

    EXPECT_TRUE(dynamic_cast<const dart::dynamics::FreeJoint*>(
          bn->getParentJoint()));

    const Eigen::Vector3d translation = bn->getTransform().translation();
    EXPECT_DOUBLE_EQ(0.0, translation[0]);
    EXPECT_DOUBLE_EQ(10.0, translation[1]);
    EXPECT_DOUBLE_EQ(10.0, translation[2]);

#if DART_VERSION_AT_LEAST(6, 10, 0)
    const dart::dynamics::ShapeNode *collision1 = bn->getShapeNode(0);
    auto aspect = collision1->getDynamicsAspect();
    EXPECT_DOUBLE_EQ(0.8, aspect->getRestitutionCoeff());
#else
    EXPECT_DOUBLE_EQ(0.8, bn->getRestitutionCoeff());
  #endif
  }

  const dart::dynamics::SkeletonPtr screwJointTest =
      dartWorld->getSkeleton("screw_joint_test");
  ASSERT_NE(nullptr, screwJointTest);
  ASSERT_EQ(2u, screwJointTest->getNumBodyNodes());
  const auto *screwJoint = dynamic_cast<const dart::dynamics::ScrewJoint*>(
      screwJointTest->getJoint(1));
  ASSERT_NE(nullptr, screwJoint);
  EXPECT_DOUBLE_EQ(-GZ_PI, screwJoint->getPitch());

  const dart::dynamics::SkeletonPtr ballJointTest =
      dartWorld->getSkeleton("ball_joint_test");
  ASSERT_NE(nullptr, ballJointTest);
  ASSERT_EQ(2u, ballJointTest->getNumBodyNodes());
  const auto *ballJoint = dynamic_cast<const dart::dynamics::BallJoint*>(
      ballJointTest->getJoint(1));
  ASSERT_NE(nullptr, ballJoint);
}

/////////////////////////////////////////////////
// Test that joint limits are working by running the simulation
TEST_P(SDFFeatures_TEST, CheckJointLimitEnforcement)
{
  WorldPtr world = this->LoadWorld(common_test::worlds::kTestWorld);
  ASSERT_NE(nullptr, world);

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const auto model = world->GetModel("joint_limit_test");
  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton("joint_limit_test");
  ASSERT_NE(nullptr, skeleton);
  auto * const joint = dynamic_cast<dart::dynamics::RevoluteJoint *>(
      skeleton->getJoint(1));
  auto jointPhys = model->GetJoint(1);

  ASSERT_NE(nullptr, joint);
  // the joint starts at 0. Apply force in either direction and check the limits
  // are enforced
  auto verify = [&](std::size_t index, const double force, const double tol)
  {
    dartWorld->reset();
    dart::dynamics::DegreeOfFreedom * const dof = joint->getDof(index);
    jointPhys->SetForce(index, force);
    for (std::size_t i = 0; i < 1000; ++i)
    {
      dartWorld->step();
    }
    jointPhys->SetForce(index, force);
    EXPECT_LE(dof->getPositionLowerLimit() - tol, dof->getPosition());
    EXPECT_LE(dof->getForceLowerLimit() - tol, dof->getForce());
    EXPECT_LE(dof->getVelocityLowerLimit() - tol, dof->getVelocity());

    EXPECT_GE(dof->getPositionUpperLimit() + tol, dof->getPosition());
    EXPECT_GE(dof->getForceUpperLimit() + tol, dof->getForce());
    EXPECT_GE(dof->getVelocityUpperLimit() + tol, dof->getVelocity());
  };

  verify(0, -1000, 2e-3);
  verify(0, 1000, 2e-3);
}

// Create Model with parent and child links. If a link is not set, the joint
// will use the world as that link.
auto CreateTestModel(WorldPtr _world, const std::string &_model,
                     const std::optional<sdf::Link> &_parentLink,
                     const std::optional<sdf::Link> &_childLink)
{
  sdf::Model sdfModel;
  sdfModel.SetName(_model);
  auto model = _world->ConstructModel(sdfModel);
  EXPECT_NE(nullptr, model);

  sdf::Joint sdfJoint;
  sdfJoint.SetName("joint0");
  sdfJoint.SetType(sdf::JointType::REVOLUTE);
  if (_parentLink)
  {
    auto parent = model->ConstructLink(*_parentLink);
    EXPECT_NE(nullptr, parent);
    sdfJoint.SetParentName(_parentLink->Name());
  }
  else
  {
    sdfJoint.SetParentName("world");
  }

  if (_childLink)
  {
    auto child = model->ConstructLink(*_childLink);
    EXPECT_NE(nullptr, child);
    sdfJoint.SetChildName(_childLink->Name());
  }
  else
  {
    sdfJoint.SetChildName("world");
  }

  auto joint0 = model->ConstructJoint(sdfJoint);
  return std::make_tuple(model, joint0);
}

/////////////////////////////////////////////////
// Test joints with world as parent or child
TEST_P(SDFFeatures_TEST, WorldIsParentOrChild)
{
  auto engine = LoadEngine();
  ASSERT_NE(nullptr, engine);
  sdf::World sdfWorld;
  sdfWorld.SetName("default");
  auto world = engine->ConstructWorld(sdfWorld);
  EXPECT_NE(nullptr, world);

  std::optional<sdf::Link> parent = sdf::Link();
  parent->SetName("parent");
  std::optional<sdf::Link> child = sdf::Link();
  child->SetName("child");

  {
    const auto &[model, joint] =
        CreateTestModel(world, "test0", std::nullopt, std::nullopt);
    EXPECT_EQ(nullptr, joint);
  }
  {
    const auto &[model, joint] = CreateTestModel(world, "test1", parent, child);
    EXPECT_NE(nullptr, joint);
  }
  {
    const auto &[model, joint] =
        CreateTestModel(world, "test2", std::nullopt, child);
    EXPECT_NE(nullptr, joint);
  }
  {
    const auto &[model, joint] =
        CreateTestModel(world, "test3", parent, std::nullopt);
    EXPECT_EQ(nullptr, joint);
  }
}

/////////////////////////////////////////////////
// The model, link and joint structure in the
// test world `world_with_nested_model.sdf`:
//
//  model and link tree:
//    parent_model
//      nested_model
//        nested_link1
//        nested_link2
//      link1
//      nested_model2
//        nested_link1
//      nested_model3
//        link1
//    parent_model2
//      child_model
//        grand_child_model
//          link1
//
//  models:
//    parent_model
//    parent_model::nested_model
//    parent_model::nested_model2
//    parent_model::nested_model3
//    parent_model2
//    parent_model2::child_model
//    parent_model2::child_model::grand_child_model
//
//  links:
//    parent_model::nested_link1::nested_link1
//    parent_model::nested_link1::nested_link2
//    parent_model::link1
//    parent_model::nested_model2::nested_link1
//    parent_model::nested_model3::link1
//    parent_model2::child_model::grand_child_model::link1
//
//  joints:
//    parent_model::nested_model::nested_joint
//    parent_model::joint1
//
TEST_P(SDFFeatures_TEST, WorldWithNestedModel)
{
  WorldPtr world =
    this->LoadWorld(common_test::worlds::kWorldWithNestedModelSdf);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(2u, world->GetModelCount());

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  // check top level model
  EXPECT_EQ("parent_model", world->GetModel(0)->GetName());
  auto parentModel = world->GetModel("parent_model");
  ASSERT_NE(nullptr, parentModel);

  auto joint1 = parentModel->GetJoint("joint1");
  EXPECT_NE(nullptr, joint1);

  EXPECT_EQ(3u, parentModel->GetNestedModelCount());
  auto nestedModel = parentModel->GetNestedModel("nested_model");
  ASSERT_NE(nullptr, nestedModel);

  // DART associates the nested joint with the skeleton of the top level
  // model when the nested model is joined to the parent model, but Gazebo
  // should not find grandchild joints when querying a parent model.
  auto nestedJoint = parentModel->GetJoint("nested_joint");
  EXPECT_EQ(nullptr, nestedJoint);

  // The nested_joint should be found when querying the nested model.
  EXPECT_NE(nullptr, nestedModel->GetJoint("nested_joint"));

  EXPECT_EQ(1u, parentModel->GetLinkCount());
  EXPECT_NE(nullptr, parentModel->GetLink("link1"));
  EXPECT_EQ(nullptr, parentModel->GetLink("nested_link1"));
  EXPECT_EQ(nullptr, parentModel->GetLink("nested_link2"));

  ASSERT_EQ(2u, nestedModel->GetLinkCount());
  auto nestedLink1 = nestedModel->GetLink("nested_link1");
  ASSERT_NE(nullptr, nestedLink1);
  EXPECT_EQ(0u, nestedLink1->GetIndex());
  EXPECT_EQ(nestedLink1, nestedModel->GetLink(0));

  auto nestedLink2 = nestedModel->GetLink("nested_link2");
  ASSERT_NE(nullptr, nestedLink2);
  EXPECT_EQ(1u, nestedLink2->GetIndex());
  EXPECT_EQ(nestedLink2, nestedModel->GetLink(1));

  auto nestedModelSkel = dartWorld->getSkeleton("parent_model::nested_model");
  ASSERT_NE(nullptr, nestedModelSkel);
  // nested_model::nested_link1 would have moved to the parent_model skeleton so
  // we expect to not find it in the nested_model skeleton
  EXPECT_EQ(nullptr, nestedModelSkel->getBodyNode("nested_link1"));

  auto nestedModel2 = world->GetModel("parent_model::nested_model2");
  ASSERT_NE(nullptr, nestedModel2);
  EXPECT_EQ(1u, nestedModel2->GetLinkCount());
  EXPECT_NE(nullptr, nestedModel2->GetLink("nested_link1"));

  auto nestedModel3 = world->GetModel("parent_model::nested_model3");
  ASSERT_NE(nullptr, nestedModel3);
  EXPECT_EQ(1u, nestedModel3->GetLinkCount());
  EXPECT_NE(nullptr, nestedModel3->GetLink("link1"));
}

/////////////////////////////////////////////////
TEST_P(SDFFeatures_TEST, WorldWithNestedModelJointToWorld)
{
  WorldPtr world = this->LoadWorld(
    dartsim::worlds::kWorldWithNestedModelJointToWorldSdf);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(1u, world->GetModelCount());

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  // check top level model
  auto parentModel = world->GetModel("parent_model");
  ASSERT_NE(nullptr, parentModel);
  EXPECT_EQ("parent_model", parentModel->GetName());
  EXPECT_EQ(1u, parentModel->GetJointCount());
  EXPECT_EQ(1u, parentModel->GetLinkCount());

  auto joint1 = parentModel->GetJoint(0);
  ASSERT_NE(nullptr, joint1);
  EXPECT_EQ(joint1->GetName(), "joint1");

  auto link1 = parentModel->GetLink("link1");
  EXPECT_NE(nullptr, link1);

  auto nestedModel = parentModel->GetNestedModel("nested_model");
  ASSERT_NE(nullptr, nestedModel);
  EXPECT_EQ("nested_model", nestedModel->GetName());
  EXPECT_EQ(2u, nestedModel->GetLinkCount());
  EXPECT_EQ(2u, nestedModel->GetJointCount());

  auto nestedJoint1 = nestedModel->GetJoint("nested_joint1");
  EXPECT_NE(nullptr, nestedJoint1);

  auto nestedJoint2 = nestedModel->GetJoint("nested_joint2");
  EXPECT_NE(nullptr, nestedJoint2);

  auto nestedLink1 = nestedModel->GetLink("nested_link1");
  EXPECT_NE(nullptr, nestedLink1);

  auto nestedLink2 = nestedModel->GetLink("nested_link2");
  EXPECT_NE(nullptr, nestedLink2);
}

/////////////////////////////////////////////////
// Test that joint type falls back to fixed if the type is not supported
TEST_P(SDFFeatures_TEST, FallbackToFixedJoint)
{
  WorldPtr world = this->LoadWorld(common_test::worlds::kTestWorld);
  ASSERT_NE(nullptr, world);

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton("unsupported_joint_test");
  ASSERT_NE(nullptr, skeleton);
  ASSERT_EQ(6u, skeleton->getNumBodyNodes());

  for (const auto &jointName : {"j0", "j1", "j2"})
  {
    const auto *joint = skeleton->getJoint(jointName);
    ASSERT_NE(nullptr, joint)
      << " joint '" << jointName << "'doesn't exist in this skeleton";
    const auto *fixedJoint =
        dynamic_cast<const dart::dynamics::WeldJoint *>(joint);
    EXPECT_NE(nullptr, fixedJoint) << " joint type is: " << joint->getType();
  }
}

/////////////////////////////////////////////////
// Check that joints between links in different models work as expected
TEST_P(SDFFeatures_TEST, JointsAcrossNestedModels)
{
  WorldPtr world = this->LoadWorld(
    dartsim::worlds::kJointAcrossNestedModelsSdf);
  ASSERT_NE(nullptr, world);

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  auto checkModel = [&world](const std::string &_modelName){
    SCOPED_TRACE("checkModel " + _modelName);
    // check top level model
    auto parentModel = world->GetModel(_modelName);
    ASSERT_NE(nullptr, parentModel);

    auto link1 = parentModel->GetLink("link1");
    ASSERT_NE(nullptr, link1);

    auto nestedModel = parentModel->GetNestedModel("nested_model");
    ASSERT_NE(nullptr, nestedModel);

    auto link2 = nestedModel->GetLink("link2");
    ASSERT_NE(nullptr, link2);

    Eigen::Vector3d link1Pos =
        link1->FrameDataRelativeToWorld().pose.translation();
    Eigen::Vector3d link2Pos =
        link2->FrameDataRelativeToWorld().pose.translation();
    EXPECT_NEAR(0.25, link1Pos.z(), 1e-6);
    EXPECT_NEAR(0.25, link2Pos.z(), 1e-6);
  };

  {
    SCOPED_TRACE("Before step");
    checkModel("M1");
    checkModel("M2");
  }
  for (int i = 0; i < 1000; ++i)
  {
    dartWorld->step();
  }
  {
    SCOPED_TRACE("After step");
    checkModel("M1");
    checkModel("M2");
  }
}

/////////////////////////////////////////////////
class SDFFeatures_FrameSemantics: public SDFFeatures_TEST
{
};

// Run with different load world functions
INSTANTIATE_TEST_SUITE_P(LoadWorld, SDFFeatures_FrameSemantics,
                        ::testing::Values(LoaderType::Whole,
                                          LoaderType::Piecemeal));

/////////////////////////////////////////////////
TEST_P(SDFFeatures_FrameSemantics, LinkRelativeTo)
{
  WorldPtr world = this->LoadWorld(dartsim::worlds::kModelFramesSdf);
  ASSERT_NE(nullptr, world);
  const std::string modelName = "link_relative_to";

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton(modelName);

  ASSERT_NE(nullptr, skeleton);
  ASSERT_EQ(2u, skeleton->getNumBodyNodes());


  const dart::dynamics::BodyNode *link2 = skeleton->getBodyNode("L2");
  ASSERT_NE(nullptr, link2);

  // Expect the world pose of L2 to be 0 0 3 0 0 pi
  Eigen::Isometry3d expWorldPose =
      Eigen::Translation3d(0, 0, 3) *
      Eigen::AngleAxisd(GZ_PI, Eigen::Vector3d::UnitZ());

  dartWorld->step();

  // Step once and check
  EXPECT_TRUE(physics::test::Equal(
      expWorldPose, link2->getWorldTransform(), 1e-3));
}

/////////////////////////////////////////////////
TEST_P(SDFFeatures_FrameSemantics, CollisionRelativeTo)
{
  WorldPtr world = this->LoadWorld(dartsim::worlds::kModelFramesSdf);
  ASSERT_NE(nullptr, world);
  const std::string modelName = "collision_relative_to";

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton(modelName);

  ASSERT_NE(nullptr, skeleton);
  ASSERT_EQ(2u, skeleton->getNumBodyNodes());


  const dart::dynamics::BodyNode *link2 = skeleton->getBodyNode("L2");
  ASSERT_NE(nullptr, link2);

  const auto collision = link2->getShapeNode(0);
  ASSERT_TRUE(collision);
  // Expect the pose of c1 relative to L2 (the parent link) to be the same
  // as the pose of L1 relative to L2
  Eigen::Isometry3d expPose;
  expPose = Eigen::Translation3d(0, 0, -1);

  EXPECT_TRUE(physics::test::Equal(
      expPose, collision->getRelativeTransform(), 1e-5));

  // Step once and check, the relative pose should still be the same
  dartWorld->step();

  EXPECT_TRUE(physics::test::Equal(
      expPose, collision->getRelativeTransform(), 1e-5));
}

/////////////////////////////////////////////////
TEST_P(SDFFeatures_FrameSemantics, ExplicitFramesWithLinks)
{
  WorldPtr world = this->LoadWorld(dartsim::worlds::kModelFramesSdf);
  ASSERT_NE(nullptr, world);
  const std::string modelName = "explicit_frames_with_links";

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton(modelName);

  ASSERT_NE(nullptr, skeleton);
  ASSERT_EQ(2u, skeleton->getNumBodyNodes());


  const dart::dynamics::BodyNode *link1 = skeleton->getBodyNode("L1");
  ASSERT_NE(nullptr, link1);

  const dart::dynamics::BodyNode *link2 = skeleton->getBodyNode("L2");
  ASSERT_NE(nullptr, link2);

  // Expect the world pose of L1 to be the same as the world pose of F1
  Eigen::Isometry3d link1ExpPose;
  link1ExpPose = Eigen::Translation3d(1, 0, 1);

  EXPECT_TRUE(physics::test::Equal(
      link1ExpPose, link1->getWorldTransform(), 1e-5));

  // Expect the world pose of L2 to be the same as the world pose of F2, which
  // is at the origin of the model
  Eigen::Isometry3d link2ExpPose;
  link2ExpPose = Eigen::Translation3d(1, 0, 0);

  EXPECT_TRUE(physics::test::Equal(
      link2ExpPose, link2->getWorldTransform(), 1e-5));

  // Step once and check
  dartWorld->step();

  EXPECT_TRUE(physics::test::Equal(
      link1ExpPose, link1->getWorldTransform(), 1e-5));
  EXPECT_TRUE(physics::test::Equal(
      link2ExpPose, link2->getWorldTransform(), 1e-5));
}

/////////////////////////////////////////////////
TEST_P(SDFFeatures_FrameSemantics, ExplicitFramesWithCollision)
{
  WorldPtr world = this->LoadWorld(dartsim::worlds::kModelFramesSdf);
  ASSERT_NE(nullptr, world);
  const std::string modelName = "explicit_frames_with_collisions";

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton(modelName);

  ASSERT_NE(nullptr, skeleton);
  ASSERT_EQ(1u, skeleton->getNumBodyNodes());

  const dart::dynamics::BodyNode *link1 = skeleton->getBodyNode("L1");
  ASSERT_NE(nullptr, link1);

  const auto collision = link1->getShapeNode(0);
  ASSERT_TRUE(collision);

  // Expect the pose of c1 relative to L1 (the parent link) to be the same
  // as the pose of F1 relative to L1
  Eigen::Isometry3d expPose;
  expPose = Eigen::Translation3d(0, 0, 1);

  EXPECT_TRUE(physics::test::Equal(
      expPose, collision->getRelativeTransform(), 1e-5));

  // Step once and check
  dartWorld->step();

  EXPECT_TRUE(physics::test::Equal(
      expPose, collision->getRelativeTransform(), 1e-5));
}

/////////////////////////////////////////////////
TEST_P(SDFFeatures_FrameSemantics, ExplicitWorldFrames)
{
  WorldPtr world = this->LoadWorld(dartsim::worlds::kWorldFramesSdf);
  ASSERT_NE(nullptr, world);
  const std::string modelName = "M";

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton(modelName);

  ASSERT_NE(nullptr, skeleton);
  ASSERT_EQ(1u, skeleton->getNumBodyNodes());

  const dart::dynamics::BodyNode *link1 = skeleton->getBodyNode("L1");
  ASSERT_NE(nullptr, link1);

  // Expect the world pose of M to be (1 1 2 0 0 0) taking into acount the chain
  // of explicit frames relative to which its pose is expressed
  Eigen::Isometry3d expPose;
  expPose = Eigen::Translation3d(1, 1, 2);

  // Since we can't get the skeleton's world transform, we use the world
  // transform of L1 which is at the origin of the model frame.
  EXPECT_TRUE(physics::test::Equal(
      expPose, link1->getWorldTransform(), 1e-5));

  // Step once and check
  dartWorld->step();

  EXPECT_TRUE(physics::test::Equal(
      expPose, link1->getWorldTransform(), 1e-5));
}

/////////////////////////////////////////////////
TEST_P(SDFFeatures_TEST, Shapes)
{
  WorldPtr world = this->LoadWorld(common_test::worlds::kShapesWorld);
  ASSERT_NE(nullptr, world);

  auto dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  ASSERT_EQ(6u, dartWorld->getNumSkeletons());

  int count{0};
  for (auto name : {"sphere", "box", "cylinder", "capsule", "ellipsoid",
                    "cone"})
  {
    const auto skeleton = dartWorld->getSkeleton(count++);
    ASSERT_NE(nullptr, skeleton);
    EXPECT_EQ(name, skeleton->getName());
    ASSERT_EQ(1u, skeleton->getNumBodyNodes());
  }
}
