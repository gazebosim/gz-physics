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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ScrewJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <gtest/gtest.h>

#include <tuple>

#include <ignition/plugin/Loader.hh>

#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/RequestEngine.hh>

#include <ignition/physics/sdf/ConstructJoint.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <ignition/physics/dartsim/World.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/Utils.hh>

struct TestFeatureList : ignition::physics::FeatureList<
    ignition::physics::GetEntities,
    ignition::physics::GetBasicJointState,
    ignition::physics::SetBasicJointState,
    ignition::physics::dartsim::RetrieveWorld,
    ignition::physics::sdf::ConstructSdfJoint,
    ignition::physics::sdf::ConstructSdfLink,
    ignition::physics::sdf::ConstructSdfModel,
    ignition::physics::sdf::ConstructSdfWorld
> { };

using World = ignition::physics::World3d<TestFeatureList>;
using WorldPtr = ignition::physics::World3dPtr<TestFeatureList>;

auto LoadEngine()
{
  ignition::plugin::Loader loader;
  loader.LoadLib(DartsimPluginLib());

  ignition::plugin::PluginPtr dartsim =
      loader.Instantiate("ignition::physics::dartsim::Plugin");

  auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  return engine;
}

World LoadWorld(const std::string &_world)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size());

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);

  return *world;
}

// Test that the dartsim plugin loaded all the relevant information correctly.
TEST(SDFFeatures_TEST, CheckDartsimData)
{
  World world = LoadWorld(TEST_WORLD_DIR"/test.world");

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  ASSERT_EQ(6u, dartWorld->getNumSkeletons());

  const dart::dynamics::SkeletonPtr skeleton = dartWorld->getSkeleton(1);
  ASSERT_NE(nullptr, skeleton);
  ASSERT_EQ(3u, skeleton->getNumBodyNodes());

  auto verify = [](const dart::dynamics::DegreeOfFreedom * dof,
                   double initialPos, double damping, double friction,
                   double springRest, double stiffness, double lower,
                   double upper, double maxForce, double maxVelocity)
  {
    EXPECT_DOUBLE_EQ(initialPos, dof->getPosition());
    EXPECT_DOUBLE_EQ(initialPos, dof->getInitialPosition());
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
         1.5706796, 3.0, 0.0, 0.0, 0.0, -1e16, 1e16,
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity());

  verify(skeleton->getJoint(2)->getDof(0),
         -0.429462, 3.0, 0.0, 0.0, 0.0, -1e16, 1e16,
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
  EXPECT_DOUBLE_EQ(-IGN_PI, screwJoint->getPitch());
}

// Test that joint limits are working by running the simulation
TEST(SDFFeatures_TEST, CheckJointLimitEnforcement)
{
  World world = LoadWorld(TEST_WORLD_DIR"/test.world");

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const auto model = world.GetModel("joint_limit_test");
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
    sdfJoint.SetParentLinkName(_parentLink->Name());
  }
  else
  {
    sdfJoint.SetParentLinkName("world");
  }

  if (_childLink)
  {
    auto child = model->ConstructLink(*_childLink);
    EXPECT_NE(nullptr, child);
    sdfJoint.SetChildLinkName(_childLink->Name());
  }
  else
  {
    sdfJoint.SetChildLinkName("world");
  }

  auto joint0 = model->ConstructJoint(sdfJoint);
  return std::make_tuple(model, joint0);
}

// Test joints with world as parent or child
TEST(SDFFeatures_TEST, WorldIsParentOrChild)
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

// Test that joint type falls back to fixed if the type is not supported
TEST(SDFFeatures_TEST, FallbackToFixedJoint)
{
  World world = LoadWorld(TEST_WORLD_DIR"/test.world");

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton("unsupported_joint_test");
  ASSERT_NE(nullptr, skeleton);
  ASSERT_EQ(6u, skeleton->getNumBodyNodes());

  for (const auto &jointName : {"j0", "j1", "j2"})
  {
    const auto *joint = skeleton->getJoint(jointName);
    const auto *fixedJoint =
        dynamic_cast<const dart::dynamics::WeldJoint *>(joint);
    EXPECT_NE(nullptr, fixedJoint) << " joint type is: " << joint->getType();
  }
}

TEST(SDFFeatures_FrameSemantics, LinkRelativeTo)
{
  World world = LoadWorld(TEST_WORLD_DIR"/model_frames.sdf");
  const std::string modelName = "link_relative_to";

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
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
      Eigen::AngleAxisd(IGN_PI, Eigen::Vector3d::UnitZ());

  dartWorld->step();

  // Step once and check
  EXPECT_TRUE(ignition::physics::test::Equal(
      expWorldPose, link2->getWorldTransform(), 1e-3));
}

TEST(SDFFeatures_FrameSemantics, CollisionRelativeTo)
{
  World world = LoadWorld(TEST_WORLD_DIR"/model_frames.sdf");
  const std::string modelName = "collision_relative_to";

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
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

  EXPECT_TRUE(ignition::physics::test::Equal(
      expPose, collision->getRelativeTransform(), 1e-5));

  // Step once and check, the relative pose should still be the same
  dartWorld->step();

  EXPECT_TRUE(ignition::physics::test::Equal(
      expPose, collision->getRelativeTransform(), 1e-5));
}

TEST(SDFFeatures_FrameSemantics, ExplicitFramesWithLinks)
{
  World world = LoadWorld(TEST_WORLD_DIR"/model_frames.sdf");
  const std::string modelName = "explicit_frames_with_links";

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
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

  EXPECT_TRUE(ignition::physics::test::Equal(
      link1ExpPose, link1->getWorldTransform(), 1e-5));

  // Expect the world pose of L2 to be the same as the world pose of F2, which
  // is at the origin of the model
  Eigen::Isometry3d link2ExpPose;
  link2ExpPose = Eigen::Translation3d(1, 0, 0);

  EXPECT_TRUE(ignition::physics::test::Equal(
      link2ExpPose, link2->getWorldTransform(), 1e-5));

  // Step once and check
  dartWorld->step();

  EXPECT_TRUE(ignition::physics::test::Equal(
      link1ExpPose, link1->getWorldTransform(), 1e-5));
  EXPECT_TRUE(ignition::physics::test::Equal(
      link2ExpPose, link2->getWorldTransform(), 1e-5));
}

TEST(SDFFeatures_FrameSemantics, ExplicitFramesWithCollision)
{
  World world = LoadWorld(TEST_WORLD_DIR"/model_frames.sdf");
  const std::string modelName = "explicit_frames_with_collisions";

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
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

  EXPECT_TRUE(ignition::physics::test::Equal(
      expPose, collision->getRelativeTransform(), 1e-5));

  // Step once and check
  dartWorld->step();

  EXPECT_TRUE(ignition::physics::test::Equal(
      expPose, collision->getRelativeTransform(), 1e-5));
}

TEST(SDFFeatures_FrameSemantics, ExplicitWorldFrames)
{
  World world = LoadWorld(TEST_WORLD_DIR"/world_frames.sdf");
  const std::string modelName = "M";

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
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
  EXPECT_TRUE(ignition::physics::test::Equal(
      expPose, link1->getWorldTransform(), 1e-5));

  // Step once and check
  dartWorld->step();

  EXPECT_TRUE(ignition::physics::test::Equal(
      expPose, link1->getWorldTransform(), 1e-5));
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
