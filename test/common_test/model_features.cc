/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include <gtest/gtest.h>

#include <gz/common/Console.hh>
#include <gz/plugin/Loader.hh>

#include "test/TestLibLoader.hh"
#include "test/Utils.hh"
#include "Worlds.hh"

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Model.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>

// ---------------------------------------------------------------------------
// Feature list
// ---------------------------------------------------------------------------
struct ModelFeaturesFeatureList : gz::physics::FeatureList<
  gz::physics::FindFreeGroupFeature,
  gz::physics::SetFreeGroupWorldVelocity,
  gz::physics::ForwardStep,
  gz::physics::GetEntities,
  gz::physics::LinkFrameSemantics,
  gz::physics::ModelStaticState,
  gz::physics::ModelGravityEnabled,
  gz::physics::sdf::ConstructSdfWorld
> { };

// ---------------------------------------------------------------------------
// Typed test fixture
// ---------------------------------------------------------------------------
template <class T>
class ModelFeaturesTest :
  public testing::Test, public gz::physics::TestLibLoader
{
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(ModelFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

// ---------------------------------------------------------------------------
// Helper: load a world from an SDF file using the given engine.
// ---------------------------------------------------------------------------
template <class T>
gz::physics::World3dPtr<T> LoadWorld(
    const gz::physics::Engine3dPtr<T> &_engine,
    const std::string &_sdfFile)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(_sdfFile);
  if (!errors.empty())
  {
    for (const auto &e : errors)
      std::cerr << e.Message() << "\n";
    return nullptr;
  }
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  if (!sdfWorld)
    return nullptr;
  return _engine->ConstructWorld(*sdfWorld);
}

// ---------------------------------------------------------------------------
// Helper: step the world N times and return the link's world position.
// ---------------------------------------------------------------------------
template <class T>
Eigen::Vector3d StepAndGetPosition(
    const gz::physics::World3dPtr<T> &_world,
    const gz::physics::Link3dPtr<T> &_link,
    std::size_t _numSteps)
{
  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  for (std::size_t i = 0; i < _numSteps; ++i)
  {
    _world->Step(output, state, input);
  }
  return _link->FrameDataRelativeToWorld().pose.translation();
}

// ---------------------------------------------------------------------------
// Instantiate test suite
// ---------------------------------------------------------------------------
using ModelFeaturesTestTypes = ::testing::Types<ModelFeaturesFeatureList>;
TYPED_TEST_SUITE(ModelFeaturesTest, ModelFeaturesTestTypes);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

//////////////////////////////////////////////////
TYPED_TEST(ModelFeaturesTest, ModelStaticStateDefault)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kSphereGravitySdf);
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("sphere");
    ASSERT_NE(nullptr, model);

    // Default: model is dynamic (not static).
    EXPECT_FALSE(model->GetStatic());
  }
}

//////////////////////////////////////////////////
TYPED_TEST(ModelFeaturesTest, ModelStaticStateSetGet)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kSphereGravitySdf);
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("sphere");
    ASSERT_NE(nullptr, model);

    EXPECT_FALSE(model->GetStatic());

    model->SetStatic(true);
    EXPECT_TRUE(model->GetStatic());

    model->SetStatic(false);
    EXPECT_FALSE(model->GetStatic());
  }
}

//////////////////////////////////////////////////
TYPED_TEST(ModelFeaturesTest, ModelStaticStatePreventsMotion)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    // sphere_gravity.sdf has gravity (0 0 -10) and a sphere at z=2.
    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kSphereGravitySdf);
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("sphere");
    ASSERT_NE(nullptr, model);

    auto link = model->GetLink("sphere_link");
    ASSERT_NE(nullptr, link);

    auto freeGroup = model->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);

    // Give the sphere an initial linear and angular velocity before freezing it.
    freeGroup->SetWorldLinearVelocity(Eigen::Vector3d(1.0, 0.5, 0.0));
    freeGroup->SetWorldAngularVelocity(Eigen::Vector3d(0.0, 0.0, 1.0));

    // Record the initial height.
    const double initialZ =
        link->FrameDataRelativeToWorld().pose.translation().z();

    // Make the model static: it must not move.
    model->SetStatic(true);
    EXPECT_TRUE(model->GetStatic());

    const double afterStaticZ =
        StepAndGetPosition<ModelFeaturesFeatureList>(world, link, 100).z();
    // Position must be frozen despite the non-zero stored velocities.
    EXPECT_NEAR(initialZ, afterStaticZ, 1e-9);

    // Re-enable mobility: gravity now integrates and the sphere falls.
    model->SetStatic(false);
    const double afterDynamicZ =
        StepAndGetPosition<ModelFeaturesFeatureList>(world, link, 100).z();
    EXPECT_LT(afterDynamicZ, initialZ);

    // Gravity must have pulled the sphere downward: Z linear velocity < 0.
    const auto dynamicFrameData = link->FrameDataRelativeToWorld();
    EXPECT_LT(dynamicFrameData.linearVelocity.z(), 0.0);
    // Angular velocity should remain near the initially-set value (no torques).
    EXPECT_NEAR(1.0, dynamicFrameData.angularVelocity.norm(), 1e-3);
  }
}

//////////////////////////////////////////////////
TYPED_TEST(ModelFeaturesTest, ModelGravityEnabledDefault)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kSphereGravitySdf);
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("sphere");
    ASSERT_NE(nullptr, model);

    // The SDF sphere has no <gravity> override, so it should be enabled.
    EXPECT_TRUE(model->GetGravityEnabled());
  }
}

//////////////////////////////////////////////////
TYPED_TEST(ModelFeaturesTest, ModelGravityEnabledSetGet)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kSphereGravitySdf);
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("sphere");
    ASSERT_NE(nullptr, model);

    EXPECT_TRUE(model->GetGravityEnabled());

    model->SetGravityEnabled(false);
    EXPECT_FALSE(model->GetGravityEnabled());

    model->SetGravityEnabled(true);
    EXPECT_TRUE(model->GetGravityEnabled());
  }
}

//////////////////////////////////////////////////
TYPED_TEST(ModelFeaturesTest, ModelGravityDisabledPreventsMotion)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    // sphere_gravity.sdf has gravity (0 0 -10) and a sphere at z=2.
    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kSphereGravitySdf);
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("sphere");
    ASSERT_NE(nullptr, model);

    auto link = model->GetLink("sphere_link");
    ASSERT_NE(nullptr, link);

    auto freeGroup = model->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);

    // Give the sphere an initial lateral velocity so we can verify it is
    // preserved when gravity is off (no energy dissipation on bare sphere),
    // and that only Z velocity grows once gravity is re-enabled.
    const Eigen::Vector3d initLinVel(1.0, 0.0, 0.0);
    const Eigen::Vector3d initAngVel(0.0, 0.0, 0.5);
    freeGroup->SetWorldLinearVelocity(initLinVel);
    freeGroup->SetWorldAngularVelocity(initAngVel);

    const Eigen::Vector3d initialPos =
        link->FrameDataRelativeToWorld().pose.translation();

    // Disable gravity: the sphere must not fall.
    model->SetGravityEnabled(false);
    EXPECT_FALSE(model->GetGravityEnabled());

    StepAndGetPosition<ModelFeaturesFeatureList>(world, link, 100);
    const auto noGravFrameData = link->FrameDataRelativeToWorld();
    const Eigen::Vector3d afterNoGravPos = noGravFrameData.pose.translation();

    // Z must stay fixed (no gravity).
    EXPECT_NEAR(initialPos.z(), afterNoGravPos.z(), 1e-6);
    // Y must stay fixed (no Y velocity).
    EXPECT_NEAR(initialPos.y(), afterNoGravPos.y(), 1e-6);
    // X must have increased (initLinVel.x = 1.0 m/s).
    EXPECT_GT(afterNoGravPos.x(), initialPos.x());

    // With gravity disabled, Z linear velocity must stay near zero.
    EXPECT_NEAR(0.0, noGravFrameData.linearVelocity.z(), 1e-6);
    // X linear velocity must be preserved.
    EXPECT_NEAR(initLinVel.x(), noGravFrameData.linearVelocity.x(), 1e-6);

    // Re-enable gravity: the sphere falls.
    model->SetGravityEnabled(true);
    StepAndGetPosition<ModelFeaturesFeatureList>(world, link, 100);
    const auto gravFrameData = link->FrameDataRelativeToWorld();
    const Eigen::Vector3d afterGravPos = gravFrameData.pose.translation();

    // Z must have dropped below the no-gravity position.
    EXPECT_LT(afterGravPos.z(), afterNoGravPos.z());
    // X must keep growing (lateral velocity is maintained).
    EXPECT_GT(afterGravPos.x(), afterNoGravPos.x());

    // Once gravity acts, the sphere must have acquired a negative Z velocity.
    EXPECT_LT(gravFrameData.linearVelocity.z(), 0.0);
    // X velocity is still positive.
    EXPECT_GT(gravFrameData.linearVelocity.x(), 0.0);
  }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!ModelFeaturesTest<ModelFeaturesFeatureList>::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
