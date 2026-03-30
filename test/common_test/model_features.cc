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
  gz::physics::GetNestedModelFromModel,
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
    // Re-apply angular velocity after the static→dynamic transition
    // (DART resets velocity state when the skeleton becomes mobile again).
    const Eigen::Vector3d angVel(0.0, 0.0, 1.0);
    freeGroup->SetWorldAngularVelocity(angVel);

    const double afterDynamicZ =
        StepAndGetPosition<ModelFeaturesFeatureList>(world, link, 100).z();
    EXPECT_LT(afterDynamicZ, initialZ);

    // Gravity must have pulled the sphere downward: Z linear velocity < 0.
    const auto dynamicFrameData = link->FrameDataRelativeToWorld();
    EXPECT_LT(dynamicFrameData.linearVelocity.z(), 0.0);
    // Angular velocity should remain near the set value (no torques acting).
    EXPECT_NEAR(angVel.norm(), dynamicFrameData.angularVelocity.norm(), 1e-3);
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

//////////////////////////////////////////////////
TYPED_TEST(ModelFeaturesTest, NestedModelGravityPropagates)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kWorldSingleNestedModelSdf);
    ASSERT_NE(nullptr, world);

    auto parentModel = world->GetModel("parent_model");
    ASSERT_NE(nullptr, parentModel);

    auto nestedModel = parentModel->GetNestedModel("nested_model");
    ASSERT_NE(nullptr, nestedModel);

    // Disable gravity on the parent: nested model must follow.
    parentModel->SetGravityEnabled(false);
    EXPECT_FALSE(parentModel->GetGravityEnabled());
    EXPECT_FALSE(nestedModel->GetGravityEnabled());

    // Re-enable gravity on the parent: nested model must follow.
    parentModel->SetGravityEnabled(true);
    EXPECT_TRUE(parentModel->GetGravityEnabled());
    EXPECT_TRUE(nestedModel->GetGravityEnabled());
  }
}

//////////////////////////////////////////////////
TYPED_TEST(ModelFeaturesTest, NestedModelStaticPropagates)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kWorldSingleNestedModelSdf);
    ASSERT_NE(nullptr, world);

    auto parentModel = world->GetModel("parent_model");
    ASSERT_NE(nullptr, parentModel);

    auto nestedModel = parentModel->GetNestedModel("nested_model");
    ASSERT_NE(nullptr, nestedModel);

    // Make the parent static: nested model must follow.
    parentModel->SetStatic(true);
    EXPECT_TRUE(parentModel->GetStatic());
    EXPECT_TRUE(nestedModel->GetStatic());

    // Make the parent dynamic again: nested model must follow.
    parentModel->SetStatic(false);
    EXPECT_FALSE(parentModel->GetStatic());
    EXPECT_FALSE(nestedModel->GetStatic());
  }
}

//////////////////////////////////////////////////
// Verify that toggling static does not silently re-enable gravity.
// Sequence: disable gravity → make static → make dynamic → gravity still off.
TYPED_TEST(ModelFeaturesTest, GravityPreservedAcrossStaticToggle)
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

    auto link = model->GetLink("sphere_link");
    ASSERT_NE(nullptr, link);

    // Step 1: disable gravity.
    model->SetGravityEnabled(false);
    EXPECT_FALSE(model->GetGravityEnabled());

    // Step 2: make static.
    model->SetStatic(true);
    EXPECT_TRUE(model->GetStatic());
    EXPECT_FALSE(model->GetGravityEnabled());

    // Step 3: make dynamic again.
    model->SetStatic(false);
    EXPECT_FALSE(model->GetStatic());

    // Step 4: gravity must still be disabled.
    EXPECT_FALSE(model->GetGravityEnabled());

    // Confirm no downward motion: the sphere must not fall.
    const double initialZ =
        link->FrameDataRelativeToWorld().pose.translation().z();
    const double afterZ =
        StepAndGetPosition<ModelFeaturesFeatureList>(world, link, 100).z();
    EXPECT_NEAR(initialZ, afterZ, 1e-6);
  }
}

//////////////////////////////////////////////////
// GetModelGravityEnabled must return true for a model whose only link has
// fluid added mass configured, even though SetLinkAddedMass forces DART
// gravity off internally on that body node.
// SetModelGravityEnabled skips added-mass links entirely, so for an
// all-added-mass model calling Set has no effect and the getter stays true.
TYPED_TEST(ModelFeaturesTest, AddedMassLinkDoesNotPollutGravityGetter)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kSphereAddedMassGravitySdf);
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("sphere_added_mass");
    ASSERT_NE(nullptr, model);

    // Despite the added-mass link having DART gravity mode = false internally,
    // the user-visible flag must read as enabled (user never called
    // SetGravityEnabled(false)).
    EXPECT_TRUE(model->GetGravityEnabled());

    // All links are added-mass links so SetGravityEnabled is a no-op;
    // the getter must not be corrupted by the internal DART flag.
    model->SetGravityEnabled(false);
    EXPECT_TRUE(model->GetGravityEnabled());

    model->SetGravityEnabled(true);
    EXPECT_TRUE(model->GetGravityEnabled());
  }
}

//////////////////////////////////////////////////
// SetModelGravityEnabled(true) must not re-enable DART's built-in gravity on
// an added-mass link. If it did, gravity would be applied twice (DART's
// built-in pass + the manual F=ma in SimulationFeatures) and the model would
// fall faster than a plain sphere of the same mass.
TYPED_TEST(ModelFeaturesTest, AddedMassLinkGravityInvariantPreserved)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<ModelFeaturesFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = LoadWorld<ModelFeaturesFeatureList>(
        engine, common_test::worlds::kSphereAddedMassGravitySdf);
    ASSERT_NE(nullptr, world);

    // Plain sphere: no added mass, reference fall rate.
    auto plainModel = world->GetModel("sphere");
    ASSERT_NE(nullptr, plainModel);
    auto plainLink = plainModel->GetLink("sphere_link");
    ASSERT_NE(nullptr, plainLink);

    // Added-mass sphere: same mass (1 kg), same starting height (z=10).
    auto amModel = world->GetModel("sphere_added_mass");
    ASSERT_NE(nullptr, amModel);
    auto amLink = amModel->GetLink("sphere_link");
    ASSERT_NE(nullptr, amLink);

    // Explicitly call SetGravityEnabled(true) to exercise the code path that
    // must NOT re-enable DART's built-in gravity on the added-mass body node.
    amModel->SetGravityEnabled(true);
    EXPECT_TRUE(amModel->GetGravityEnabled());

    // Step both models forward and compare heights. With added mass zz=1 and
    // body mass=1, effective inertial mass for vertical motion is 2 kg, so the
    // added-mass sphere falls more slowly (a = F/m_eff = 10/2 = 5 m/s^2)
    // compared to the plain sphere (a = 10 m/s^2). If SetGravityEnabled(true)
    // incorrectly re-enabled DART's built-in gravity, the added-mass sphere
    // would receive gravity twice and fall faster than the plain one.
    const std::size_t steps = 100;
    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;
    for (std::size_t i = 0; i < steps; ++i)
      world->Step(output, state, input);

    const double plainZ =
        plainLink->FrameDataRelativeToWorld().pose.translation().z();
    const double amZ =
        amLink->FrameDataRelativeToWorld().pose.translation().z();

    // The added-mass sphere must not have fallen faster than the plain one.
    // (If the invariant is broken, amZ < plainZ.)
    EXPECT_GE(amZ, plainZ)
        << "Added-mass sphere fell faster than plain sphere, suggesting "
           "SetGravityEnabled(true) incorrectly re-enabled DART built-in "
           "gravity on the added-mass link (double gravity applied).";
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
