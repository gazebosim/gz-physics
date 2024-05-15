/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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
#include <gz/physics/FindFeatures.hh>
#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetRayIntersection.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "test/Utils.hh"
#include "test/common_test/Worlds.hh"

struct TestFeatureList : gz::physics::FeatureList<
    gz::physics::CollisionDetector,
    gz::physics::ForwardStep,
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::GetRayIntersectionFromLastStepFeature
> { };

using namespace gz;

using TestEnginePtr = physics::Engine3dPtr<TestFeatureList>;
using TestWorldPtr = physics::World3dPtr<TestFeatureList>;

class SimulationFeaturesFixture : public ::testing::TestWithParam<const char *>
{
  protected: void SetUp() override
  {
    gz::plugin::Loader loader;
    loader.LoadLib(dartsim_plugin_LIB);

    gz::plugin::PluginPtr dartsim =
        loader.Instantiate("gz::physics::dartsim::Plugin");

    this->engine =
        gz::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
    ASSERT_NE(nullptr, this->engine);
  }
  protected: TestEnginePtr engine;
};

TestWorldPtr LoadWorld(
    const TestEnginePtr &_engine,
    const std::string &_sdfFile)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(_sdfFile);
  EXPECT_TRUE(errors.empty());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  return _engine->ConstructWorld(*sdfWorld);
}

// Test suite for raycasting with supported collision detectors
class RayIntersectionSupportedFixture : public SimulationFeaturesFixture {};

// Test suite for raycasting with unsupported collision detectors
class RayIntersectionNotSupportedFixture : public SimulationFeaturesFixture {};

TEST_P(RayIntersectionSupportedFixture, RayIntersection)
{
  const auto world = LoadWorld(this->engine, common_test::worlds::kSphereSdf);
  world->SetCollisionDetector(GetParam());

  // ray hits the sphere
  auto result = world->GetRayIntersectionFromLastStep(Eigen::Vector3d(-2, 0, 2),
                                                      Eigen::Vector3d( 2, 0, 2));
  auto rayIntersection =
      result.template Get<gz::physics::World3d<TestFeatureList>::RayIntersection>();

  double epsilon = 1e-3;
  EXPECT_TRUE(rayIntersection.point.isApprox(Eigen::Vector3d(-1, 0, 2), epsilon));
  EXPECT_TRUE(rayIntersection.normal.isApprox(Eigen::Vector3d(-1, 0, 0), epsilon));
  EXPECT_DOUBLE_EQ(rayIntersection.fraction, 0.25);

  // ray does not hit the sphere
  result = world->GetRayIntersectionFromLastStep(Eigen::Vector3d( 2, 0, 10),
                                                 Eigen::Vector3d(-2, 0, 10));
  rayIntersection =
      result.template Get<gz::physics::World3d<TestFeatureList>::RayIntersection>();

  ASSERT_TRUE(rayIntersection.point.array().isNaN().any());
  ASSERT_TRUE(rayIntersection.normal.array().isNaN().any());
  ASSERT_TRUE(std::isnan(rayIntersection.fraction));
}

TEST_P(RayIntersectionNotSupportedFixture, RayIntersection)
{
  const auto world = LoadWorld(this->engine, common_test::worlds::kSphereSdf);
  world->SetCollisionDetector(GetParam());

  // ray would hit the sphere, but the collision detector does not support ray intersection
  auto result = world->GetRayIntersectionFromLastStep(Eigen::Vector3d(-2, 0, 2),
                                                      Eigen::Vector3d(2, 0, 2));
  auto rayIntersection =
      result.template Get<gz::physics::World3d<TestFeatureList>::RayIntersection>();

  ASSERT_TRUE(rayIntersection.point.array().isNaN().any());
  ASSERT_TRUE(rayIntersection.normal.array().isNaN().any());
  ASSERT_TRUE(std::isnan(rayIntersection.fraction));
}

// Parameterized instantiation of test suites
INSTANTIATE_TEST_SUITE_P(CollisionDetectorsSupported, RayIntersectionSupportedFixture,
                         ::testing::Values("bullet"));

INSTANTIATE_TEST_SUITE_P(CollisionDetectorsNotSupported, RayIntersectionNotSupportedFixture,
                         ::testing::Values("ode", "dart", "fcl", "banana"));