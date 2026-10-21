/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#include <string>

#include <gz/common/Console.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "Base.hh"

using namespace gz;

struct TestFeatureList : physics::FeatureList<
    physics::ForwardStep,
    physics::GetEngineInfo,
    physics::GetWorldFromEngine,
    physics::GetModelFromWorld,
    physics::GetLinkFromModel,
    physics::LinkFrameSemantics,
    physics::AddLinkExternalForceTorque,
    physics::sdf::ConstructSdfModel,
    physics::sdf::ConstructSdfWorld
> { };

using WorldPtr = physics::World3dPtr<TestFeatureList>;

/////////////////////////////////////////////////
auto LoadEngine()
{
  plugin::Loader loader;
  loader.LoadLib(bullet_featherstone_plugin_LIB);

  plugin::PluginPtr plugin =
      loader.Instantiate("gz::physics::bullet_featherstone::Plugin");

  auto engine =
      physics::RequestEngine3d<TestFeatureList>::From(plugin);
  return engine;
}

/////////////////////////////////////////////////
WorldPtr LoadWorldString(const std::string &_worldSdf)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);
  if (!engine)
    return nullptr;

  sdf::Root root;
  const sdf::Errors &errors = root.LoadSdfString(_worldSdf);
  EXPECT_EQ(0u, errors.size());
  for (const auto &error : errors)
  {
    std::cout << error << std::endl;
  }

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);
  if (!sdfWorld)
    return nullptr;

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);
  return world;
}

/////////////////////////////////////////////////
TEST(SDFFeatures_TEST, BulletConstraintsDefaultParameters)
{
  const std::string sdfStr = R"(
    <?xml version="1.0" ?>
    <sdf version="1.10">
      <world name="default">
        <physics name="default_physics" type="bullet"/>
      </world>
    </sdf>)";

  WorldPtr world = LoadWorldString(sdfStr);
  ASSERT_NE(nullptr, world);

  auto *worldInfo = static_cast<physics::bullet_featherstone::WorldInfo *>(
      world->FullIdentity().ref.get());
  ASSERT_NE(nullptr, worldInfo);
  ASSERT_NE(nullptr, worldInfo->world);

  const auto &solverInfo = worldInfo->world->getSolverInfo();
  EXPECT_NEAR(0.0, solverInfo.m_globalCfm, 1e-6);
  EXPECT_NEAR(0.2, solverInfo.m_erp, 1e-6);
  EXPECT_NEAR(0.02, solverInfo.m_erp2, 1e-6);
  EXPECT_NEAR(0.2, solverInfo.m_frictionERP, 1e-6);
  EXPECT_NEAR(0.0, solverInfo.m_frictionCFM, 1e-6);
}

/////////////////////////////////////////////////
TEST(SDFFeatures_TEST, BulletConstraintsCustomParameters)
{
  const std::string sdfStr = R"(
    <?xml version="1.0" ?>
    <sdf version="1.10">
      <world name="default">
        <physics name="custom_physics" type="bullet">
          <bullet>
            <constraints>
              <cfm>0.05</cfm>
              <erp>0.4</erp>
              <erp2>0.15</erp2>
              <friction_erp>0.35</friction_erp>
              <friction_cfm>0.025</friction_cfm>
            </constraints>
          </bullet>
        </physics>
      </world>
    </sdf>)";

  WorldPtr world = LoadWorldString(sdfStr);
  ASSERT_NE(nullptr, world);

  auto *worldInfo = static_cast<physics::bullet_featherstone::WorldInfo *>(
      world->FullIdentity().ref.get());
  ASSERT_NE(nullptr, worldInfo);
  ASSERT_NE(nullptr, worldInfo->world);

  const auto &solverInfo = worldInfo->world->getSolverInfo();
  EXPECT_NEAR(0.05, solverInfo.m_globalCfm, 1e-6);
  EXPECT_NEAR(0.4, solverInfo.m_erp, 1e-6);
  EXPECT_NEAR(0.15, solverInfo.m_erp2, 1e-6);
  EXPECT_NEAR(0.35, solverInfo.m_frictionERP, 1e-6);
  EXPECT_NEAR(0.025, solverInfo.m_frictionCFM, 1e-6);
}

/////////////////////////////////////////////////
TEST(SDFFeatures_TEST, BulletConstraintsSimulationEffect)
{
  auto makeWorldSdf = [](double _cfm, double _frictionCfm)
  {
    return R"(
      <?xml version="1.0" ?>
      <sdf version="1.10">
        <world name="default">
          <gravity>0 0 -9.8</gravity>
          <physics name="custom_physics" type="bullet">
            <max_step_size>0.001</max_step_size>
            <bullet>
              <constraints>
                <cfm>)" + std::to_string(_cfm) + R"(</cfm>
                <friction_cfm>)" + std::to_string(_frictionCfm) + R"(</friction_cfm>
              </constraints>
            </bullet>
          </physics>
          <model name="ground">
            <static>true</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box><size>10 10 1</size></box>
                </geometry>
              </collision>
            </link>
          </model>
          <model name="box">
            <pose>0 0 1.0 0 0 0</pose>
            <link name="link">
              <inertial>
                <mass>10.0</mass>
                <inertia>
                  <ixx>1.66</ixx>
                  <iyy>1.66</iyy>
                  <izz>1.66</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry>
                  <box><size>1 1 1</size></box>
                </geometry>
              </collision>
            </link>
          </model>
        </world>
      </sdf>)";
  };

  WorldPtr rigidWorld = LoadWorldString(makeWorldSdf(0.0, 0.0));
  WorldPtr softWorld = LoadWorldString(makeWorldSdf(0.1, 0.5));
  ASSERT_NE(nullptr, rigidWorld);
  ASSERT_NE(nullptr, softWorld);

  auto rigidBoxLink = rigidWorld->GetModel("box")->GetLink("link");
  auto softBoxLink = softWorld->GetModel("box")->GetLink("link");
  ASSERT_NE(nullptr, rigidBoxLink);
  ASSERT_NE(nullptr, softBoxLink);

  physics::ForwardStep::Input input;
  physics::ForwardStep::State state;
  physics::ForwardStep::Output output;

  // Step until the boxes settle on the ground
  for (std::size_t i = 0; i < 500; ++i)
  {
    rigidWorld->Step(output, state, input);
    softWorld->Step(output, state, input);
  }

  const double rigidZ =
      rigidBoxLink->FrameDataRelativeToWorld().pose.translation().z();
  const double softZ =
      softBoxLink->FrameDataRelativeToWorld().pose.translation().z();

  // Larger cfm makes contact constraints more compliant, resulting in deeper
  // steady-state penetration under gravity.
  EXPECT_LT(softZ, rigidZ - 1e-2);

  // Apply a tangential horizontal force smaller than static friction limit
  // (mu = 1.0, normal force = 98 N, applied force = 20 N) and verify that
  // larger friction_cfm allows more tangential slip.
  for (std::size_t i = 0; i < 200; ++i)
  {
    rigidBoxLink->AddExternalForce(
        physics::RelativeForce3d(physics::FrameID::World(),
                                 Eigen::Vector3d(20.0, 0.0, 0.0)),
        physics::RelativePosition3d(*rigidBoxLink, Eigen::Vector3d::Zero()));
    softBoxLink->AddExternalForce(
        physics::RelativeForce3d(physics::FrameID::World(),
                                 Eigen::Vector3d(20.0, 0.0, 0.0)),
        physics::RelativePosition3d(*softBoxLink, Eigen::Vector3d::Zero()));
    rigidWorld->Step(output, state, input);
    softWorld->Step(output, state, input);
  }

  const double rigidX =
      rigidBoxLink->FrameDataRelativeToWorld().pose.translation().x();
  const double softX =
      softBoxLink->FrameDataRelativeToWorld().pose.translation().x();

  EXPECT_GT(softX, rigidX + 1e-2);
}
