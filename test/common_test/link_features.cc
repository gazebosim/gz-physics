/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "../helpers/TestLibLoader.hh"
#include "../Utils.hh"

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/math/eigen3/Conversions.hh>

// Features
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructLink.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
// gz::physics::AddLinkExternalForceTorque,
gz::physics::ForwardStep,
gz::physics::Gravity,
gz::physics::sdf::ConstructSdfWorld,
gz::physics::sdf::ConstructSdfModel,
gz::physics::sdf::ConstructSdfLink,
gz::physics::GetEntities,
gz::physics::GetLinkBoundingBox,
gz::physics::GetModelBoundingBox
>;

using TestEnginePtr = gz::physics::Engine3dPtr<Features>;
using TestWorldPtr = gz::physics::World3dPtr<Features>;


// A predicate-formatter for asserting that two vectors are approximately equal.
class AssertVectorApprox
{
public: explicit AssertVectorApprox(double _tol = 1e-6) : tol(_tol)
{
}

public: ::testing::AssertionResult operator()(
            const char *_mExpr, const char *_nExpr, Eigen::Vector3d _m,
            Eigen::Vector3d _n)
{
  if (gz::physics::test::Equal(_m, _n, this->tol))
    return ::testing::AssertionSuccess();

  return ::testing::AssertionFailure()
         << _mExpr << " and " << _nExpr << " ([" << _m.transpose()
         << "] and [" << _n.transpose() << "]"
         << ") are not equal";
}

private: double tol;
};

class LinkFeaturesTest:
  public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    auto plugins = loader.LoadLib(LinkFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<Features>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in " << GetLibToTest();
      // TODO(ahcorde): If we update gtest we can use here GTEST_SKIP()
    }
  }
};

/////////////////////////////////////////////////
TEST_F(LinkFeaturesTest, LinkForceTorque)
{
  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(PhysicsEngineName(name)) !=
                std::string::npos);

    sdf::Root root;
    const sdf::Errors errors = root.Load(TEST_WORLD_DIR "/test.world");
    EXPECT_TRUE(errors.empty()) << errors;
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    EXPECT_NE(nullptr, sdfWorld);

    auto graphErrors = sdfWorld->ValidateGraphs();
    EXPECT_EQ(0u, graphErrors.size()) << graphErrors;

    // Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

    TestWorldPtr world = engine->ConstructWorld(*sdfWorld);
    EXPECT_NE(nullptr, world);
    // world->SetGravity(gravity);
    //
    // AssertVectorApprox vectorPredicate(1e-10);
    // EXPECT_PRED_FORMAT2(vectorPredicate, gravity,
    //                     world->GetGravity());
  }
}

TEST_F(LinkFeaturesTest, AxisAlignedBoundingBox)
{
  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(PhysicsEngineName(name)) !=
                std::string::npos);

    sdf::Root root;
    const sdf::Errors errors = root.Load(TEST_WORLD_DIR "/test.world");
    EXPECT_TRUE(errors.empty()) << errors;
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    EXPECT_NE(nullptr, sdfWorld);

    auto graphErrors = sdfWorld->ValidateGraphs();
    EXPECT_EQ(0u, graphErrors.size()) << graphErrors;

    TestWorldPtr world = engine->ConstructWorld(*sdfWorld);
    EXPECT_NE(nullptr, world);

    auto model = world->GetModel("double_pendulum_with_base");
    auto baseLink = model->GetLink("base");
    auto bbox = baseLink->GetAxisAlignedBoundingBox();
    AssertVectorApprox vectorPredicate(1e-4);
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(0.2, -0.8, 0), bbox.min());
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(1.8, 0.8, 2.2), bbox.max());

    // test with non-world frame
    auto bboxModelFrame = baseLink->GetAxisAlignedBoundingBox(
        model->GetFrameID());
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(-0.8, -0.8, 0), bboxModelFrame.min());
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(0.8, 0.8, 2.2), bboxModelFrame.max());

    // test with non-world rotated frame
    auto upperLink = model->GetLink("upper_link");
    auto bboxUpperLinkFrame = baseLink->GetAxisAlignedBoundingBox(
        upperLink->GetFrameID());
    EXPECT_PRED_FORMAT2(vectorPredicate,
        gz::physics::Vector3d(-0.8, -0.1, -0.8), bboxUpperLinkFrame.min());
    EXPECT_PRED_FORMAT2(vectorPredicate,
        gz::physics::Vector3d(0.8, 2.1, 0.8), bboxUpperLinkFrame.max());
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  LinkFeaturesTest::init(argc, argv);
  return RUN_ALL_TESTS();
}
