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

#include <string>
#include <unordered_set>

#include <gz/common/Console.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Loader.hh>

#include "test/Resources.hh"
#include "test/TestLibLoader.hh"
#include "Worlds.hh"

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FreeJoint.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/mesh/MeshShape.hh>
#include <gz/physics/PlaneShape.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>

template <class T>
class CollisionTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(CollisionTest::GetLibToTest());

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


struct CollisionFeaturesList : gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::LinkFrameSemantics,
  gz::physics::ForwardStep,
  gz::physics::GetEntities,
  gz::physics::ConstructEmptyWorldFeature,
  gz::physics::ConstructEmptyModelFeature,
  gz::physics::ConstructEmptyLinkFeature,
  gz::physics::mesh::AttachMeshShapeFeature,
  gz::physics::AttachPlaneShapeFeature,
  gz::physics::SetFreeJointRelativeTransformFeature,
  gz::physics::AttachFixedJointFeature
> { };

using CollisionTestTypes =
  ::testing::Types<CollisionFeaturesList>;
TYPED_TEST_SUITE(CollisionTest,
                 CollisionTestTypes);

TYPED_TEST(CollisionTest, MeshAndPlane)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<CollisionFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = engine->ConstructEmptyWorld("world");
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation()[2] = 2.0;

    auto model = world->ConstructEmptyModel("mesh");
    auto link = model->ConstructEmptyLink("link");

    const std::string meshFilename = gz::physics::test::resources::kChassisDae;
    auto &meshManager = *gz::common::MeshManager::Instance();
    auto *mesh = meshManager.Load(meshFilename);
    ASSERT_NE(nullptr, mesh);

    // TODO(anyone): This test is somewhat awkward because we lift up the mesh
    // from the center of the link instead of lifting up the link or the model.
    // We're doing this because we don't currently have an API for moving models
    // or links around. See the conversation here for more:
    // https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/46/page/1#comment-87592809
    link->AttachMeshShape("mesh", *mesh, tf);

    model = world->ConstructEmptyModel("plane");
    link = model->ConstructEmptyLink("link");

    link->AttachPlaneShape("plane", gz::physics::LinearVector3d::UnitZ());
    link->AttachFixedJoint(nullptr);

    const auto link2 = world->GetModel(0)->GetLink(0);

    EXPECT_NEAR(
          0.0, link2->FrameDataRelativeToWorld().pose.translation()[2], 1e-6);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;
    for (std::size_t i = 0; i < 1000; ++i)
    {
      world->Step(output, state, input);
    }

    // Make sure the mesh was stopped by the plane. In 2000 time steps at the
    // default step size of 0.001, a free falling body should drop approximately
    // 19.6 meters. As long as the body is somewhere near 1.91, then it has been
    // stopped by the plane (the exact value might vary because the body might
    // be rocking side-to-side after falling).
    EXPECT_NEAR(
          -1.91, link2->FrameDataRelativeToWorld().pose.translation()[2], 0.05);
  }
}

struct CollisionMeshFeaturesList : gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::LinkFrameSemantics,
  gz::physics::ForwardStep,
  gz::physics::GetEntities
> { };

template <class T>
class CollisionMeshTest :
  public CollisionTest<T>{};
using CollisionMeshTestTypes =
  ::testing::Types<CollisionMeshFeaturesList>;
TYPED_TEST_SUITE(CollisionMeshTest,
                 CollisionMeshTestTypes);

TYPED_TEST(CollisionMeshTest, MeshDecomposition)
{
  // Load an optimized mesh, drop it from some height,
  // and verify it collides with the ground plane

  auto getModelOptimizedStr = [](const std::string &_optimization,
                                 const std::string &_name,
                                 const gz::math::Pose3d &_pose)
  {
    std::stringstream modelOptimizedStr;
    modelOptimizedStr << R"(
    <sdf version="1.11">
      <model name=")";
    modelOptimizedStr << _name << R"(">
        <pose>)";
    modelOptimizedStr << _pose;
    modelOptimizedStr << R"(</pose>
        <link name="body">
          <collision name="collision">
            <geometry>
              <mesh optimization=")";
    modelOptimizedStr << _optimization;
    modelOptimizedStr << R"(">
                <uri>)";
    modelOptimizedStr << gz::physics::test::resources::kChassisDae;
    modelOptimizedStr << R"(</uri>
              </mesh>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)";
    return modelOptimizedStr.str();
  };

  for (const std::string &name : this->pluginNames)
  {
    // currently only bullet-featherstone supports mesh decomposition
    if (this->PhysicsEngineName(name) != "bullet-featherstone")
      continue;
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    sdf::Root rootWorld;
    const sdf::Errors errorsWorld =
        rootWorld.Load(common_test::worlds::kGroundSdf);
    ASSERT_TRUE(errorsWorld.empty()) << errorsWorld.front();

    auto engine =
        gz::physics::RequestEngine3d<CollisionMeshFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = engine->ConstructWorld(*rootWorld.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    // load the mesh into mesh manager first to create a cache
    // so the model can be constructed later - needed by bullet-featherstone
    const std::string meshFilename = gz::physics::test::resources::kChassisDae;
    auto &meshManager = *gz::common::MeshManager::Instance();
    ASSERT_NE(nullptr, meshManager.Load(meshFilename));

    std::unordered_set<std::string> optimizations;
    optimizations.insert("");
    optimizations.insert("convex_decomposition");
    optimizations.insert("convex_hull");

    gz::math::Pose3d initialModelPose(0, 0, 2, 0, 0, 0);
    // Test all optimization methods
    for (const auto &optimizationStr : optimizations)
    {
      // create the chassis model
      const std::string modelOptimizedName = "model_optimized_" + optimizationStr;
      sdf::Root root;
      sdf::Errors errors = root.LoadSdfString(getModelOptimizedStr(
          optimizationStr, modelOptimizedName, initialModelPose));
      ASSERT_TRUE(errors.empty()) << errors.front();
      ASSERT_NE(nullptr, root.Model());
      world->ConstructModel(*root.Model());

      const std::string bodyName{"body"};
      auto modelOptimized = world->GetModel(modelOptimizedName);
      auto modelOptimizedBody = modelOptimized->GetLink(bodyName);

      auto frameDataModelOptimizedBody =
          modelOptimizedBody->FrameDataRelativeToWorld();

      EXPECT_EQ(initialModelPose,
                gz::math::eigen3::convert(frameDataModelOptimizedBody.pose));

      // After a while, the mesh model should reach the ground and come to a stop
      gz::physics::ForwardStep::Output output;
      gz::physics::ForwardStep::State state;
      gz::physics::ForwardStep::Input input;
      std::size_t stepCount = 3000u;
      for (unsigned int i = 0; i < stepCount; ++i)
        world->Step(output, state, input);

      frameDataModelOptimizedBody =
          modelOptimizedBody->FrameDataRelativeToWorld();

      // convex decomposition gives more accurate results
      double tol = (optimizationStr == "convex_decomposition") ? 1e-3 : 1e-2;
      EXPECT_NEAR(0.1,
                  frameDataModelOptimizedBody.pose.translation().z(), tol);
      EXPECT_NEAR(0.0, frameDataModelOptimizedBody.linearVelocity.z(), tol);

      initialModelPose.Pos() += gz::math::Vector3d(0, 2, 0);
    }
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!CollisionTest<CollisionFeaturesList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
