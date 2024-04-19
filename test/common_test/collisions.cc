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

#include <sstream>
#include <string>

#include <gz/common/Console.hh>
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
#include <gz/physics/GetContacts.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/mesh/MeshShape.hh>
#include <gz/physics/PlaneShape.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <gz/common/MeshManager.hh>

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

using CollisionStaticFeaturesList = gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::GetContactsFromLastStepFeature,
  gz::physics::ForwardStep
>;

using CollisionStaticTestFeaturesList =
  CollisionTest<CollisionStaticFeaturesList>;

TEST_F(CollisionStaticTestFeaturesList, StaticCollisions)
{
  auto getBoxStaticStr = [](const std::string &_name,
                            const gz::math::Pose3d &_pose)
  {
    std::stringstream modelStaticStr;
    modelStaticStr << R"(
    <sdf version="1.11">
      <model name=")";
    modelStaticStr << _name << R"(">
        <pose>)";
    modelStaticStr << _pose;
    modelStaticStr << R"(</pose>
        <link name="body">
          <collision name="collision">
            <geometry>
              <box><size>1 1 1</size></box>
            </geometry>
          </collision>
        </link>
        <static>true</static>
      </model>
    </sdf>)";
    return modelStaticStr.str();
  };

  auto getBoxFixedJointStr = [](const std::string &_name,
                                const gz::math::Pose3d &_pose)
  {
    std::stringstream modelFixedJointStr;
    modelFixedJointStr << R"(
    <sdf version="1.11">
      <model name=")";
    modelFixedJointStr << _name << R"(">
        <pose>)";
    modelFixedJointStr << _pose;
    modelFixedJointStr << R"(</pose>
        <link name="body">
          <collision name="collision">
            <geometry>
              <box><size>1 1 1</size></box>
            </geometry>
          </collision>
        </link>
        <joint name="world_fixed" type="fixed">
          <parent>world</parent>
          <child>body</child>
        </joint>
      </model>
    </sdf>)";
    return modelFixedJointStr.str();
  };

  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    sdf::Root rootWorld;
    const sdf::Errors errorsWorld =
        rootWorld.Load(common_test::worlds::kGroundSdf);
    ASSERT_TRUE(errorsWorld.empty()) << errorsWorld.front();

    auto engine =
        gz::physics::RequestEngine3d<CollisionStaticFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = engine->ConstructWorld(*rootWorld.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(getBoxStaticStr(
        "box_static", gz::math::Pose3d::Zero));
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;
    for (std::size_t i = 0; i < 10; ++i)
    {
      world->Step(output, state, input);
    }

    // static box overlaps with ground plane
    // verify no contacts between static bodies.
    auto contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(0u, contacts.size());

    // currently only bullet-featherstone skips collision checking between
    // static bodies and bodies with world fixed joint
    if (this->PhysicsEngineName(name) != "bullet-featherstone")
      continue;

    errors = root.LoadSdfString(getBoxFixedJointStr(
        "box_fixed_world_joint", gz::math::Pose3d::Zero));
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    world->Step(output, state, input);
    // box fixed to world overlaps with static box and ground plane
    // verify there are still no contacts.
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(0u, contacts.size());
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
