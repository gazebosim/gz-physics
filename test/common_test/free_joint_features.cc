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

#include <tuple>

#include "test/TestLibLoader.hh"
#include "test/Utils.hh"
#include "Worlds.hh"

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "gz/physics/FindFeatures.hh"
#include "gz/physics/ForwardStep.hh"
#include "gz/physics/FrameSemantics.hh"
#include "gz/physics/FreeGroup.hh"
#include "gz/physics/GetEntities.hh"
#include "gz/physics/RequestEngine.hh"
#include "gz/physics/sdf/ConstructModel.hh"
#include "gz/physics/sdf/ConstructNestedModel.hh"
#include "gz/physics/sdf/ConstructWorld.hh"

struct BaseTestFeatureList : gz::physics::FeatureList<
    gz::physics::GetEngineInfo,
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::sdf::ConstructSdfModel,
    gz::physics::GetWorldFromEngine,
    gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel,
    gz::physics::LinkFrameSemantics,
    gz::physics::FindFreeGroupFeature,
    gz::physics::SetFreeGroupWorldPose > { };

struct NestedTestFeatureList : gz::physics::FeatureList<
    BaseTestFeatureList,
    gz::physics::sdf::ConstructSdfNestedModel,
    gz::physics::GetNestedModelFromModel > { };

struct VelocityTestFeatureList : gz::physics::FeatureList<
    BaseTestFeatureList,
    gz::physics::ForwardStep,
    gz::physics::SetFreeGroupWorldVelocity > { };

using BaseWorldPtr = gz::physics::World3dPtr<BaseTestFeatureList>;
using BaseModelPtr = gz::physics::Model3dPtr<BaseTestFeatureList>;
using BaseLinkPtr = gz::physics::Link3dPtr<BaseTestFeatureList>;

using NestedWorldPtr = gz::physics::World3dPtr<NestedTestFeatureList>;
using NestedModelPtr = gz::physics::Model3dPtr<NestedTestFeatureList>;
using NestedLinkPtr = gz::physics::Link3dPtr<NestedTestFeatureList>;

template<typename Features>
class FreeGroupFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(FreeGroupFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<Features>::From(loader);
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

using BaseFreeGroupFeaturesTest = FreeGroupFeaturesTest<BaseTestFeatureList>;
using NestedFreeGroupFeaturesTest =
    FreeGroupFeaturesTest<NestedTestFeatureList>;
using VelocityFreeGroupFeaturesTest =
    FreeGroupFeaturesTest<VelocityTestFeatureList>;

NestedModelPtr GetModelFromAbsoluteName(const NestedWorldPtr &_world,
                                        const std::string &_absoluteName)
{
  std::vector<std::string> names =
      gz::common::split(_absoluteName, std::string(sdf::kScopeDelimiter));
  if (names.empty())
  {
    return nullptr;
  }

  auto currentModel = _world->GetModel(names.front());
  for (std::size_t i = 1; i < names.size(); ++i)
  {
    if (nullptr == currentModel)
      return nullptr;
    currentModel = currentModel->GetNestedModel(names[i]);
  }
  return currentModel;
}

TEST_F(NestedFreeGroupFeaturesTest, NestedFreeGroup)
{
  for (const std::string &name : pluginNames)
  {
    // https://github.com/gazebosim/gz-physics/issues/550
    // bullet-feathersone does not support multiple kinematic trees in
    // a model so nested_model2 and nested_model3 are not constructed.
    CHECK_UNSUPPORTED_ENGINE(name, "bullet-featherstone")

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<NestedTestFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors &errors = root.Load(
      common_test::worlds::kWorldWithNestedModelSdf);
    EXPECT_EQ(0u, errors.size()) << errors;

    EXPECT_EQ(1u, root.WorldCount());
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    EXPECT_NE(nullptr, sdfWorld);

    auto world = engine->ConstructWorld(*sdfWorld);
    EXPECT_NE(nullptr, world);

    auto checkFreeGroupForModel = [&](const std::string &_modelName)
    {
      auto model = GetModelFromAbsoluteName(world, _modelName);
      if (nullptr == model)
        return testing::AssertionFailure() << "Model could not be found";
      auto freeGroup = model->FindFreeGroup();
      if (nullptr == freeGroup)
        return testing::AssertionFailure() << "Freegroup could not be found";
      if (nullptr == freeGroup->RootLink())
        return testing::AssertionFailure() << "RootLink could not be found";
      return testing::AssertionSuccess();
    };

    EXPECT_TRUE(checkFreeGroupForModel("parent_model"));
    if (engine->GetName() == "tpe")
    {
      // Expect true because tpe doesn't support joints, the link in
      // nested_model could not be referenced by a joint.
      EXPECT_TRUE(checkFreeGroupForModel("parent_model::nested_model"));
    }
    else
    {
      // Expect false because the link in nested_model is referenced by a joint.
      EXPECT_FALSE(checkFreeGroupForModel("parent_model::nested_model"));
    }

    EXPECT_TRUE(checkFreeGroupForModel("parent_model::nested_model2"));
    EXPECT_TRUE(checkFreeGroupForModel("parent_model::nested_model3"));
  }
}

TEST_F(NestedFreeGroupFeaturesTest, NestedFreeGroupSetWorldPose)
{
  for (const std::string &name : pluginNames)
  {
    // https://github.com/gazebosim/gz-physics/issues/550
    // bullet-feathersone does not support multiple kinematic trees in
    // a model so nested_model2 and nested_model3 are not constructed.
    CHECK_UNSUPPORTED_ENGINE(name, "bullet-featherstone")

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<NestedTestFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors &errors = root.Load(
      common_test::worlds::kWorldWithNestedModelSdf);
    EXPECT_EQ(0u, errors.size()) << errors;

    EXPECT_EQ(1u, root.WorldCount());
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    EXPECT_NE(nullptr, sdfWorld);

    auto world = engine->ConstructWorld(*sdfWorld);
    EXPECT_NE(nullptr, world);

    gz::math::Pose3d parentModelNewPose(0, 0, 2, 0, 0, 0);
    {
      auto parentModel = world->GetModel("parent_model");
      ASSERT_NE(nullptr, parentModel);
      auto freeGroup = parentModel->FindFreeGroup();
      ASSERT_NE(nullptr, freeGroup);

      freeGroup->SetWorldPose(
          gz::math::eigen3::convert(parentModelNewPose));

      auto link1 = parentModel->GetLink("link1");
      ASSERT_NE(nullptr, link1);
      auto frameData = link1->FrameDataRelativeToWorld();
      EXPECT_EQ(parentModelNewPose,
                gz::math::eigen3::convert(frameData.pose));
    }
    {
      auto nestedModel =
          GetModelFromAbsoluteName(world, "parent_model::nested_model");
      ASSERT_NE(nullptr, nestedModel);
      auto nestedLink1 = nestedModel->GetLink("nested_link1");
      ASSERT_NE(nullptr, nestedLink1);
      auto frameData = nestedLink1->FrameDataRelativeToWorld();
      // Poses from SDF
      gz::math::Pose3d nestedModelPose(1, 2, 2, 0, 0, 0);
      gz::math::Pose3d nestedLinkPose(3, 1, 1, 0, 0, GZ_PI_2);

      gz::math::Pose3d nestedLinkExpectedPose =
          parentModelNewPose * nestedModelPose * nestedLinkPose;

      EXPECT_EQ(nestedLinkExpectedPose,
          gz::math::eigen3::convert(frameData.pose));
    }
    {
      auto parentModel = world->GetModel("parent_model2");
      ASSERT_NE(nullptr, parentModel);
      auto freeGroup = parentModel->FindFreeGroup();
      ASSERT_NE(nullptr, freeGroup);

      freeGroup->SetWorldPose(
          gz::math::eigen3::convert(parentModelNewPose));

      auto grandChild = GetModelFromAbsoluteName(
          world, "parent_model2::child_model::grand_child_model");
      ASSERT_NE(nullptr, grandChild);
      auto link1 = grandChild->GetLink("link1");
      ASSERT_NE(nullptr, link1);
      auto frameData = link1->FrameDataRelativeToWorld();
      EXPECT_EQ(parentModelNewPose,
                gz::math::eigen3::convert(frameData.pose));
    }
  }
}

TEST_F(NestedFreeGroupFeaturesTest, FreeGroupNestedNoLink)
{
  const std::string modelStr = R"(
    <sdf version="1.11">
      <model name="box">
        <static>true</static>
        <pose>1 2 3.0 0 0 0</pose>
        <model name="nested_box">
          <static>true</static>
        </model>
      </model>
    </sdf>)";

  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<NestedTestFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(
      common_test::worlds::kGroundSdf);
    EXPECT_EQ(0u, errors.size()) << errors;

    EXPECT_EQ(1u, root.WorldCount());
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    ASSERT_NE(nullptr, sdfWorld);

    auto world = engine->ConstructWorld(*sdfWorld);
    ASSERT_NE(nullptr, world);

    // Create the model with no links
    sdf::Root rootNested;
    sdf::Errors errorsNested = rootNested.LoadSdfString(modelStr);
    EXPECT_TRUE(errorsNested.empty()) << errorsNested;
    ASSERT_NE(nullptr, rootNested.Model());
    world->ConstructModel(*rootNested.Model());

    auto model = world->GetModel("box");
    // bullet-featherstone strictly does not allow models with no links.
    if (this->PhysicsEngineName(name) == "bullet-featherstone")
    {
      EXPECT_EQ(nullptr, model);
    }
    else
    {
      EXPECT_NE(nullptr, model);

      // Free group should not be available
      auto freeGroup = model->FindFreeGroup();
      EXPECT_EQ(nullptr, freeGroup);
    }
  }
}

TEST_F(BaseFreeGroupFeaturesTest, FreeGroupSetWorldPosePrincipalAxesOffset)
{
  const std::string modelStr = R"(
    <sdf version="1.11">
      <model name="box">
        <pose>1 2 3.0 0 0 0</pose>
        <link name="link">
          <inertial>
            <pose>-0.00637 -0.008 0.13254 0 0 0</pose>
            <inertia>
              <ixx>0.01331127</ixx>
              <ixy>-0.00030365</ixy>
              <ixz>-0.00034148</ixz>
              <iyy>0.01157659</iyy>
              <iyz>0.00088073</iyz>
              <izz>0.00378028</izz>
            </inertia>
            <mass>1.50251902</mass>
          </inertial>
          <collision name="coll_sphere">
            <geometry>
              <sphere>
                <radius>0.1</radius>
              </sphere>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)";

  for (const std::string &name : pluginNames)
  {
    CHECK_UNSUPPORTED_ENGINE(name, "bullet")
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<BaseTestFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(
      common_test::worlds::kGroundSdf);
    EXPECT_EQ(0u, errors.size()) << errors;

    EXPECT_EQ(1u, root.WorldCount());
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    ASSERT_NE(nullptr, sdfWorld);

    auto world = engine->ConstructWorld(*sdfWorld);
    ASSERT_NE(nullptr, world);

    // create the model
    errors = root.LoadSdfString(modelStr);
    ASSERT_TRUE(errors.empty()) << errors;
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    auto model = world->GetModel("box");
    ASSERT_NE(nullptr, model);
    auto link = model->GetLink("link");
    ASSERT_NE(nullptr, link);
    auto frameDataLink = link->FrameDataRelativeToWorld();
    EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0),
              gz::math::eigen3::convert(frameDataLink.pose));

    // get free group and set new pose
    auto freeGroup = model->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);
    gz::math::Pose3d newPose(4, 5, 6, 0, 0, 1.57);
    freeGroup->SetWorldPose(
        gz::math::eigen3::convert(newPose));
    frameDataLink = link->FrameDataRelativeToWorld();
    EXPECT_EQ(newPose,
              gz::math::eigen3::convert(frameDataLink.pose));
  }
}

TEST_F(BaseFreeGroupFeaturesTest, FreeGroupSetWorldPoseStaticAndFixedModel)
{
  const std::string modelStaticStr = R"(
    <sdf version="1.11">
      <model name="sphere">
        <static>true</static>
        <pose>1 2 3.0 0 0 0</pose>
        <link name="link">
          <collision name="coll_sphere">
            <geometry>
              <sphere>
                <radius>0.1</radius>
              </sphere>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)";

  const std::string modelWorldFixedStr = R"(
    <sdf version="1.11">
      <model name="sphere_world_fixed">
        <static>false</static>
        <pose>3 2 3.0 0 0 0</pose>
        <link name="link">
          <collision name="coll_sphere">
            <geometry>
              <sphere>
                <radius>0.1</radius>
              </sphere>
            </geometry>
          </collision>
        </link>
        <joint name="world_fixed" type="fixed">
          <parent>world</parent>
          <child>link</child>
        </joint>
      </model>
    </sdf>)";

  for (const std::string &name : pluginNames)
  {
    CHECK_UNSUPPORTED_ENGINE(name, "bullet")
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<BaseTestFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(
      common_test::worlds::kGroundSdf);
    EXPECT_EQ(0u, errors.size()) << errors;

    EXPECT_EQ(1u, root.WorldCount());
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    ASSERT_NE(nullptr, sdfWorld);

    auto world = engine->ConstructWorld(*sdfWorld);
    ASSERT_NE(nullptr, world);

    // create the static model
    errors = root.LoadSdfString(modelStaticStr);
    ASSERT_TRUE(errors.empty()) << errors;
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    auto modelStatic = world->GetModel("sphere");
    ASSERT_NE(nullptr, modelStatic);
    auto link = modelStatic->GetLink("link");
    ASSERT_NE(nullptr, link);
    auto frameDataLink = link->FrameDataRelativeToWorld();
    EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0),
              gz::math::eigen3::convert(frameDataLink.pose));

    // get free group and set new pose
    auto freeGroup = modelStatic->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);
    gz::math::Pose3d newPose(4, 5, 6, 0, 0, 1.57);
    freeGroup->SetWorldPose(
        gz::math::eigen3::convert(newPose));
    frameDataLink = link->FrameDataRelativeToWorld();
    // static model should move to new pose
    EXPECT_EQ(newPose,
              gz::math::eigen3::convert(frameDataLink.pose));

    // create the model with world joint
    errors = root.LoadSdfString(modelWorldFixedStr);
    ASSERT_TRUE(errors.empty()) << errors;
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    // Verify static model pose is preserved after a new model is constructed
    EXPECT_EQ(newPose,
              gz::math::eigen3::convert(
                  modelStatic->GetLink("link")->FrameDataRelativeToWorld().pose));

    auto modelFixed = world->GetModel("sphere_world_fixed");
    ASSERT_NE(nullptr, modelFixed);
    link = modelFixed->GetLink("link");
    ASSERT_NE(nullptr, link);
    frameDataLink = link->FrameDataRelativeToWorld();
    gz::math::Pose3d origPose(3, 2, 3, 0, 0, 0);
    EXPECT_EQ(origPose,
              gz::math::eigen3::convert(frameDataLink.pose));

    // get free group and set new pose
    freeGroup = modelFixed->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);
    freeGroup->SetWorldPose(
        gz::math::eigen3::convert(newPose));
    frameDataLink = link->FrameDataRelativeToWorld();
    // world fixed model should not be able to move to new pose
    if (this->PhysicsEngineName(name) != "tpe")
    {
      EXPECT_EQ(origPose,
                gz::math::eigen3::convert(frameDataLink.pose));
    }
  }
}

TEST_F(VelocityFreeGroupFeaturesTest, FreeGroupSetWorldVelocityCoMOffset)
{
  // The velocity of a free group is the velocity of its canonical link
  // frame. The center of mass of the link below is offset from the link
  // frame, so the engine has to convert between the velocity of the link
  // frame and the velocity of the center of mass when the link rotates.
  const std::string modelStr = R"(
    <sdf version="1.11">
      <model name="offset">
        <pose>5 0 2 0 0 0</pose>
        <link name="link">
          <inertial>
            <pose>0.5 0 0 0 0 0</pose>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.01</ixx>
              <iyy>0.01</iyy>
              <izz>0.01</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <sphere>
                <radius>0.1</radius>
              </sphere>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)";
  // Center of mass in the link frame
  const Eigen::Vector3d comOffset(0.5, 0, 0);
  // Model pose, away from the sphere of sphere.sdf
  const Eigen::Vector3d linkPos(5, 0, 2);

  for (const std::string &name : pluginNames)
  {
    // bullet sets the velocity of the center of mass and tpe has no notion
    // of a center of mass.
    CHECK_UNSUPPORTED_ENGINE(name, "bullet", "tpe")
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<VelocityTestFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    // sphere.sdf has no gravity
    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kSphereSdf);
    EXPECT_EQ(0u, errors.size()) << errors;
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    ASSERT_NE(nullptr, sdfWorld);
    auto world = engine->ConstructWorld(*sdfWorld);
    ASSERT_NE(nullptr, world);

    errors = root.LoadSdfString(modelStr);
    ASSERT_TRUE(errors.empty()) << errors;
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    auto model = world->GetModel("offset");
    ASSERT_NE(nullptr, model);
    auto link = model->GetLink("link");
    ASSERT_NE(nullptr, link);
    auto freeGroup = model->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);

    const Eigen::Vector3d linVel(1, 0, 0);
    const Eigen::Vector3d angVel(0, 0, 1);
    freeGroup->SetWorldAngularVelocity(angVel);
    freeGroup->SetWorldLinearVelocity(linVel);
    auto frameData = link->FrameDataRelativeToWorld();
    EXPECT_TRUE(gz::physics::test::Equal(linVel, frameData.linearVelocity,
        1e-6));
    EXPECT_TRUE(gz::physics::test::Equal(angVel, frameData.angularVelocity,
        1e-6));

    // Setting the angular velocity keeps the linear velocity of the link
    // frame, in both orders of the calls.
    const Eigen::Vector3d angVel2(0, 0, 2);
    freeGroup->SetWorldAngularVelocity(angVel2);
    frameData = link->FrameDataRelativeToWorld();
    EXPECT_TRUE(gz::physics::test::Equal(linVel, frameData.linearVelocity,
        1e-6));
    EXPECT_TRUE(gz::physics::test::Equal(angVel2, frameData.angularVelocity,
        1e-6));
    freeGroup->SetWorldLinearVelocity(linVel);
    freeGroup->SetWorldAngularVelocity(angVel);
    frameData = link->FrameDataRelativeToWorld();
    EXPECT_TRUE(gz::physics::test::Equal(linVel, frameData.linearVelocity,
        1e-6));
    EXPECT_TRUE(gz::physics::test::Equal(angVel, frameData.angularVelocity,
        1e-6));

    // Without gravity the center of mass keeps its velocity
    // v_com = v_link + w x (p_com - p_link) and the link keeps spinning
    // about a principal axis, so the link frame orbits the center of mass
    // with the velocity v_com + w x (p_link - p_com).
    const Eigen::Vector3d comVel = linVel + angVel.cross(comOffset);
    const std::size_t steps = 500;
    const double time = steps * 0.001;
    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;
    for (std::size_t i = 0; i < steps; ++i)
      world->Step(output, state, input);

    frameData = link->FrameDataRelativeToWorld();
    const Eigen::Vector3d linkToCom = frameData.pose.linear() * comOffset;
    const Eigen::Vector3d expectedLinVel = comVel - angVel.cross(linkToCom);
    EXPECT_TRUE(gz::physics::test::Equal(expectedLinVel,
        frameData.linearVelocity, 1e-2));
    EXPECT_TRUE(gz::physics::test::Equal(angVel, frameData.angularVelocity,
        1e-2));
    // The center of mass moved in a straight line while the link yawed.
    const Eigen::Vector3d expectedComPos =
        linkPos + comOffset + comVel * time;
    const Eigen::Vector3d comPos = frameData.pose.translation() + linkToCom;
    EXPECT_TRUE(gz::physics::test::Equal(expectedComPos, comPos, 1e-2));
    EXPECT_NEAR(angVel.z() * time,
        gz::math::eigen3::convert(frameData.pose).Rot().Yaw(), 1e-2);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  BaseFreeGroupFeaturesTest::init(argc, argv);
  NestedFreeGroupFeaturesTest::init(argc, argv);
  VelocityFreeGroupFeaturesTest::init(argc, argv);
  return RUN_ALL_TESTS();
}
