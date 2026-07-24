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

#include <string>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/plugin/Loader.hh>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <mujoco/mujoco.h>

#include "Base.hh"

using gz::physics::mujoco::ComputeWeldExclusions;
using gz::physics::mujoco::WorldInfo;

struct TestFeatures: public gz::physics::FeatureList<
    gz::physics::AttachFixedJointFeature,
    gz::physics::DetachJointFeature,
    gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel,
    gz::physics::sdf::ConstructSdfWorld>
{
};

using WorldPtr = gz::physics::World3dPtr<TestFeatures>;

/////////////////////////////////////////////////
TEST(WeldExclusionsTest, DynamicWeldConnectedComponents)
{
  const std::string sdfString = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="weld_exclusions_world">
    <!-- Tree 1 -->
    <model name="M1">
      <link name="link1A">
        <collision name="c1">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
        </collision>
      </link>
      <joint name="j1B" type="fixed">
        <parent>link1A</parent>
        <child>link1B</child>
      </joint>
      <link name="link1B">
        <collision name="c1">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
        </collision>
      </link>
      <joint name="j1C" type="revolute">
        <parent>link1B</parent>
        <child>link1C</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <link name="link1C">
        <collision name="c1">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
        </collision>
      </link>
    </model>

    <!-- Tree 2 -->
    <model name="M2">
      <link name="link2A">
        <collision name="c1">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
        </collision>
      </link>
      <joint name="j2B" type="fixed">
        <parent>link2A</parent>
        <child>link2B</child>
      </joint>
      <link name="link2B">
        <collision name="c1">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
)";

  gz::plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);

  gz::plugin::PluginPtr mujoco =
      loader.Instantiate("gz::physics::mujoco::Plugin");
  ASSERT_TRUE(mujoco);

  auto engine = gz::physics::RequestEngine3d<TestFeatures>::From(mujoco);
  ASSERT_TRUE(engine);

  sdf::Root root;
  const sdf::Errors errors = root.LoadSdfString(sdfString);
  ASSERT_TRUE(errors.empty());

  const sdf::World *sdfWorld = root.WorldByIndex(0);
  ASSERT_NE(nullptr, sdfWorld);

  WorldPtr world = engine->ConstructWorld(*sdfWorld);
  ASSERT_NE(nullptr, world);

  auto *worldInfo = static_cast<WorldInfo *>(
      world->FullIdentity().ref.get());
  ASSERT_NE(nullptr, worldInfo);
  ASSERT_NE(nullptr, worldInfo->mjModelObj);
  ASSERT_NE(nullptr, worldInfo->mjDataObj);

  auto m1 = world->GetModel("M1");
  ASSERT_NE(nullptr, m1);
  auto m2 = world->GetModel("M2");
  ASSERT_NE(nullptr, m2);

  auto link1A = m1->GetLink("link1A");
  auto link1B = m1->GetLink("link1B");
  auto link1C = m1->GetLink("link1C");
  auto link2A = m2->GetLink("link2A");
  auto link2B = m2->GetLink("link2B");
  ASSERT_TRUE(link1A && link1B && link1C && link2A && link2B);

  // Attach M2::link2B to M1::link1B using a dynamic fixed joint
  auto fixedJoint = link2B->AttachFixedJoint(link1B);
  ASSERT_NE(nullptr, fixedJoint);

  int b1A = mj_name2id(worldInfo->mjModelObj, mjOBJ_BODY, "M1::link1A");
  int b1B = mj_name2id(worldInfo->mjModelObj, mjOBJ_BODY, "M1::link1B");
  int b1C = mj_name2id(worldInfo->mjModelObj, mjOBJ_BODY, "M1::link1C");
  int b2A = mj_name2id(worldInfo->mjModelObj, mjOBJ_BODY, "M2::link2A");
  int b2B = mj_name2id(worldInfo->mjModelObj, mjOBJ_BODY, "M2::link2B");

  int w1A = worldInfo->mjModelObj->body_weldid[b1A];
  int w1B = worldInfo->mjModelObj->body_weldid[b1B];
  int w1C = worldInfo->mjModelObj->body_weldid[b1C];
  int w2A = worldInfo->mjModelObj->body_weldid[b2A];
  int w2B = worldInfo->mjModelObj->body_weldid[b2B];

  // Verify MuJoCo's native body_weldid mapping:
  // Fixed child link1B has the same weldid as its parent link1A.
  EXPECT_EQ(w1A, w1B);
  EXPECT_EQ(w2A, w2B);
  // Non-fixed child link1C has a unique weldid (terminates rigid tree).
  EXPECT_NE(w1A, w1C);

  auto recompile = [&]() {
    mj_recompile(worldInfo->mjSpecObj, nullptr, worldInfo->mjModelObj,
                 worldInfo->mjDataObj);
  };

  // When dynamic weld constraint is active:
  // Tree 1 (link1A, link1B) and Tree 2 (link2A, link2B) are merged into one.
  recompile();
  auto clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);
  EXPECT_NE(-1, clusterMap[w1A]);
  EXPECT_EQ(clusterMap[w1A], clusterMap[w2A]);
  // Non-fixed body link1C is NOT part of the dynamic weld cluster.
  EXPECT_EQ(-1, clusterMap[w1C]);

  // Detach dynamic weld constraint:
  fixedJoint->Detach();
  recompile();
  clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);

  // Cross-tree bodies are no longer in an active dynamic cluster.
  EXPECT_EQ(-1, clusterMap[w1A]);
  EXPECT_EQ(-1, clusterMap[w2A]);
}
