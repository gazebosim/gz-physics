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
class WeldExclusionsTest : public ::testing::Test
{
  protected: void SetUp() override
  {
    this->loader.LoadLib(mujoco_plugin_LIB);
    this->plugin = this->loader.Instantiate("gz::physics::mujoco::Plugin");
    ASSERT_TRUE(this->plugin);
    this->engine =
        gz::physics::RequestEngine3d<TestFeatures>::From(this->plugin);
    ASSERT_TRUE(this->engine);
  }

  protected: WorldPtr LoadWorld(const std::string &_sdfString,
                                WorldInfo **_worldInfo)
  {
    sdf::Root root;
    if (!root.LoadSdfString(_sdfString).empty())
      return nullptr;

    const sdf::World *sdfWorld = root.WorldByIndex(0);
    if (!sdfWorld)
      return nullptr;

    WorldPtr world = this->engine->ConstructWorld(*sdfWorld);
    if (!world)
      return nullptr;

    if (_worldInfo)
    {
      *_worldInfo =
          static_cast<WorldInfo *>(world->FullIdentity().ref.get());
    }
    return world;
  }

  protected: gz::physics::Link3dPtr<TestFeatures> GetLink(
      const WorldPtr &_world, const std::string &_scopedName)
  {
    std::size_t pos = _scopedName.find("::");
    if (pos == std::string::npos)
      return nullptr;
    auto model = _world->GetModel(_scopedName.substr(0, pos));
    if (!model)
      return nullptr;
    return model->GetLink(_scopedName.substr(pos + 2));
  }

  protected: int GetWeldId(WorldInfo *_worldInfo, const std::string &_bodyName)
  {
    int b = mj_name2id(_worldInfo->mjModelObj, mjOBJ_BODY, _bodyName.c_str());
    if (b < 0 || b >= _worldInfo->mjModelObj->nbody)
      return -1;
    return _worldInfo->mjModelObj->body_weldid[b];
  }

  protected: void Recompile(WorldInfo *_worldInfo)
  {
    mj_recompile(_worldInfo->mjSpecObj, nullptr, _worldInfo->mjModelObj,
                 _worldInfo->mjDataObj);
  }

  private: gz::plugin::Loader loader;
  private: gz::plugin::PluginPtr plugin;
  private: gz::physics::Engine3dPtr<TestFeatures> engine;
};

/////////////////////////////////////////////////
TEST_F(WeldExclusionsTest, DynamicWeldConnectedComponents)
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

  WorldInfo *worldInfo = nullptr;
  WorldPtr world = this->LoadWorld(sdfString, &worldInfo);
  ASSERT_NE(nullptr, world);
  ASSERT_NE(nullptr, worldInfo);

  auto link1B = this->GetLink(world, "M1::link1B");
  auto link2B = this->GetLink(world, "M2::link2B");
  ASSERT_TRUE(link1B && link2B);

  // Attach M2::link2B to M1::link1B using a dynamic fixed joint
  auto fixedJoint = link2B->AttachFixedJoint(link1B);
  ASSERT_NE(nullptr, fixedJoint);

  int w1A = this->GetWeldId(worldInfo, "M1::link1A");
  int w1B = this->GetWeldId(worldInfo, "M1::link1B");
  int w1C = this->GetWeldId(worldInfo, "M1::link1C");
  int w2A = this->GetWeldId(worldInfo, "M2::link2A");
  int w2B = this->GetWeldId(worldInfo, "M2::link2B");

  // Verify MuJoCo's native body_weldid mapping:
  // Fixed child link1B has the same weldid as its parent link1A.
  EXPECT_EQ(w1A, w1B);
  EXPECT_EQ(w2A, w2B);
  // Non-fixed child link1C has a unique weldid (terminates rigid tree).
  EXPECT_NE(w1A, w1C);

  // When dynamic weld constraint is active:
  // Tree 1 (link1A, link1B) and Tree 2 (link2A, link2B) are merged into one.
  this->Recompile(worldInfo);
  auto clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);
  EXPECT_NE(-1, clusterMap[w1A]);
  EXPECT_EQ(clusterMap[w1A], clusterMap[w2A]);
  // Non-fixed body link1C is NOT part of the dynamic weld cluster.
  EXPECT_EQ(-1, clusterMap[w1C]);

  // Detach dynamic weld constraint:
  fixedJoint->Detach();
  this->Recompile(worldInfo);
  clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);

  // Cross-tree bodies are no longer in an active dynamic cluster.
  EXPECT_EQ(-1, clusterMap[w1A]);
  EXPECT_EQ(-1, clusterMap[w2A]);
}

/////////////////////////////////////////////////
TEST_F(WeldExclusionsTest, DynamicWeldMultipleClusters)
{
  const std::string sdfString = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="multiple_clusters_world">
    <model name="MA"><link name="lA"/></model>
    <model name="MB"><link name="lB"/></model>
    <model name="MC"><link name="lC"/></model>
    <model name="MD"><link name="lD"/></model>
  </world>
</sdf>
)";

  WorldInfo *worldInfo = nullptr;
  WorldPtr world = this->LoadWorld(sdfString, &worldInfo);
  ASSERT_NE(nullptr, world);
  ASSERT_NE(nullptr, worldInfo);

  auto linkA = this->GetLink(world, "MA::lA");
  auto linkB = this->GetLink(world, "MB::lB");
  auto linkC = this->GetLink(world, "MC::lC");
  auto linkD = this->GetLink(world, "MD::lD");
  ASSERT_TRUE(linkA && linkB && linkC && linkD);

  // Cluster 1: A attached to B
  linkA->AttachFixedJoint(linkB);
  // Cluster 2: C attached to D
  linkC->AttachFixedJoint(linkD);

  this->Recompile(worldInfo);

  int wA = this->GetWeldId(worldInfo, "MA::lA");
  int wB = this->GetWeldId(worldInfo, "MB::lB");
  int wC = this->GetWeldId(worldInfo, "MC::lC");
  int wD = this->GetWeldId(worldInfo, "MD::lD");

  auto clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);

  // Cluster 1: A and B are in the same cluster
  EXPECT_NE(-1, clusterMap[wA]);
  EXPECT_EQ(clusterMap[wA], clusterMap[wB]);

  // Cluster 2: C and D are in the same cluster
  EXPECT_NE(-1, clusterMap[wC]);
  EXPECT_EQ(clusterMap[wC], clusterMap[wD]);

  // Cluster 1 and Cluster 2 are distinct
  EXPECT_NE(clusterMap[wA], clusterMap[wC]);
}

/////////////////////////////////////////////////
TEST_F(WeldExclusionsTest, DynamicWeldChainedClusters)
{
  const std::string sdfString = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="chained_clusters_world">
    <model name="MA"><link name="lA"/></model>
    <model name="MB"><link name="lB"/></model>
    <model name="MC"><link name="lC"/></model>
  </world>
</sdf>
)";

  WorldInfo *worldInfo = nullptr;
  WorldPtr world = this->LoadWorld(sdfString, &worldInfo);
  ASSERT_NE(nullptr, world);
  ASSERT_NE(nullptr, worldInfo);

  auto linkA = this->GetLink(world, "MA::lA");
  auto linkB = this->GetLink(world, "MB::lB");
  auto linkC = this->GetLink(world, "MC::lC");
  ASSERT_TRUE(linkA && linkB && linkC);

  // Chain: A -> B -> C
  linkA->AttachFixedJoint(linkB);
  linkB->AttachFixedJoint(linkC);

  this->Recompile(worldInfo);

  int wA = this->GetWeldId(worldInfo, "MA::lA");
  int wB = this->GetWeldId(worldInfo, "MB::lB");
  int wC = this->GetWeldId(worldInfo, "MC::lC");

  auto clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);

  // All 3 models should belong to the exact same cluster
  EXPECT_NE(-1, clusterMap[wA]);
  EXPECT_EQ(clusterMap[wA], clusterMap[wB]);
  EXPECT_EQ(clusterMap[wB], clusterMap[wC]);
}

/////////////////////////////////////////////////
TEST_F(WeldExclusionsTest, DynamicWeldLoops)
{
  const std::string sdfString = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="loop_clusters_world">
    <model name="MA"><link name="lA"/></model>
    <model name="MB"><link name="lB"/></model>
    <model name="MC"><link name="lC"/></model>
  </world>
</sdf>
)";

  WorldInfo *worldInfo = nullptr;
  WorldPtr world = this->LoadWorld(sdfString, &worldInfo);
  ASSERT_NE(nullptr, world);
  ASSERT_NE(nullptr, worldInfo);

  auto linkA = this->GetLink(world, "MA::lA");
  auto linkB = this->GetLink(world, "MB::lB");
  auto linkC = this->GetLink(world, "MC::lC");
  ASSERT_TRUE(linkA && linkB && linkC);

  // Loop: A -> B -> C -> A
  auto jAB = linkA->AttachFixedJoint(linkB);
  auto jBC = linkB->AttachFixedJoint(linkC);
  auto jCA = linkC->AttachFixedJoint(linkA);

  this->Recompile(worldInfo);

  int wA = this->GetWeldId(worldInfo, "MA::lA");
  int wB = this->GetWeldId(worldInfo, "MB::lB");
  int wC = this->GetWeldId(worldInfo, "MC::lC");

  auto clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);

  // All 3 models in the loop share the same cluster
  EXPECT_NE(-1, clusterMap[wA]);
  EXPECT_EQ(clusterMap[wA], clusterMap[wB]);
  EXPECT_EQ(clusterMap[wB], clusterMap[wC]);

  // Break the loop (detach C->A).
  // The remaining chain A->B->C keeps all 3 connected.
  jCA->Detach();
  this->Recompile(worldInfo);
  clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);

  EXPECT_NE(-1, clusterMap[wA]);
  EXPECT_EQ(clusterMap[wA], clusterMap[wB]);
  EXPECT_EQ(clusterMap[wB], clusterMap[wC]);

  // Detach remaining joints. Now no connections remain.
  jBC->Detach();
  jAB->Detach();
  this->Recompile(worldInfo);
  clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);

  EXPECT_EQ(-1, clusterMap[wA]);
  EXPECT_EQ(-1, clusterMap[wB]);
  EXPECT_EQ(-1, clusterMap[wC]);
}

/////////////////////////////////////////////////
TEST_F(WeldExclusionsTest, DynamicWeldWorldBody)
{
  const std::string sdfString = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="world_body_weld_world">
    <model name="MA"><link name="lA"/></model>
    <model name="MB"><link name="lB"/></model>
  </world>
</sdf>
)";

  WorldInfo *worldInfo = nullptr;
  WorldPtr world = this->LoadWorld(sdfString, &worldInfo);
  ASSERT_NE(nullptr, world);
  ASSERT_NE(nullptr, worldInfo);

  auto linkA = this->GetLink(world, "MA::lA");
  auto linkB = this->GetLink(world, "MB::lB");
  ASSERT_TRUE(linkA && linkB);

  // Attach linkA and linkB to worldBody (nullptr parent)
  linkA->AttachFixedJoint(nullptr);
  linkB->AttachFixedJoint(nullptr);

  this->Recompile(worldInfo);

  int wA = this->GetWeldId(worldInfo, "MA::lA");
  int wB = this->GetWeldId(worldInfo, "MB::lB");
  int wWorld = worldInfo->mjModelObj->body_weldid[0];  // worldBody

  auto clusterMap =
      ComputeWeldExclusions(worldInfo->mjModelObj, worldInfo->mjDataObj);

  // Both models attached to world should belong to the same world cluster
  EXPECT_NE(-1, clusterMap[wA]);
  EXPECT_EQ(clusterMap[wA], clusterMap[wB]);
  EXPECT_EQ(clusterMap[wA], clusterMap[wWorld]);
}
