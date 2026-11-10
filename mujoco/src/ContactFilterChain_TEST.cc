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
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <mujoco/mujoco.h>

using TestFeatures =
    gz::physics::FeatureList<gz::physics::sdf::ConstructSdfWorld>;
using WorldPtr = gz::physics::World3dPtr<TestFeatures>;

namespace
{
int appFilterCalls = 0;

/// \brief Stands in for an embedding application's filter. Rejects every
/// contact, so its effect on the contact count is easy to see.
int RejectAllFilter(const mjModel *, mjData *, int, int)
{
  ++appFilterCalls;
  return 1;
}

mjfConFilt filterReplacedByApp = nullptr;

/// \brief An application filter that chains to whatever it replaced.
int ChainingFilter(const mjModel *m, mjData *d, int g1, int g2)
{
  ++appFilterCalls;
  return filterReplacedByApp ? filterReplacedByApp(m, d, g1, g2) : 0;
}

/// \brief Two overlapping spheres in a model that gz-physics does not manage.
/// Returns the number of contacts MuJoCo keeps between them.
int ForeignModelContactCount()
{
  const char *xml = R"(
<mujoco>
  <worldbody>
    <body><freejoint/><geom type="sphere" size="0.1"/></body>
    <body><freejoint/><geom type="sphere" size="0.1" pos="0.05 0 0"/></body>
  </worldbody>
</mujoco>)";
  char error[1000] = "";
  mjSpec *spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));
  EXPECT_NE(nullptr, spec) << error;
  if (!spec)
    return -1;
  mjModel *model = mj_compile(spec, nullptr);
  mjData *data = mj_makeData(model);
  mj_forward(model, data);
  const int ncon = data->ncon;
  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
  return ncon;
}
}  // namespace

/////////////////////////////////////////////////
class ContactFilterChainTest : public ::testing::Test
{
  protected: void SetUp() override
  {
    this->loader.LoadLib(mujoco_plugin_LIB);
    this->plugin = this->loader.Instantiate("gz::physics::mujoco::Plugin");
    ASSERT_TRUE(this->plugin);
    this->engine =
        gz::physics::RequestEngine3d<TestFeatures>::From(this->plugin);
    ASSERT_TRUE(this->engine);
    appFilterCalls = 0;
  }

  protected: void TearDown() override
  {
    mjcb_contactfilter = nullptr;
    filterReplacedByApp = nullptr;
  }

  protected: WorldPtr ConstructWorld()
  {
    sdf::Root root;
    EXPECT_TRUE(root.LoadSdfString(R"(
<sdf version="1.6">
  <world name="w">
    <model name="m">
      <link name="l">
        <collision name="c">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>)").empty());
    return this->engine->ConstructWorld(*root.WorldByIndex(0));
  }

  private: gz::plugin::Loader loader;
  private: gz::plugin::PluginPtr plugin;
  private: gz::physics::Engine3dPtr<TestFeatures> engine;
};

/////////////////////////////////////////////////
// Sanity check: without any filter, the foreign spheres collide.
TEST_F(ContactFilterChainTest, ForeignModelCollidesByDefault)
{
  mjcb_contactfilter = nullptr;
  EXPECT_GT(ForeignModelContactCount(), 0);
}

/////////////////////////////////////////////////
// An application's filter must keep deciding contacts for its own models
// after a gz-physics world is constructed.
TEST_F(ContactFilterChainTest, ApplicationFilterStillRuns)
{
  mjcb_contactfilter = RejectAllFilter;

  WorldPtr world = this->ConstructWorld();
  ASSERT_NE(nullptr, world);
  // gz-physics installs its own filter in front of the application's.
  EXPECT_NE(RejectAllFilter, mjcb_contactfilter);

  EXPECT_EQ(0, ForeignModelContactCount());
  EXPECT_GT(appFilterCalls, 0);

  // A second world must not chain gz-physics to itself.
  WorldPtr world2 = this->ConstructWorld();
  ASSERT_NE(nullptr, world2);
  appFilterCalls = 0;
  EXPECT_EQ(0, ForeignModelContactCount());
  EXPECT_GT(appFilterCalls, 0);
}

/////////////////////////////////////////////////
// An application filter that chains back to the gz-physics filter it
// replaced must not recurse forever once gz-physics reinstalls its filter.
TEST_F(ContactFilterChainTest, ChainingBackDoesNotRecurse)
{
  WorldPtr world = this->ConstructWorld();
  ASSERT_NE(nullptr, world);
  filterReplacedByApp = mjcb_contactfilter;
  mjcb_contactfilter = ChainingFilter;

  WorldPtr world2 = this->ConstructWorld();
  ASSERT_NE(nullptr, world2);
  EXPECT_NE(ChainingFilter, mjcb_contactfilter);

  EXPECT_GT(ForeignModelContactCount(), 0);
  EXPECT_GT(appFilterCalls, 0);
}
