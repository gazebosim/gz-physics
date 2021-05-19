/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/plugin/Loader.hh>

#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/RequestEngine.hh>

#include <ignition/physics/sdf/ConstructJoint.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/Utils.hh>

struct TestFeatureList : ignition::physics::FeatureList<
    ignition::physics::GetBasicJointState,
    ignition::physics::SetBasicJointState,
    ignition::physics::sdf::ConstructSdfJoint,
    ignition::physics::sdf::ConstructSdfLink,
    ignition::physics::sdf::ConstructSdfModel,
    ignition::physics::sdf::ConstructSdfWorld
> { };

using World = ignition::physics::World3d<TestFeatureList>;
using WorldPtr = ignition::physics::World3dPtr<TestFeatureList>;

auto LoadEngine()
{
  ignition::plugin::Loader loader;
  loader.LoadLib(bullet_plugin_LIB);

  ignition::plugin::PluginPtr bullet =
      loader.Instantiate("ignition::physics::bullet::Plugin");

  auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(bullet);
  return engine;
}

// Create Model with parent and child links. If a link is not set, the joint
// will use the world as that link.
auto CreateTestModel(WorldPtr _world, const std::string &_model,
                     const std::optional<sdf::Link> &_parentLink,
                     const std::optional<sdf::Link> &_childLink)
{
  sdf::Model sdfModel;
  sdfModel.SetName(_model);
  auto model = _world->ConstructModel(sdfModel);
  EXPECT_NE(nullptr, model);

  sdf::Joint sdfJoint;
  sdfJoint.SetName("joint0");
  sdfJoint.SetType(sdf::JointType::REVOLUTE);
  if (_parentLink)
  {
    auto parent = model->ConstructLink(*_parentLink);
    EXPECT_NE(nullptr, parent);
    sdfJoint.SetParentLinkName(_parentLink->Name());
  }
  else
  {
    sdfJoint.SetParentLinkName("world");
  }

  if (_childLink)
  {
    auto child = model->ConstructLink(*_childLink);
    EXPECT_NE(nullptr, child);
    sdfJoint.SetChildLinkName(_childLink->Name());
  }
  else
  {
    sdfJoint.SetChildLinkName("world");
  }

  auto joint0 = model->ConstructJoint(sdfJoint);
  return std::make_tuple(model, joint0);
}

// Test joints with world as parent or child
TEST(SDFFeatures_TEST, WorldIsParentOrChild)
{
  auto engine = LoadEngine();
  ASSERT_NE(nullptr, engine);
  sdf::World sdfWorld;
  sdfWorld.SetName("default");
  auto world = engine->ConstructWorld(sdfWorld);
  EXPECT_NE(nullptr, world);

  std::optional<sdf::Link> parent = sdf::Link();
  parent->SetName("parent");
  std::optional<sdf::Link> child = sdf::Link();
  child->SetName("child");

  {
    const auto &[model, joint] =
        CreateTestModel(world, "test0", std::nullopt, std::nullopt);
    EXPECT_EQ(nullptr, joint);
  }
  {
    const auto &[model, joint] =
        CreateTestModel(world, "test2", std::nullopt, child);
    EXPECT_NE(nullptr, joint);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
