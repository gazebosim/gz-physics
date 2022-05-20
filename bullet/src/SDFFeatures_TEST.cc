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

#include <gz/plugin/Loader.hh>

#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/Utils.hh>

struct TestFeatureList : gz::physics::FeatureList<
    gz::physics::GetBasicJointState,
    gz::physics::SetBasicJointState,
    gz::physics::sdf::ConstructSdfJoint,
    gz::physics::sdf::ConstructSdfLink,
    gz::physics::sdf::ConstructSdfModel,
    gz::physics::sdf::ConstructSdfWorld
> { };

using World = gz::physics::World3d<TestFeatureList>;
using WorldPtr = gz::physics::World3dPtr<TestFeatureList>;

auto LoadEngine()
{
  gz::plugin::Loader loader;
  loader.LoadLib(bullet_plugin_LIB);

  gz::plugin::PluginPtr bullet =
      loader.Instantiate("gz::physics::bullet::Plugin");

  auto engine =
      gz::physics::RequestEngine3d<TestFeatureList>::From(bullet);
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
