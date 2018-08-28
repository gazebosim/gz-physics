/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/plugin/Loader.hh>

#include <ignition/physics/Joint.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/RevoluteJoint.hh>

#include "EntityManagementFeatures.hh"
#include "JointFeatures.hh"

using TestFeatureList = ignition::physics::FeatureList<
  ignition::physics::dartsim::EntityManagementFeatureList,
  ignition::physics::dartsim::JointFeatureList
>;

TEST(EntityManagement_TEST, ConstructEmptyWorld)
{
  ignition::plugin::Loader loader;
  loader.LoadLibrary(dartsim_plugin_LIB);

  ignition::plugin::PluginPtr dartsim =
      loader.Instantiate("ignition::physics::dartsim::Plugin");

  auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  ASSERT_NE(nullptr, engine);

  auto world = engine->ConstructEmptyWorld("empty world");
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("empty world", world->GetName());
  EXPECT_EQ(engine, world->GetEngine());

  auto model = world->ConstructEmptyModel("empty model");
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("empty model", model->GetName());
  EXPECT_EQ(world, model->GetWorld());
  EXPECT_NE(model, world->ConstructEmptyModel("dummy"));

  auto link = model->ConstructEmptyLink("empty link");
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("empty link", link->GetName());
  EXPECT_EQ(model, link->GetModel());
  EXPECT_NE(link, model->ConstructEmptyLink("dummy"));

  auto joint = link->AttachRevoluteJoint(nullptr);
  EXPECT_NEAR((Eigen::Vector3d::UnitX() - joint->GetAxis()).norm(), 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(0.0, joint->GetPosition(0));

  joint->SetAxis(Eigen::Vector3d::UnitZ());
  EXPECT_NEAR((Eigen::Vector3d::UnitZ() - joint->GetAxis()).norm(), 0.0, 1e-6);

  auto child = model->ConstructEmptyLink("child link");
  child->AttachPrismaticJoint(link);
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
