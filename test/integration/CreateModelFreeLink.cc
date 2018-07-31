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
#include <ignition/plugin/PluginPtr.hh>
#include <ignition/physics/RequestFeatures.hh>

#include "../MockCreateEntities.hh"


/////////////////////////////////////////////////
ignition::plugin::PluginPtr LoadMockCreateModelFreeBodyPlugin(
    const std::string &_pluginName)
{
  ignition::plugin::Loader pl;
  auto plugins = pl.LoadLibrary(MockEntities_LIB);
  EXPECT_EQ(2u, plugins.size());

  ignition::plugin::PluginPtr plugin =
      pl.Instantiate(_pluginName);
  EXPECT_FALSE(plugin.IsEmpty());

  return plugin;
}

/////////////////////////////////////////////////
template <typename PolicyT>
void TestCreateStaticBox(const double _tolerance)
{
  using Scalar = typename PolicyT::Scalar;
  constexpr std::size_t Dim = PolicyT::Dim;

  // Instantiate an engine that provides Frame Semantics.
  auto fs =
      ignition::physics::RequestFeatures<PolicyT, mock::MockCreateEntities>
        ::From(LoadMockCreateModelFreeBodyPlugin("mock::EntitiesPlugin3d"));

  mock::MockWorld3dPtr world = engine->GetWorld("Some world");
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("Some world", world->Name());

  mock::ModelFreeLink model;
  model.name = "static_box";
  model.link = mock::Link();
  model.link.name = "link";
  model.link.motionType = mock::STATIC;
  mock::Collision collision;
  auto box = collision.Geometry.InsertOrAssign<ignition::math::Box>(10, 10, 1);
  model.link.collisions.push_back(collision);
  mock::MockModel3dPtr newModel = world->CreateModelFreeLink(model);
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrames3d)
{
  TestRelativeFrames<ignition::physics::FeaturePolicy3d>(1e-11);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
