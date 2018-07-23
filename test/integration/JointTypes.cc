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

#include <Eigen/Geometry>

#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/PluginPtr.hh>
#include <ignition/physics/RequestFeatures.hh>

#include <ignition/physics/RevoluteJoint.hh>
#include <ignition/physics/FrameSemantics.hh>

#include "../MockJoints.hh"

using TestFeatures = ignition::physics::FeatureList<
    ignition::physics::RevoluteJoint
>;

/////////////////////////////////////////////////
ignition::plugin::PluginPtr LoadMockJointTypesPlugin(
    const std::string &_suffix)
{
  ignition::plugin::Loader pl;
  auto plugins = pl.LoadLibrary("MockJointTypes_LIB"); // TODO: Replace with macro

  ignition::plugin::PluginPtr plugin =
      pl.Instantiate("mock::MockJointTypesPlugin"+_suffix);
  EXPECT_FALSE(plugin.IsEmpty());

  return plugin;
}

/////////////////////////////////////////////////
template <typename PolicyT>
void TestRevoluteJointCast(const std::string &_suffix)
{
  auto engine =
      ignition::physics::RequestFeatures<PolicyT, mock::MockJointList>::From(
        LoadMockJointTypesPlugin(_suffix));

  auto joint = engine->GetJoint(0);
  auto revolute = joint->CastToRevoluteJoint();

  std::cout << revolute->GetAxis() << std::endl;
}

/////////////////////////////////////////////////
TEST(RevoluteJoint_TEST, Cast)
{
  TestRevoluteJointCast<ignition::physics::FeaturePolicy3d>("3d");
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
