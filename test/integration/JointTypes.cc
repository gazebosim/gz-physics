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

using Features = ignition::physics::FeatureList<
  ignition::physics::SetRevoluteJointProperties,
  ignition::physics::GetRevoluteJointProperties
>;

/////////////////////////////////////////////////
TEST(RevoluteJoint_TEST, Cast)
{
  ignition::physics::RevoluteJoint::Using<
      ignition::physics::FeaturePolicy3d, Features> revolute;


//  using Joint = ignition::physics::Joint3d<TestFeatures>;

//  auto engine =
//      ignition::physics::RequestFeatures3d<TestFeatures>::From(
//        LoadMockJointTypesPlugin("TODO"));

//  std::unique_ptr<Joint> joint = engine->GetJoint("RevoluteJoint");
//  auto revolute = joint->CastTo<ignition::physics::RevoluteJoint>();

//  Eigen::Vector3d axis = revolute->GetAxis();
//  Eigen::Matrix3d R = Eigen::AngleAxisd(90.0*M_PI/180.0,
//                                        Eigen::Vector3d(0.0, 1.0, 0.0));
//  axis = R*axis;
//  revolute->SetAxis(axis);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
