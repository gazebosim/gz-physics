/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/physics/FeatureList.hh>

#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/RevoluteJoint.hh>
#include <ignition/physics/CylinderShape.hh>
#include <ignition/physics/Link.hh>

struct FeatureList : ignition::physics::FeatureList<
  ignition::physics::CompleteFrameSemantics,
  ignition::physics::GetRevoluteJointProperties,
  ignition::physics::SetRevoluteJointProperties,
  ignition::physics::GetCylinderShapeProperties,
  ignition::physics::SetCylinderShapeProperties
    > {};

//using FeatureList = ignition::physics::FeatureList<
//  ignition::physics::CompleteFrameSemantics,
//  ignition::physics::GetRevoluteJointProperties,
//  ignition::physics::SetRevoluteJointProperties,
//  ignition::physics::GetCylinderShapeProperties,
//  ignition::physics::SetCylinderShapeProperties
//    >;

TEST(symbol_names, Length)
{
  using Link = ignition::physics::Link3d<FeatureList>;
  std::cout << typeid(Link).name() << std::endl;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
