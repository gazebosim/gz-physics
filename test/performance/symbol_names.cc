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
#include <ignition/physics/FeaturePolicy.hh>

#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/RevoluteJoint.hh>
#include <ignition/physics/CylinderShape.hh>
#include <ignition/physics/Link.hh>
#include <ignition/physics/CylinderShape.hh>
#include <ignition/physics/BoxShape.hh>
#include <ignition/physics/ForwardStep.hh>

//struct NamedFeatureList : ignition::physics::FeatureList<
//  ignition::physics::CompleteFrameSemantics,
//  ignition::physics::GetRevoluteJointProperties,
//  ignition::physics::SetRevoluteJointProperties,
//  ignition::physics::GetCylinderShapeProperties,
//  ignition::physics::SetCylinderShapeProperties,
//  ignition::physics::GetBoxShapeProperties,
//  ignition::physics::SetBoxShapeProperties,
//  ignition::physics::AttachCylinderShapeFeature,
//  ignition::physics::GetBoxShapeProperties,
//  ignition::physics::SetBoxShapeProperties,
//  ignition::physics::AttachBoxShapeFeature
//> {};

//using AliasFeatureList = ignition::physics::FeatureList<
//  ignition::physics::CompleteFrameSemantics,
//  ignition::physics::GetRevoluteJointProperties,
//  ignition::physics::SetRevoluteJointProperties,
//  ignition::physics::GetCylinderShapeProperties,
//  ignition::physics::SetCylinderShapeProperties,
//  ignition::physics::GetCylinderShapeProperties,
//  ignition::physics::SetCylinderShapeProperties,
//  ignition::physics::AttachCylinderShapeFeature,
//  ignition::physics::GetBoxShapeProperties,
//  ignition::physics::SetBoxShapeProperties,
//  ignition::physics::AttachBoxShapeFeature
//>;



//TEST(symbol_names, Length)
//{
//  using AliasLink = ignition::physics::Link3dPtr<AliasFeatureList>;
//  using NamedLink = ignition::physics::Link3dPtr<NamedFeatureList>;

//  /// The length of the Link defined by a named feature list should be less
//  /// than half the length of the Link defined by an aliased feature list.
//  EXPECT_LT(std::string(typeid(NamedLink).name()).size(),
//            std::string(typeid(AliasLink).name()).size()/2);
//}

//struct SingleNestedNamedList : ignition::physics::FeatureList<
//  NamedFeatureList,
//  AliasFeatureList
//> { };

//struct DoubleNestedNamedList : ignition::physics::FeatureList<
//  ignition::physics::ForwardStep,
//  SingleNestedNamedList
//> { };

//using DoubleNestedAliasList = ignition::physics::FeatureList<
//  ignition::physics::ForwardStep,
//  SingleNestedNamedList
//>;

//TEST(symbol_names, Nested)
//{
//  // Note: This function just needs to compile successfully for the test to pass

//  using SingleNestedLink = ignition::physics::Link3dPtr<SingleNestedNamedList>;
//  SingleNestedLink composite;

//  using DoubleNestedLink = ignition::physics::Link3dPtr<DoubleNestedNamedList>;
//  DoubleNestedLink nested;

//  ignition::physics::Link3dPtr<DoubleNestedAliasList> alias;
//}

struct FeatureSet1 : ignition::physics::FeatureList<
    ignition::physics::CompleteFrameSemantics,
    ignition::physics::GetRevoluteJointProperties,
    ignition::physics::SetRevoluteJointProperties,
    ignition::physics::GetCylinderShapeProperties,
    ignition::physics::SetCylinderShapeProperties
> { };

struct FeatureSet2 : ignition::physics::FeatureList<
    ignition::physics::GetBoxShapeProperties,
    ignition::physics::SetBoxShapeProperties,
    ignition::physics::AttachCylinderShapeFeature,
    ignition::physics::GetBoxShapeProperties,
    ignition::physics::SetBoxShapeProperties,
    ignition::physics::AttachBoxShapeFeature
> { };

struct FeatureSets : ignition::physics::FeatureList<
    FeatureSet1,
    FeatureSet2
> { };

TEST(symbol_names, Length)
{
  ignition::physics::Link3dPtr<FeatureSets> link;
//  link->AttachCylinderShape();
//  link->FrameDataRelativeTo(ignition::physics::FrameID::World());
  std::cout << typeid(ignition::physics::Link3dPtr<FeatureSets>).name() << std::endl;

  ignition::plugin::PluginPtr plugin;
  auto engine = ignition::physics::RequestEngine3d<FeatureSets>::From(plugin);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
