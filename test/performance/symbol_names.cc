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

#include <gz/physics/FeatureList.hh>
#include <gz/physics/FeaturePolicy.hh>

#include <gz/physics/BoxShape.hh>
#include <ignition/physics/CapsuleShape.hh>
#include <gz/physics/CylinderShape.hh>
#include <ignition/physics/EllipsoidShape.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RevoluteJoint.hh>

/////////////////////////////////////////////////
struct CapsuleFeaturesClass : ignition::physics::FeatureList<
    ignition::physics::GetCapsuleShapeProperties,
    ignition::physics::SetCapsuleShapeProperties,
    ignition::physics::AttachCapsuleShapeFeature
> { };

using CapsuleFeaturesAlias = ignition::physics::FeatureList<
    ignition::physics::GetCapsuleShapeProperties,
    ignition::physics::SetCapsuleShapeProperties,
    ignition::physics::AttachCapsuleShapeFeature
>;

/////////////////////////////////////////////////
struct CylinderFeaturesClass : ignition::physics::FeatureList<
    ignition::physics::GetCylinderShapeProperties,
    ignition::physics::SetCylinderShapeProperties,
    ignition::physics::AttachCylinderShapeFeature
> { };

using CylinderFeaturesAlias = ignition::physics::FeatureList<
    ignition::physics::GetCylinderShapeProperties,
    ignition::physics::SetCylinderShapeProperties,
    ignition::physics::AttachCylinderShapeFeature
>;

/////////////////////////////////////////////////
struct EllipsoidFeaturesClass : ignition::physics::FeatureList<
    ignition::physics::GetEllipsoidShapeProperties,
    ignition::physics::SetEllipsoidShapeProperties,
    ignition::physics::AttachEllipsoidShapeFeature
> { };

using EllipsoidFeaturesAlias = ignition::physics::FeatureList<
    ignition::physics::GetEllipsoidShapeProperties,
    ignition::physics::SetEllipsoidShapeProperties,
    ignition::physics::AttachEllipsoidShapeFeature
>;

/////////////////////////////////////////////////
struct JointFeaturesClass : ignition::physics::FeatureList<
    ignition::physics::GetRevoluteJointProperties,
    ignition::physics::SetRevoluteJointProperties
> { };

using JointFeaturesAlias = ignition::physics::FeatureList<
    ignition::physics::GetRevoluteJointProperties,
    ignition::physics::SetRevoluteJointProperties
>;

/////////////////////////////////////////////////
struct BoxFeaturesClass : ignition::physics::FeatureList<
    ignition::physics::GetBoxShapeProperties,
    ignition::physics::SetBoxShapeProperties,
    ignition::physics::AttachBoxShapeFeature
> { };

using BoxFeaturesAlias = ignition::physics::FeatureList<
    ignition::physics::GetBoxShapeProperties,
    ignition::physics::SetBoxShapeProperties,
    ignition::physics::AttachBoxShapeFeature
>;

/////////////////////////////////////////////////
struct FeatureSetClass : ignition::physics::FeatureList<
    CapsuleFeaturesClass,
    CylinderFeaturesClass,
    EllipsoidFeaturesClass,
    JointFeaturesClass,
    BoxFeaturesClass
> { };

using FeatureSetAlias = ignition::physics::FeatureList<
    CapsuleFeaturesAlias,
    CylinderFeaturesAlias,
    EllipsoidFeaturesAlias,
    JointFeaturesAlias,
    BoxFeaturesAlias
>;

TEST(symbol_names, Length)
{
  const std::string composite_featurelist_name =
      typeid(ignition::physics::Link3dPtr<FeatureSetClass>).name();

  const std::string aliased_featurelist_name =
      typeid(ignition::physics::Link3dPtr<FeatureSetAlias>).name();

  // The size of the feature list that uses inheritance composition should be
  // less than a third of the size of the feature list that uses aliasing.
  EXPECT_LT(composite_featurelist_name.size(),
            aliased_featurelist_name.size()/3);


  // We instantiate these so we can observe the kinds of symbols that get
  // created when these classes are instantiated. This isn't tested explicitly,
  // but the symbols can be seen by running
  // $ nm PERFORMANCE_symbol_names
  auto engineClass = ignition::physics::RequestEngine3d<FeatureSetClass>::From(
        ignition::plugin::PluginPtr());

  auto engineAlias = ignition::physics::RequestEngine3d<FeatureSetAlias>::From(
        ignition::plugin::PluginPtr());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
