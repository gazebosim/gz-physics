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

#include <ignition/physics/BoxShape.hh>
#include <ignition/physics/CapsuleShape.hh>
#include <ignition/physics/CylinderShape.hh>
#include <ignition/physics/EllipsoidShape.hh>
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/Link.hh>
#include <ignition/physics/RevoluteJoint.hh>


/////////////////////////////////////////////////
struct CapsuleFeaturesClass : gz::physics::FeatureList<
    gz::physics::GetCapsuleShapeProperties,
    gz::physics::SetCapsuleShapeProperties,
    gz::physics::AttachCapsuleShapeFeature
> { };

using CapsuleFeaturesAlias = gz::physics::FeatureList<
    gz::physics::GetCapsuleShapeProperties,
    gz::physics::SetCapsuleShapeProperties,
    gz::physics::AttachCapsuleShapeFeature
>;

/////////////////////////////////////////////////
struct CylinderFeaturesClass : gz::physics::FeatureList<
    gz::physics::GetCylinderShapeProperties,
    gz::physics::SetCylinderShapeProperties,
    gz::physics::AttachCylinderShapeFeature
> { };

using CylinderFeaturesAlias = gz::physics::FeatureList<
    gz::physics::GetCylinderShapeProperties,
    gz::physics::SetCylinderShapeProperties,
    gz::physics::AttachCylinderShapeFeature
>;

/////////////////////////////////////////////////
struct EllipsoidFeaturesClass : gz::physics::FeatureList<
    gz::physics::GetEllipsoidShapeProperties,
    gz::physics::SetEllipsoidShapeProperties,
    gz::physics::AttachEllipsoidShapeFeature
> { };

using EllipsoidFeaturesAlias = gz::physics::FeatureList<
    gz::physics::GetEllipsoidShapeProperties,
    gz::physics::SetEllipsoidShapeProperties,
    gz::physics::AttachEllipsoidShapeFeature
>;

/////////////////////////////////////////////////
struct JointFeaturesClass : gz::physics::FeatureList<
    gz::physics::GetRevoluteJointProperties,
    gz::physics::SetRevoluteJointProperties
> { };

using JointFeaturesAlias = gz::physics::FeatureList<
    gz::physics::GetRevoluteJointProperties,
    gz::physics::SetRevoluteJointProperties
>;

/////////////////////////////////////////////////
struct BoxFeaturesClass : gz::physics::FeatureList<
    gz::physics::GetBoxShapeProperties,
    gz::physics::SetBoxShapeProperties,
    gz::physics::AttachBoxShapeFeature
> { };

using BoxFeaturesAlias = gz::physics::FeatureList<
    gz::physics::GetBoxShapeProperties,
    gz::physics::SetBoxShapeProperties,
    gz::physics::AttachBoxShapeFeature
>;

/////////////////////////////////////////////////
struct FeatureSetClass : gz::physics::FeatureList<
    CapsuleFeaturesClass,
    CylinderFeaturesClass,
    EllipsoidFeaturesClass,
    JointFeaturesClass,
    BoxFeaturesClass
> { };

using FeatureSetAlias = gz::physics::FeatureList<
    CapsuleFeaturesAlias,
    CylinderFeaturesAlias,
    EllipsoidFeaturesAlias,
    JointFeaturesAlias,
    BoxFeaturesAlias
>;

TEST(symbol_names, Length)
{
  const std::string composite_featurelist_name =
      typeid(gz::physics::Link3dPtr<FeatureSetClass>).name();

  const std::string aliased_featurelist_name =
      typeid(gz::physics::Link3dPtr<FeatureSetAlias>).name();

  // The size of the feature list that uses inheritance composition should be
  // less than a third of the size of the feature list that uses aliasing.
  EXPECT_LT(composite_featurelist_name.size(),
            aliased_featurelist_name.size()/3);


  // We instantiate these so we can observe the kinds of symbols that get
  // created when these classes are instantiated. This isn't tested explicitly,
  // but the symbols can be seen by running
  // $ nm PERFORMANCE_symbol_names
  auto engineClass = gz::physics::RequestEngine3d<FeatureSetClass>::From(
        gz::plugin::PluginPtr());

  auto engineAlias = gz::physics::RequestEngine3d<FeatureSetAlias>::From(
        gz::plugin::PluginPtr());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
