/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/RevoluteJoint.hh>

#include <gz/physics/FeatureList.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RequestFeatures.hh>

#include <gz/plugin/Loader.hh>

#include "../../src/TestUtilities.hh"
#include "../MockFeatures.hh"

TEST(RequestFeatures_TEST, Casting)
{
  using InitialFeatures =
    ignition::physics::FeatureList<
      mock::MockGetByName>;

  using ExtendFeatures =
    ignition::physics::FeatureList<
      InitialFeatures,
      mock::MockSetName>;

  using SidewaysFeatures =
    ignition::physics::FeatureList<
      mock::MockCenterOfMass>;

  using UnavailableFeatures =
    ignition::physics::FeatureList<test::UnimplementedFeature>;

  // Get a list of all plugins who satisfy the features that are needed for
  // testing
  ignition::plugin::Loader loader;
  const auto pluginNames = loader.LoadLib(MockEntities_LIB);
  EXPECT_EQ(2u, pluginNames.size());

  auto plugin = loader.Instantiate("mock::EntitiesPlugin3d");
  EXPECT_TRUE(plugin);

  auto engine =
      ignition::physics::RequestEngine3d<InitialFeatures>::From(plugin);
  ASSERT_TRUE(engine);

  // Initial features
  auto initialWorld = engine->GetWorld("Some world");
  ASSERT_TRUE(initialWorld);
  EXPECT_EQ("Some world", initialWorld->Name());

  EXPECT_TRUE(ignition::physics::RequestFeatures<ExtendFeatures>
              ::MissingFeatureNames(initialWorld).empty());

  // Extend features
  auto extendWorld =
      ignition::physics::RequestFeatures<ExtendFeatures>::From(initialWorld);
  ASSERT_TRUE(extendWorld);
  extendWorld->SetName("Another world");
  EXPECT_EQ("Another world", initialWorld->Name());

  auto extendModel = extendWorld->GetModel("First model");
  ASSERT_TRUE(extendModel);
  extendModel->SetName("Another model");
  EXPECT_EQ("Another model", extendModel->Name());

  // Sideways features
  EXPECT_TRUE(ignition::physics::RequestFeatures<SidewaysFeatures>
              ::MissingFeatureNames(extendModel).empty());

  auto sidewaysModel =
      ignition::physics::RequestFeatures<SidewaysFeatures>::From(extendModel);
  ASSERT_TRUE(sidewaysModel);
  sidewaysModel->CenterOfMass();

  // Unavailable features
  EXPECT_EQ(1u, ignition::physics::RequestFeatures<UnavailableFeatures>
            ::MissingFeatureNames(sidewaysModel).size());

  auto unavailableFeatureModel =
      ignition::physics::RequestFeatures<UnavailableFeatures>::From(
      sidewaysModel);
  EXPECT_FALSE(unavailableFeatureModel);

  // Invalid entity
  EXPECT_EQ(1u, ignition::physics::RequestFeatures<InitialFeatures>
            ::MissingFeatureNames(unavailableFeatureModel).size());

  auto invalidModel =
      ignition::physics::RequestFeatures<UnavailableFeatures>::From(
      unavailableFeatureModel);
  EXPECT_FALSE(invalidModel);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
