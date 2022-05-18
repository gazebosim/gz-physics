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

#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/ConstructEmpty.hh>
#include <ignition/physics/RevoluteJoint.hh>

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FindFeatures.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/RequestFeatures.hh>

#include <ignition/plugin/Loader.hh>

#include "../../src/TestUtilities.hh"
#include "../MockFeatures.hh"

TEST(RequestFeatures_TEST, Casting)
{
  using InitialFeatures =
    gz::physics::FeatureList<
      mock::MockGetByName>;

  using ExtendFeatures =
    gz::physics::FeatureList<
      InitialFeatures,
      mock::MockSetName>;

  using SidewaysFeatures =
    gz::physics::FeatureList<
      mock::MockCenterOfMass>;

  using UnavailableFeatures =
    gz::physics::FeatureList<test::UnimplementedFeature>;

  // Get a list of all plugins who satisfy the features that are needed for
  // testing
  gz::plugin::Loader loader;
  const auto pluginNames = loader.LoadLib(MockEntities_LIB);
  EXPECT_EQ(2u, pluginNames.size());

  auto plugin = loader.Instantiate("mock::EntitiesPlugin3d");
  EXPECT_TRUE(plugin);

  auto engine =
      gz::physics::RequestEngine3d<InitialFeatures>::From(plugin);
  ASSERT_TRUE(engine);

  // Initial features
  auto initialWorld = engine->GetWorld("Some world");
  ASSERT_TRUE(initialWorld);
  EXPECT_EQ("Some world", initialWorld->Name());

  EXPECT_TRUE(gz::physics::RequestFeatures<ExtendFeatures>
              ::MissingFeatureNames(initialWorld).empty());

  // Extend features
  auto extendWorld =
      gz::physics::RequestFeatures<ExtendFeatures>::From(initialWorld);
  ASSERT_TRUE(extendWorld);
  extendWorld->SetName("Another world");
  EXPECT_EQ("Another world", initialWorld->Name());

  auto extendModel = extendWorld->GetModel("First model");
  ASSERT_TRUE(extendModel);
  extendModel->SetName("Another model");
  EXPECT_EQ("Another model", extendModel->Name());

  // Sideways features
  EXPECT_TRUE(gz::physics::RequestFeatures<SidewaysFeatures>
              ::MissingFeatureNames(extendModel).empty());

  auto sidewaysModel =
      gz::physics::RequestFeatures<SidewaysFeatures>::From(extendModel);
  ASSERT_TRUE(sidewaysModel);
  sidewaysModel->CenterOfMass();

  // Unavailable features
  EXPECT_EQ(1u, gz::physics::RequestFeatures<UnavailableFeatures>
            ::MissingFeatureNames(sidewaysModel).size());

  auto unavailableFeatureModel =
      gz::physics::RequestFeatures<UnavailableFeatures>::From(
      sidewaysModel);
  EXPECT_FALSE(unavailableFeatureModel);

  // Invalid entity
  EXPECT_EQ(1u, gz::physics::RequestFeatures<InitialFeatures>
            ::MissingFeatureNames(unavailableFeatureModel).size());

  auto invalidModel =
      gz::physics::RequestFeatures<UnavailableFeatures>::From(
      unavailableFeatureModel);
  EXPECT_FALSE(invalidModel);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
