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

#include "TestUtilities.hh"

TEST(RequestFeatures_TEST, Casting)
{
  ignition::plugin::Loader loader;
  PrimeTheLoader(loader);

  using InitialFeatures =
    ignition::physics::FeatureList<
      ignition::physics::ConstructEmptyWorldFeature,
      ignition::physics::GetWorldFromEngine>;

  using ExtendFeatures =
    ignition::physics::FeatureList<
      InitialFeatures,
      ignition::physics::ConstructEmptyModelFeature,
      ignition::physics::GetModelFromWorld>;

  using SidewaysFeatures =
    ignition::physics::FeatureList<
      ignition::physics::ConstructEmptyLinkFeature,
      ignition::physics::AttachRevoluteJointFeature>;

  using AllFeatures =
    ignition::physics::FeatureList<ExtendFeatures, SidewaysFeatures>;

  using UnavailableFeatures =
    ignition::physics::FeatureList<test::UnimplementedFeature>;

  // Get a list of all plugins who satisfy the features that are needed for
  // testing
  const auto validPlugins =
      ignition::physics::FindFeatures3d<AllFeatures>::From(loader);
  ASSERT_LT(0u, validPlugins.size());

  for (const auto &pluginName : validPlugins)
  {
    auto engine = ignition::physics::RequestEngine3d<InitialFeatures>::From(
          loader.Instantiate(pluginName));
    ASSERT_TRUE(engine);

    auto initialWorld = engine->ConstructEmptyWorld("test_world");
    ASSERT_TRUE(initialWorld);
    EXPECT_EQ("test_world", initialWorld->GetName());

    EXPECT_TRUE(ignition::physics::RequestFeatures<ExtendFeatures>
                ::MissingFeatureNames(initialWorld).empty());

    auto extendWorld =
        ignition::physics::RequestFeatures<ExtendFeatures>::From(initialWorld);
    ASSERT_TRUE(extendWorld);
    auto model = extendWorld->ConstructEmptyModel("test_model");
    ASSERT_TRUE(model);
    EXPECT_EQ("test_model", model->GetName());

    EXPECT_TRUE(ignition::physics::RequestFeatures<SidewaysFeatures>
                ::MissingFeatureNames(model).empty());

    auto sidewaysModel =
        ignition::physics::RequestFeatures<SidewaysFeatures>::From(model);
    ASSERT_TRUE(sidewaysModel);
    auto link = sidewaysModel->ConstructEmptyLink("test_link");
    link->AttachRevoluteJoint(nullptr);

    EXPECT_EQ(1u, ignition::physics::RequestFeatures<UnavailableFeatures>
              ::MissingFeatureNames(link).size());

    auto unavailableFeatureLink =
        ignition::physics::RequestFeatures<UnavailableFeatures>::From(link);
    // Since the feature is not available, this should be a null EntityPtr
    EXPECT_FALSE(unavailableFeatureLink);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
