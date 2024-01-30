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

#include <map>

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>

#include <gz/plugin/Loader.hh>

#include "test/Utils.hh"

using namespace gz;

TEST(FindFeatures_TEST, ForwardStep)
{
  // List of plugin names that are known to provide this test feature.
  // TODO(MXG): Add more plugins to this list as they are implemented
  const std::set<std::string> knownAcceptablePlugins =
  {
    "gz::physics::dartsim::Plugin"
  };

  using TestFeatures =
    physics::FeatureList<physics::ForwardStep>;

  plugin::Loader loader;
  PrimeTheLoader(loader);

  const std::set<std::string> allPlugins = loader.AllPlugins();
  const std::set<std::string> foundPlugins =
      physics::FindFeatures3d<TestFeatures>::From(loader);

  for (const std::string &acceptable : knownAcceptablePlugins)
  {
    if (allPlugins.count(acceptable) > 0)
    {
      EXPECT_EQ(1u, foundPlugins.count(acceptable));
    }
  }
}

TEST(FindFeatures_TEST, Unimplemented)
{
  using TestFeatures =
    physics::FeatureList<physics::test::UnimplementedFeature>;

  plugin::Loader loader;
  PrimeTheLoader(loader);

  const std::set<std::string> foundPlugins =
      physics::FindFeatures3d<TestFeatures>::From(loader);

  // No plugins should ever have implemented this spoofed feature list
  EXPECT_EQ(0u, foundPlugins.size());
}
