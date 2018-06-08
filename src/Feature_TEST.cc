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

#include <ignition/physics/RequestFeatures.hh>
#include <ignition/physics/Entity.hh>

using namespace ignition::physics;

/////////////////////////////////////////////////
class EngineMockFeature : public virtual Feature
{
  public: template <typename FeatureType, typename Pimpl>
  class Engine : public virtual Feature::Engine<FeatureType, Pimpl>
  {
    public: bool MockAnEngineFunction() const
    {
      return true;
    }
  };
};

/////////////////////////////////////////////////
class LinkMockFeature : public virtual Feature
{
  public: template <typename FeatureType, typename Pimpl>
  class Link : public virtual Feature::Link<FeatureType, Pimpl>
  {
    public: bool MockALinkFunction() const
    {
      return true;
    }
  };
};

/////////////////////////////////////////////////
class SecondLinkMockFeature : public virtual Feature
{
  public: template <typename FeatureType, typename Pimpl>
  class Link : public virtual Feature::Link<FeatureType, Pimpl>
  {
    public: bool MockAnotherLinkFunction() const
    {
      return true;
    }
  };
};

/////////////////////////////////////////////////
TEST(Feature_TEST, SimpleMock)
{
  using MockList = FeatureList<
      EngineMockFeature,
      LinkMockFeature,
      SecondLinkMockFeature>;

  Engine3d<MockList> engine3d;
  EXPECT_TRUE(engine3d.MockAnEngineFunction());

  Link3d<MockList> link3d;
  EXPECT_TRUE(link3d.MockALinkFunction());
  EXPECT_TRUE(link3d.MockAnotherLinkFunction());

  std::set<std::string> missing =
      RequestFeatures3d<MockList>::MissingFeatureNames(
        ignition::common::PluginPtr());

  EXPECT_EQ(3u, missing.size());
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
