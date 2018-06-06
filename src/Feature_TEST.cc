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

#include <ignition/physics/Feature.hh>

#include <gtest/gtest.h>

using namespace ignition::physics;

/////////////////////////////////////////////////
class EngineMockFeature : public virtual Feature
{
  public: template <typename P>
  class Engine
  {
    public: bool MockAnEngine() const
    {
      return true;
    }
  };
};

/////////////////////////////////////////////////
class LinkMockFeature : public virtual Feature
{
  public: template <typename P>
  class Link
  {
    public: bool MockALink() const
    {
      return true;
    }
  };
};

/////////////////////////////////////////////////
class SecondLinkMockFeature : public virtual Feature
{
  public: template <typename P>
  class Link
  {
    public: bool MockAnotherLinkFeature() const
    {
      return true;
    }
  };
};

/////////////////////////////////////////////////
TEST(Feature_TEST, Mock)
{
  using MockList = FeatureList<
      EngineMockFeature,
      LinkMockFeature,
      SecondLinkMockFeature>;

  Engine3d<MockList> engine3d;
  EXPECT_TRUE(engine3d.MockAnEngine());

  Link3d<MockList> link3d;
  EXPECT_TRUE(link3d.MockALink());
  EXPECT_TRUE(link3d.MockAnotherLinkFeature());
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
