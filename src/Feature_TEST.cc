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

#include <gz/physics/RequestEngine.hh>
#include <gz/physics/Entity.hh>

using namespace gz::physics;

/////////////////////////////////////////////////
class EngineMockFeature : public virtual Feature
{
  public: template <typename Policy, typename Features>
  class Engine : public virtual Feature::Engine<Policy, Features>
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
TEST(Feature_TEST, ConflictsWith)
{
  class FeatureA : public virtual Feature { };
  class FeatureB : public virtual Feature { };

  EXPECT_FALSE(FeatureA::ConflictsWith<FeatureA>());
  EXPECT_FALSE(FeatureA::ConflictsWith<FeatureB>());
  EXPECT_FALSE(FeatureB::ConflictsWith<FeatureA>());
  EXPECT_FALSE(FeatureB::ConflictsWith<FeatureB>());
}

/////////////////////////////////////////////////
TEST(Feature_TEST, SimpleMock)
{
  using MockList = FeatureList<
      EngineMockFeature,
      LinkMockFeature,
      SecondLinkMockFeature>;

  class BogusImplementation : detail::Implementation
  {
    // Generate a bogus identity to side-step the constraints on creating
    // entities.
    public: Identity Generate() const
    {
      return this->Implementation::GenerateIdentity(0, nullptr);
    }

    public: std::shared_ptr<Entity<FeaturePolicy3d, MockList>::Pimpl> pimpl;
  };

  BogusImplementation bogus;

  // Note: We initialize these entities with garbage because in this case, it
  // doesn't matter. These "features" don't actually use any plugin.
  Engine3dPtr<MockList> engine3d =
      Engine3dPtr<MockList>(bogus.pimpl, bogus.Generate());
  EXPECT_TRUE(engine3d->MockAnEngineFunction());

  Link3dPtr<MockList> link3d(bogus.pimpl, bogus.Generate());
  EXPECT_TRUE(link3d->MockALinkFunction());
  EXPECT_TRUE(link3d->MockAnotherLinkFunction());

  // An empty plugin will not provide any features. When we ask for the missing
  // feature names, we should receive the name of every feature in the list, for
  // a total of 3 feature names.
  std::set<std::string> missing =
      RequestEngine3d<MockList>::MissingFeatureNames(
        gz::plugin::PluginPtr());

  EXPECT_EQ(3u, missing.size());
}
