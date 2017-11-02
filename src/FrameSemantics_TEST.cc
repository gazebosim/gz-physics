/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include <gtest/gtest.h>

#include <ignition/physics/FrameSemantics.hh>

using ignition::physics::FrameData3d;
using ignition::physics::FrameID;

/////////////////////////////////////////////////
class EngineLink
{
  public: EngineLink(
    const std::string &_name,
    const FrameData3d &_data)
    : name(_name),
      data(_data)
  {
    // Do nothing
  }

  public: std::string name;
  public: FrameData3d data;
};

/////////////////////////////////////////////////
class TestFrameSemantics final :
    public ignition::physics::CompleteFrameSemantics::Engine<double, 3>
{
  public: class Link : public virtual ignition::physics::CompleteFrameSemantics::Link<double, 3>
  {
    public: Link(
        ignition::physics::Feature::Engine *const _engine,
        const std::size_t _id,
        const std::shared_ptr<const void> &_ref)
      : BasicObject(_engine, _id, _ref)
    {
      // Do nothing
    }
  };

  public: TestFrameSemantics()
  {
    // Initialize this with a nullptr object which will act as a stand-in for
    // the world frame (which must have an object ID of 0).
    idToEngineLink.push_back(std::weak_ptr<EngineLink>());
  }

  /////////////////////////////////////////////////
  public: FrameData3d FrameDataRelativeToWorld(
    const FrameID &_id) const override
  {
    const std::size_t id = _id.ID();
//    ASSERT_LT(id, idToEngineLink.size());

    const std::weak_ptr<EngineLink> &weakLink = idToEngineLink[id];
    const std::shared_ptr<EngineLink> &link = weakLink.lock();
//    ASSERT_NE(nullptr, link);

    return link->data;
  }

  /////////////////////////////////////////////////
  public: Link *CreateLink(
    const std::string &_linkName,
    const FrameData3d &_frameData)
  {
    LinkNameMap::iterator it; bool inserted;
    std::tie(it, inserted) = this->nameToLink.insert(
          std::make_pair(_linkName, nullptr));

    if (inserted)
    {
      std::shared_ptr<EngineLink> newEngineLink =
          std::make_shared<EngineLink>(_linkName, _frameData);

      const std::size_t newID = idToEngineLink.size();
      Link *newProxyLink = new Link(this, newID, newEngineLink);
      it->second = std::unique_ptr<Link>(newProxyLink);

      this->idToEngineLink.push_back(newEngineLink);
      this->activeEngineLinks.insert(std::make_pair(newID, newEngineLink));
    }

    return it->second.get();
  }

  /////////////////////////////////////////////////
  public: Link *GetLink(const std::string &_linkName)
  {
    LinkNameMap::iterator linkEntry = nameToLink.find(_linkName);
    if (linkEntry == nameToLink.end())
      return nullptr;

    return linkEntry->second.get();
  }

  /////////////////////////////////////////////////
  public: const Link *GetLink(const std::string &_linkName) const
  {
    return const_cast<TestFrameSemantics*>(this)->GetLink(_linkName);
  }

  /////////////////////////////////////////////////
  public: void DeactivateLink(const std::string &_linkName)
  {
    Link *link = this->GetLink(_linkName);

    if (link)
      activeEngineLinks.erase(link->GetFrameID().ID());
  }

  private: using LinkNameMap = std::map<std::string, std::unique_ptr<Link>>;
  /// \brief A map from a name to a proxy Link object. This allows us to do
  /// lookups based on the link name.
  private: LinkNameMap nameToLink;

  /// \brief A map from a unique object ID to the corresponding engine link (if
  /// it still exists).
  private: std::vector<std::weak_ptr<EngineLink>> idToEngineLink;

  private: using ActiveEngineLinkMap =
      std::unordered_map<std::size_t, std::shared_ptr<EngineLink>>;
  /// \brief A map from a unique ID to the corresponding engine link. All engine
  /// links inside of this map are guaranteed to still exist.
  private: ActiveEngineLinkMap activeEngineLinks;

};

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID)
{
  // We test FrameID in this unit test, because the FrameSemantics interface is
  // needed in order to produce FrameIDs.

  TestFrameSemantics fs;

  FrameData3d link1;
  link1.pose.translation() = ignition::physics::Vector3d(0.1, 0.2, 0.3);

  FrameID link1ID = fs.CreateLink("link1", link1)->GetFrameID();


}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
