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

#include "../MockFrameSemantics.hh"

#include <map>
#include <vector>

#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/Implements.hh>

namespace mock
{
  template <typename PolicyT>
  class MockFrameSemanticsPlugin
      : public ignition::physics::Implements<PolicyT, MockFrameSemanticsList>
  {
    using Identity = ignition::physics::Identity;
    public: using FrameData = typename
        ignition::physics::FrameSemantics::Implementation<PolicyT>::FrameData;

    public: MockFrameSemanticsPlugin()
    {
      // The 0-th frame is always the world frame
      frames.push_back(FrameData());
    }

    public: Identity InitiateEngine(std::size_t /*_engineID*/) override
    {
      return this->GenerateIdentity(0);
    }

    public: Identity CreateEntity(
        const std::string &_entityName,
        const FrameData &_frameData,
        std::map<std::string, std::size_t> &_ids,
        std::map<std::size_t, std::shared_ptr<void>> *_refs)
    {
      const std::size_t newId = frames.size();

      const auto inserted = _ids.insert(std::make_pair(_entityName, newId));
      if (!inserted.second)
        return this->GenerateInvalidId();

      frames.push_back(_frameData);

      std::shared_ptr<void> ref;
      if (_refs)
      {
        ref = std::make_shared<int>(0);
        (*_refs)[newId] = ref;
      }

      return this->GenerateIdentity(newId, ref);
    }

    public: Identity CreateLink(
        const std::string &_linkName,
        const FrameData &_frameData) override
    {
      return this->CreateEntity(_linkName, _frameData, linkIds, &linkRefs);
    }

    public: Identity CreateJoint(
        const std::string &_jointName,
        const FrameData &_frameData) override
    {
      return this->CreateEntity(_jointName, _frameData, jointIds, nullptr);
    }

    public: Identity GetEntity(
        const std::string &_entityName,
        const std::map<std::string, std::size_t> &_ids,
        const std::map<std::size_t, std::shared_ptr<void>> *_refs) const
    {
      const auto it = _ids.find(_entityName);
      if (it == _ids.end())
        return this->GenerateInvalidId();

      std::shared_ptr<void> ref;
      if (_refs)
        ref = _refs->at(it->second);

      return this->GenerateIdentity(it->second, ref);
    }

    public: Identity GetLink(const std::string &_linkName) const override
    {
      return this->GetEntity(_linkName, linkIds, &linkRefs);
    }

    public: Identity GetJoint(const std::string &_jointName) const override
    {
      return this->GetEntity(_jointName, jointIds, nullptr);
    }

    public: FrameData FrameDataRelativeToWorld(
        const ignition::physics::FrameID &_id) const override
    {
      return frames[_id.ID()];
    }

    std::map<std::string, std::size_t> linkIds;
    std::map<std::string, std::size_t> jointIds;
    std::vector<FrameData> frames;

    /// \brief This is used to store smart pointers for links for the sake of
    /// testing. Our test expects links to be reference counted while joints
    /// are not.
    std::map<std::size_t, std::shared_ptr<void>> linkRefs;
  };


#define REGISTER_FRAME_SEMANTICS_PLUGIN( X ) \
  class MockFrameSemanticsPlugin ## X : \
    public MockFrameSemanticsPlugin<ignition::physics::FeaturePolicy ## X>{ }; \
  \
  IGN_PHYSICS_ADD_PLUGIN( \
    MockFrameSemanticsPlugin ## X, \
    ignition::physics::FeaturePolicy ## X, \
    MockFrameSemanticsList)

  REGISTER_FRAME_SEMANTICS_PLUGIN(3d)
  REGISTER_FRAME_SEMANTICS_PLUGIN(2d)
  REGISTER_FRAME_SEMANTICS_PLUGIN(3f)
  REGISTER_FRAME_SEMANTICS_PLUGIN(2f)
}
