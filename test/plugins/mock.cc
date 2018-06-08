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

#include "../MockFeatures.hh"

#include <unordered_map>

#include <iostream>

#include <ignition/physics/Implements.hh>

namespace mock
{
  using MockFeatureList = ignition::physics::FeatureList<MockGetByName>;

  class MockPhysicsPlugin
      : public ignition::physics::Implements3d<MockFeatureList>
  {
    using NameToId = std::unordered_map<std::string, std::size_t>;
    using ParentToNameToId = std::unordered_map<std::size_t, NameToId>;
    using IdToName = std::unordered_map<std::size_t, std::string>;

    std::size_t AddEntity(const std::string &_name,
                   const std::size_t _parentId,
                   ParentToNameToId &_nameToId,
                   IdToName &_idToName)
    {
      const std::size_t id = NextId();
      _idToName[id] = _name;
      _nameToId[_parentId][_name] = id;
      return id;
    }

    public: std::size_t InitiateEngine(std::size_t /*_engineID*/) override
    {
      engineNames[NextId()] = "Only one engine";

      const std::size_t world =
        AddEntity("Some world", 0, engineToWorldNameToId, worldNames);

      const std::size_t model1 =
          AddEntity("First model", world, worldToModelNameToId, modelNames);

      AddEntity("First link", model1, modelToLinkNameToId, linkNames);
      AddEntity("A joint", model1, modelToJointNameToId, jointNames);
      AddEntity("Second link", model1, modelToLinkNameToId, linkNames);

      const std::size_t model2 =
          AddEntity("Second model", world, worldToModelNameToId, modelNames);

      AddEntity("Link of second model", model2, modelToLinkNameToId, linkNames);
      AddEntity("Another joint", model2, modelToJointNameToId, jointNames);

      return 0;
    }

    public: std::shared_ptr<const void> EngineRef(
        std::size_t /*engineID*/) override
    {
      return nullptr;
    }

    public: std::string GetEngineName(std::size_t _id) const override
    {
      return engineNames.at(_id);
    }

    public: std::size_t GetWorldByName(
        std::size_t _engineId, const std::string &_name) const override
    {
      return engineToWorldNameToId.at(_engineId).at(_name);
    }

    public: std::string GetWorldName(std::size_t _id) const override
    {
      return worldNames.at(_id);
    }

    public: std::size_t GetModelByName(
        std::size_t _worldId, const std::string &_name) const override
    {
      return worldToModelNameToId.at(_worldId).at(_name);
    }

    public: std::string GetModelName(std::size_t _id) const override
    {
      return modelNames.at(_id);
    }

    public: std::size_t GetLinkByName(
        std::size_t _modelId, const std::string &_name) const override
    {
      return modelToLinkNameToId.at(_modelId).at(_name);
    }

    public: std::string GetLinkName(std::size_t _id) const override
    {
      return linkNames.at(_id);
    }

    public: std::size_t GetJointByName(
        std::size_t _modelId, const std::string &_name) const override
    {
      return modelToJointNameToId.at(_modelId).at(_name);
    }

    public: std::string GetJointName(std::size_t _id) const override
    {
      return jointNames.at(_id);
    }


    ParentToNameToId engineToWorldNameToId;
    ParentToNameToId worldToModelNameToId;
    ParentToNameToId modelToLinkNameToId;
    ParentToNameToId modelToJointNameToId;

    IdToName engineNames;
    IdToName worldNames;
    IdToName modelNames;
    IdToName linkNames;
    IdToName jointNames;

    std::size_t NextId()
    {
      return highestId++;
    }

    std::size_t highestId;

  };

  IGN_PHYSICS_ADD_PLUGIN(MockPhysicsPlugin, ::ignition::physics::FeaturePolicy3d, MockFeatureList)
}
