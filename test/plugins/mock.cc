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
  template <typename PolicyT>
  class MockPhysicsPlugin
      : public ignition::physics::Implements<PolicyT, MockFeatureList>
  {
    using Identity = ignition::physics::Identity;

    using NameToId = std::unordered_map<std::string, std::size_t>;
    using ParentToNameToId = std::unordered_map<std::size_t, NameToId>;
    using IdToName = std::unordered_map<std::size_t, std::string>;
    using IdToParent = std::unordered_map<std::size_t, std::size_t>;
    using IdToCom = std::unordered_map<std::size_t, Vector<PolicyT>>;

    std::size_t AddEntity(
        const std::string &_name,
        const std::size_t _parentId,
        ParentToNameToId &_parentToNameToId,
        IdToName &_idToName)
    {
      const std::size_t id = NextId();
      _idToName[id] = _name;
      _parentToNameToId[_parentId][_name] = id;
      parentId[id] = _parentId;
      return id;
    }

    void AddLink(
        const std::string &_name,
        const std::size_t _modelId,
        const Eigen::Vector3d &_com)
    {
      const std::size_t id =
          AddEntity(_name, _modelId, modelToLinkNameToId, linkNames);

      // The input is a 3d vector, but we'll just grab however many components
      // we actually care about based on the current policy of this plugin.
      Vector<PolicyT> com;
      for (std::size_t i=0; i < PolicyT::Dim; ++i)
        com[i] = static_cast<typename PolicyT::Scalar>(_com[i]);

      linkCenterOfMass[id] = com;
    }

    Identity GetEntityByName(
        const std::size_t _parent,
        const std::string &_name,
        const ParentToNameToId &_map) const
    {
      const NameToId &nameToId = _map.at(_parent);
      const NameToId::const_iterator it = nameToId.find(_name);
      if (it == nameToId.end())
        return this->GenerateInvalidId();

      return this->GenerateIdentity(it->second);
    }

    bool SetEntityName(
        const std::size_t _id,
        const std::string &_desiredName,
        ParentToNameToId &_parentToNameMap,
        IdToName &_idToNameMap)
    {
      const std::size_t parent = parentId.at(_id);
      NameToId &nameMap = _parentToNameMap.at(parent);

      const auto result = nameMap.insert(std::make_pair(_desiredName, _id));
      if (!result.second)
      {
        const NameToId::const_iterator &name_it = result.first;
        if (_id == name_it->second)
        {
          // The entity already has the desired name
          return true;
        }

        return false;
      }

      IdToName::iterator it = _idToNameMap.find(_id);
      const std::string &originalName = it->second;
      nameMap.erase(originalName);
      it->second = _desiredName;

      return true;
    }

    public: ignition::physics::Identity InitiateEngine(
        std::size_t /*_engineID*/) override
    {
      engineNames[NextId()] = "Only one engine";

      const std::size_t world =
        AddEntity("Some world", 0, engineToWorldNameToId, worldNames);

      const std::size_t model1 =
          AddEntity("First model", world, worldToModelNameToId, modelNames);

      AddLink("First link", model1, Eigen::Vector3d(1.0, 0.0, 3.0));
      AddEntity("A joint", model1, modelToJointNameToId, jointNames);
      AddLink("Second link", model1, Eigen::Vector3d(0.0, 2.0, 4.0));

      const std::size_t model2 =
          AddEntity("Second model", world, worldToModelNameToId, modelNames);

      AddLink("Link of second model", model2, Eigen::Vector3d(5.0, 0.0, 8.0));
      AddEntity("Another joint", model2, modelToJointNameToId, jointNames);

      return this->GenerateIdentity(0);
    }

    public: std::string GetEngineName(std::size_t _id) const override
    {
      return engineNames.at(_id);
    }

    public: Identity GetWorldByName(
        std::size_t _engineId, const std::string &_name) const override
    {
      return this->GetEntityByName(_engineId, _name, engineToWorldNameToId);
    }

    public: std::string GetWorldName(std::size_t _id) const override
    {
      return worldNames.at(_id);
    }

    public: Identity GetModelByName(
        std::size_t _worldId, const std::string &_name) const override
    {
      return this->GetEntityByName(_worldId, _name, worldToModelNameToId);
    }

    public: std::string GetModelName(std::size_t _id) const override
    {
      return modelNames.at(_id);
    }

    public: Identity GetLinkByName(
        std::size_t _modelId, const std::string &_name) const override
    {
      return this->GetEntityByName(_modelId, _name, modelToLinkNameToId);
    }

    public: std::string GetLinkName(std::size_t _id) const override
    {
      return linkNames.at(_id);
    }

    public: Identity GetJointByName(
        std::size_t _modelId, const std::string &_name) const override
    {
      return this->GetEntityByName(_modelId, _name, modelToJointNameToId);
    }

    public: std::string GetJointName(std::size_t _id) const override
    {
      return jointNames.at(_id);
    }

    public: bool SetEngineName(
        std::size_t _id, const std::string &_desiredName) override
    {
      engineNames[_id] = _desiredName;
      return true;
    }

    public: bool SetWorldName(
        std::size_t _id, const std::string &_desiredName) override
    {
      return this->SetEntityName(
            _id, _desiredName, engineToWorldNameToId, worldNames);
    }

    public: bool SetModelName(
        std::size_t _id, const std::string &_desiredName) override
    {
      return this->SetEntityName(
            _id, _desiredName, worldToModelNameToId, modelNames);
    }

    public: bool SetLinkName(
        std::size_t _id, const std::string &_desiredName) override
    {
      return this->SetEntityName(
            _id, _desiredName, modelToLinkNameToId, linkNames);
    }

    public: bool SetJointName(
        std::size_t _id, const std::string &_desiredName) override
    {
      return this->SetEntityName(
            _id, _desiredName, modelToJointNameToId, jointNames);
    }

    public: Vector<PolicyT> GetLinkCenterOfMass(std::size_t _id) const override
    {
      return linkCenterOfMass.at(_id);
    }

    public: Vector<PolicyT> GetModelCenterOfMass(std::size_t _id) const override
    {
      // For this pretend physics plugin, we'll pretend that all links are of
      // equal mass.
      unsigned int count = 0;
      const NameToId &linkMap = modelToLinkNameToId.at(_id);
      Vector<PolicyT> com = Vector<PolicyT>::Zero();

      // Note: This is a very inefficient way to look up Link COMs, but this is
      // just a test.
      for (const auto &entry : linkCenterOfMass)
      {
        const std::size_t linkId = entry.first;
        const std::string &linkName = linkNames.at(linkId);
        if (linkMap.count(linkName) > 0)
        {
          ++count;
          com += entry.second;
        }
      }

      if (count == 0)
      {
        std::cerr << "Requested center of mass of model ["
                  << modelNames.at(_id) << "], but the model does not "
                  << "contain any links\n";
      }

      com /= static_cast<double>(count);
      return com;
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

    IdToParent parentId;

    IdToCom linkCenterOfMass;

    std::size_t NextId()
    {
      return highestId++;
    }

    std::size_t highestId;

  };

  class MockPhysicsPlugin3d
      : public MockPhysicsPlugin<ignition::physics::FeaturePolicy3d> { };

  IGN_PHYSICS_ADD_PLUGIN(
      MockPhysicsPlugin3d,
      ignition::physics::FeaturePolicy3d,
      MockFeatureList)

  class MockPhysicsPlugin2d
    : public MockPhysicsPlugin<ignition::physics::FeaturePolicy2d> { };

  IGN_PHYSICS_ADD_PLUGIN(
      MockPhysicsPlugin2d,
      ignition::physics::FeaturePolicy2d,
      MockFeatureList)
}
