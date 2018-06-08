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

#ifndef IGNITION_PHYSICS_TEST_MOCKFEATURES_HH_
#define IGNITION_PHYSICS_TEST_MOCKFEATURES_HH_

#include <memory>

#include <ignition/physics/FeatureList.hh>

namespace mock
{
  struct MockGetByName : public ignition::physics::Feature
  {
    template <typename PolicyT, typename FeaturesT>
    class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
    {
      public: using World = ignition::physics::World<PolicyT, FeaturesT>;

      std::string Name() const
      {
        return this->template Interface<MockGetByName>()->
            GetEngineName(this->id);
      }

      std::unique_ptr<World> GetWorld(const std::string &_name)
      {
        const std::size_t worldId =
            this->template Interface<MockGetByName>()->GetWorldByName(
              this->id, _name);

        return std::make_unique<World>(this->pimpl, worldId, nullptr);
      }

      std::unique_ptr<const World> GetWorld(const std::string &_name) const
      {
        return const_cast<Engine*>(this)->GetWorld(_name);
      }

      public: virtual ~Engine() = default;
    };

    template <typename PolicyT, typename FeaturesT>
    class World : public virtual Feature::World<PolicyT, FeaturesT>
    {
      public: using Model = ignition::physics::Model<PolicyT, FeaturesT>;

      std::string Name() const
      {
        return this->template Interface<MockGetByName>()->
            GetWorldName(this->id);
      }

      std::unique_ptr<Model> GetModel(const std::string &_name)
      {
        const std::size_t modelId =
            this->template Interface<MockGetByName>()->GetModelByName(
              this->id, _name);

        return std::make_unique<Model>(this->pimpl, modelId, nullptr);
      }

      std::unique_ptr<const Model> GetWorld(const std::string &_name) const
      {
        return const_cast<World*>(this)->GetModel(_name);
      }

      public: virtual ~World() = default;
    };

    template <typename PolicyT, typename FeaturesT>
    class Model : public virtual Feature::World<PolicyT, FeaturesT>
    {
      public: using Link = ignition::physics::Link<PolicyT, FeaturesT>;
      public: using Joint = ignition::physics::Joint<PolicyT, FeaturesT>;

      std::string Name() const
      {
        return this->template Interface<MockGetByName>()->
            GetModelName(this->id);
      }

      std::unique_ptr<Link> GetLink(const std::string &_name)
      {
        const std::size_t linkId =
            this->template Interface<MockGetByName>()->GetLinkByName(
              this->id, _name);

        return std::make_unique<Link>(this->pimpl, linkId, nullptr);
      }

      std::unique_ptr<const Link> GetLink(const std::string &_name) const
      {
        return const_cast<Model*>(this)->GetLink(_name);
      }

      std::unique_ptr<Joint> GetJoint(const std::string &_name)
      {
        const std::size_t jointId =
            this->template Interface<MockGetByName>()->GetJointByName(
              this->id, _name);

        return std::make_unique<Joint>(this->pimpl, jointId, nullptr);
      }

      std::unique_ptr<const Joint> GetJoint(const std::string &_name) const
      {
        return const_cast<Model*>(this)->GetJoint(_name);
      }
    };

    template <typename PolicyT, typename FeaturesT>
    class Link : public virtual Feature::Link<PolicyT, FeaturesT>
    {
      public: std::string Name() const
      {
        return this->template Interface<MockGetByName>()->GetLinkName(this->id);
      }
    };

    template <typename PolicyT, typename FeaturesT>
    class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
    {
      public: std::string Name() const
      {
        return this->template Interface<MockGetByName>()->
            GetJointName(this->id);
      }
    };

    template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: virtual std::string GetEngineName(std::size_t _id) const = 0;

      public: virtual std::size_t GetWorldByName(
          std::size_t _engineId, const std::string &_name) const = 0;

      public: virtual std::string GetWorldName(std::size_t _id) const = 0;

      public: virtual std::size_t GetModelByName(
          std::size_t _worldId, const std::string &_name) const = 0;

      public: virtual std::string GetModelName(std::size_t _id) const = 0;

      public: virtual std::size_t GetLinkByName(
          std::size_t _modelId, const std::string &_name) const = 0;

      public: virtual std::string GetLinkName(std::size_t _id) const = 0;

      public: virtual std::size_t GetJointByName(
          std::size_t _modelId, const std::string &_name) const = 0;

      public: virtual std::string GetJointName(std::size_t _id) const = 0;
    };
  };
}

#endif
