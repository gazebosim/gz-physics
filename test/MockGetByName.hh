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

#ifndef IGNITION_PHYSICS_TEST_MOCKGETBYNAME_HH_
#define IGNITION_PHYSICS_TEST_MOCKGETBYNAME_HH_

#include <memory>

#include <ignition/physics/FeatureList.hh>

namespace mock
{
  /// \brief A feature for retrieving objects from their owners by name, and for
  /// checking the names of those objects.
  struct MockGetByName : public ignition::physics::Feature
  {
    using Identity = ignition::physics::Identity;

    template <typename PolicyT, typename FeaturesT>
    class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
    {
      public: using WorldPtr = ignition::physics::WorldPtr<PolicyT, FeaturesT>;
      public: using ConstWorldPtr =
          ignition::physics::ConstWorldPtr<PolicyT, FeaturesT>;

      /// \brief Name of the engine
      std::string Name() const;

      /// \brief Get a pointer to the world of this engine with the matching
      /// name
      WorldPtr GetWorld(const std::string &_name);

      /// \brief Get a const-qualified pointer to the world of this engine with
      /// the matching name
      ConstWorldPtr GetWorld(const std::string &_name) const;

      // Virtual destructor
      public: virtual ~Engine() = default;
    };

    template <typename PolicyT, typename FeaturesT>
    class World : public virtual Feature::World<PolicyT, FeaturesT>
    {
      public: using ModelPtr = ignition::physics::ModelPtr<PolicyT, FeaturesT>;
      public: using ConstModelPtr =
          ignition::physics::ConstModelPtr<PolicyT, FeaturesT>;

      /// \brief Name of this world
      std::string Name() const;

      /// \brief Get a pointer to the model of this world with the matching name
      ModelPtr GetModel(const std::string &_name);

      /// \brief Get a const-qualified pointer to the model of this world with
      /// the matching name.
      ConstModelPtr GetModel(const std::string &_name) const;

      // Virtual destructor
      public: virtual ~World() = default;
    };

    template <typename PolicyT, typename FeaturesT>
    class Model : public virtual Feature::Model<PolicyT, FeaturesT>
    {
      public: using LinkPtr = ignition::physics::LinkPtr<PolicyT, FeaturesT>;
      public: using ConstLinkPtr =
          ignition::physics::ConstLinkPtr<PolicyT, FeaturesT>;

      public: using JointPtr = ignition::physics::JointPtr<PolicyT, FeaturesT>;
      public: using ConstJointPtr =
          ignition::physics::ConstJointPtr<PolicyT, FeaturesT>;

      /// \brief Name of this model
      std::string Name() const;

      /// \brief Get a pointer to the link of this model with the matching name
      LinkPtr GetLink(const std::string &_name);

      /// \brief Get a const-qualified pointer to the link of this model with
      /// the matching name
      ConstLinkPtr GetLink(const std::string &_name) const;

      /// \brief Get a pointer to the joint of this model with the matching name
      JointPtr GetJoint(const std::string &_name);

      /// \brief Get a const-qualified pointer to the joint of this model with
      /// the matching name
      ConstJointPtr GetJoint(const std::string &_name) const;
    };

    template <typename PolicyT, typename FeaturesT>
    class Link : public virtual Feature::Link<PolicyT, FeaturesT>
    {
      /// \brief Get the name of this link
      public: std::string Name() const;
    };

    template <typename PolicyT, typename FeaturesT>
    class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
    {
      /// \brief Get the name of this joint
      public: std::string Name() const;
    };

    /// \brief This class is inherited by physics plugin classes that want to
    /// provide this feature.
    template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: virtual std::string GetEngineName(std::size_t _id) const = 0;

      public: virtual Identity GetWorldByName(
          std::size_t _engineId, const std::string &_name) const = 0;

      public: virtual std::string GetWorldName(std::size_t _id) const = 0;

      public: virtual Identity GetModelByName(
          std::size_t _worldId, const std::string &_name) const = 0;

      public: virtual std::string GetModelName(std::size_t _id) const = 0;

      public: virtual Identity GetLinkByName(
          std::size_t _modelId, const std::string &_name) const = 0;

      public: virtual std::string GetLinkName(std::size_t _id) const = 0;

      public: virtual Identity GetJointByName(
          std::size_t _modelId, const std::string &_name) const = 0;

      public: virtual std::string GetJointName(std::size_t _id) const = 0;
    };
  };


  // ---------------------- Implementations ----------------------

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  std::string MockGetByName::Engine<PolicyT, FeaturesT>::Name() const
  {
    return this->template Interface<MockGetByName>()->GetEngineName(
          this->identity);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto MockGetByName::Engine<PolicyT, FeaturesT>::GetWorld(
      const std::string &_name) -> WorldPtr
  {
    return WorldPtr(this->pimpl,
          this->template Interface<MockGetByName>()->GetWorldByName(
                      this->identity, _name));
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto MockGetByName::Engine<PolicyT, FeaturesT>::GetWorld(
      const std::string &_name) const -> ConstWorldPtr
  {
    return const_cast<Engine*>(this)->GetWorld(_name);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  std::string MockGetByName::World<PolicyT, FeaturesT>::Name() const
  {
    return this->template Interface<MockGetByName>()->GetWorldName(
          this->identity);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto MockGetByName::World<PolicyT, FeaturesT>::GetModel(
      const std::string &_name) -> ModelPtr
  {
    return ModelPtr(this->pimpl,
          this->template Interface<MockGetByName>()->GetModelByName(
                     this->identity, _name));
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto MockGetByName::World<PolicyT, FeaturesT>::GetModel(
      const std::string &_name) const -> ConstModelPtr
  {
    return const_cast<World*>(this)->GetModel(_name);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  std::string MockGetByName::Model<PolicyT, FeaturesT>::Name() const
  {
    return this->template Interface<MockGetByName>()->GetModelName(
          this->identity);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto MockGetByName::Model<PolicyT, FeaturesT>::GetLink(
      const std::string &_name) -> LinkPtr
  {
    return LinkPtr(this->pimpl,
          this->template Interface<MockGetByName>()->GetLinkByName(
                     this->identity, _name));
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto MockGetByName::Model<PolicyT, FeaturesT>::GetLink(
      const std::string &_name) const -> ConstLinkPtr
  {
    return const_cast<Model*>(this)->GetLink(_name);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto MockGetByName::Model<PolicyT, FeaturesT>::GetJoint(
      const std::string &_name) -> JointPtr
  {
    return JointPtr(this->pimpl,
          this->template Interface<MockGetByName>()->GetJointByName(
                      this->identity, _name));
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto MockGetByName::Model<PolicyT, FeaturesT>::GetJoint(
      const std::string &_name) const -> ConstJointPtr
  {
    return const_cast<Model*>(this)->GetJoint(_name);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  std::string MockGetByName::Link<PolicyT, FeaturesT>::Name() const
  {
    return this->template Interface<MockGetByName>()->GetLinkName(
          this->identity);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  std::string MockGetByName::Joint<PolicyT, FeaturesT>::Name() const
  {
    return this->template Interface<MockGetByName>()->GetJointName(
          this->identity);
  }
}

#endif
