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

#ifndef GZ_PHYSICS_TEST_MOCKSETNAME_HH_
#define GZ_PHYSICS_TEST_MOCKSETNAME_HH_

#include <string>

#include <ignition/physics/Feature.hh>

namespace mock
{
  struct MockSetName : public gz::physics::Feature
  {
    using Identity = gz::physics::Identity;

    template <typename PolicyT, typename FeaturesT>
    class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
    {
      /// \brief Set the name of this engine.
      /// \return true if the name was set; false if the name was already taken.
      public: bool SetName(const std::string &_desiredName);
    };

    template <typename PolicyT, typename FeaturesT>
    class World : public virtual Feature::World<PolicyT, FeaturesT>
    {
      /// \brief Set the name of this world.
      /// \return true if the name was set; false if the name was already taken.
      public: bool SetName(const std::string &_desiredName);
    };

    template <typename PolicyT, typename FeaturesT>
    class Model : public virtual Feature::Model<PolicyT, FeaturesT>
    {
      /// \brief Set the name of this model.
      /// \return true if the name was set; false if the name was already taken.
      public: bool SetName(const std::string &_desiredName);
    };

    template <typename PolicyT, typename FeaturesT>
    class Link : public virtual Feature::Link<PolicyT, FeaturesT>
    {
      /// \brief Set the name of this link.
      /// \return true if the name was set; false if the name was already taken.
      public: bool SetName(const std::string &_desiredName);
    };

    template <typename PolicyT, typename FeaturesT>
    class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
    {
      /// \brief Set the name of this joint.
      /// \return true if the name was set; false if the name was already taken.
      public: bool SetName(const std::string &_desiredName);
    };

    /// \brief This class is inherited by physics plugin classes that want to
    /// provide this feature.
    template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: virtual bool SetEngineName(
          const Identity &_id, const std::string &_desiredName) = 0;

      public: virtual bool SetWorldName(
          const Identity &_id, const std::string &_desiredName) = 0;

      public: virtual bool SetModelName(
          const Identity &_id, const std::string &_desiredName) = 0;

      public: virtual bool SetLinkName(
          const Identity &_id, const std::string &_desiredName) = 0;

      public: virtual bool SetJointName(
          const Identity &_id, const std::string &_desiredName) = 0;
    };
  };


  // ---------------------- Implementations ----------------------

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  bool MockSetName::Engine<PolicyT, FeaturesT>::SetName(
      const std::string &_desiredName)
  {
    return this->template Interface<MockSetName>()->SetEngineName(
          this->identity, _desiredName);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  bool MockSetName::World<PolicyT, FeaturesT>::SetName(
      const std::string &_desiredName)
  {
    return this->template Interface<MockSetName>()->SetWorldName(
          this->identity, _desiredName);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  bool MockSetName::Model<PolicyT, FeaturesT>::SetName(
      const std::string &_desiredName)
  {
    return this->template Interface<MockSetName>()->SetModelName(
          this->identity, _desiredName);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  bool MockSetName::Link<PolicyT, FeaturesT>::SetName(
      const std::string &_desiredName)
  {
    return this->template Interface<MockSetName>()->SetLinkName(
          this->identity, _desiredName);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  bool MockSetName::Joint<PolicyT, FeaturesT>::SetName(
      const std::string &_desiredName)
  {
    return this->template Interface<MockSetName>()->SetJointName(
          this->identity, _desiredName);
  }
}

#endif
