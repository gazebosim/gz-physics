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

#ifndef GZ_PHYSICS_TEST_MOCKJOINTS_HH_
#define GZ_PHYSICS_TEST_MOCKJOINTS_HH_

#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/RevoluteJoint.hh>
#include <ignition/physics/FreeJoint.hh>
#include <ignition/physics/PrismaticJoint.hh>

namespace mock
{
  class MockGetJointByIndex : public ignition::physics::Feature
  {
    public: using Identity = ignition::physics::Identity;

    template <typename PolicyT, typename FeaturesT>
    class Engine :
        public virtual ignition::physics::Feature::Engine<PolicyT, FeaturesT>
    {
      public: using JointPtr = ignition::physics::JointPtr<PolicyT, FeaturesT>;
      public: using ConstJointPtr =
        ignition::physics::ConstJointPtr<PolicyT, FeaturesT>;

      public: JointPtr GetJoint(const std::size_t _id)
      {
        return JointPtr(this->pimpl,
            this->template Interface<MockGetJointByIndex>()
                              ->GetJointByIndex(_id));
      }

      public: ConstJointPtr GetJoint(const std::size_t _id) const
      {
        return ConstJointPtr(this->pimpl,
            this->template Interface<MockGetJointByIndex>()
                             ->GetJointByIndex(_id));
      }
    };

    template <typename PolicyT>
    class Implementation :
        public virtual ignition::physics::Feature::Implementation<PolicyT>
    {
      public: virtual Identity GetJointByIndex(std::size_t _index) const = 0;
    };
  };


  using MockJointList = ignition::physics::FeatureList<
    MockGetJointByIndex,
    ignition::physics::GetBasicJointState,
    ignition::physics::SetBasicJointState,
    ignition::physics::GetBasicJointProperties,
    ignition::physics::SetJointTransformFromParentFeature,
    ignition::physics::SetJointTransformToChildFeature,
    ignition::physics::SetRevoluteJointProperties,
    ignition::physics::GetRevoluteJointProperties,
    ignition::physics::GetPrismaticJointProperties,
    ignition::physics::SetPrismaticJointProperties,
    ignition::physics::SetFreeJointRelativeTransformFeature,
    ignition::physics::JointFrameSemantics
  >;

}

#endif
