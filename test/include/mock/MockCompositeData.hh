/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_TEST_MOCKCOMPOSITEDATA_HH_
#define GZ_PHYSICS_TEST_MOCKCOMPOSITEDATA_HH_

#include <gz/physics/CompositeData.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/FeaturePolicy.hh>

namespace mock
{
  using ExtraContactData =
      gz::physics::GetContactsFromLastStepFeature::ExtraContactDataT<
          gz::physics::FeaturePolicy3d>;

  struct MockCompositeDataFeature : public gz::physics::Feature
  {
    template <typename PolicyT, typename FeaturesT>
    class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
    {
      public: gz::physics::CompositeData GetCompositeData() const
      {
        return this->template Interface<MockCompositeDataFeature>()->
            GetCompositeData();
      }
    };

    template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: virtual gz::physics::CompositeData GetCompositeData() const = 0;
    };
  };

  using MockCompositeDataList = gz::physics::FeatureList<
      MockCompositeDataFeature
  >;
}

#endif
