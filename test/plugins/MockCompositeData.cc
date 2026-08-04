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

#include "mock/MockCompositeData.hh"
#include <gz/physics/Register.hh>

namespace mock
{
  template <typename PolicyT>
  class CompositeDataPlugin
      : public gz::physics::Implements<PolicyT, MockCompositeDataList>
  {
    public: gz::physics::Identity InitiateEngine(
        std::size_t /*_engineID*/) override
    {
      return this->GenerateIdentity(0);
    }

    public: gz::physics::CompositeData GetCompositeData() const override
    {
      gz::physics::CompositeData data;
      auto &extra = data.Get<ExtraContactData>();
      extra.depth = 42.0;
      return data;
    }
  };

  class CompositeDataPlugin3d
      : public CompositeDataPlugin<gz::physics::FeaturePolicy3d> { };

  GZ_PHYSICS_ADD_PLUGIN(
      CompositeDataPlugin3d,
      gz::physics::FeaturePolicy3d,
      MockCompositeDataList)
}
