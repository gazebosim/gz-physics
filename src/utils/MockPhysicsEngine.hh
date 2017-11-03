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

#ifndef IGNITION_PHYSICS_SRC_UTILS_MOCKPHYSICSENGINE_HH_
#define IGNITION_PHYSICS_SRC_UTILS_MOCKPHYSICSENGINE_HH_

#include <memory>
#include <vector>

#include <ignition/physics/FrameData.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    class MockLink
    {
      public: MockLink(
        const std::string &_name,
        const FrameData3d &_data)
        : name(_name),
          data(_data)
      {
        // Do nothing
      }

      public: std::string name;
      public: FrameData<_Scalar, _Dim> data;
    };

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    class MockJoint
    {
      public: MockJoint(
        const std::string &_name,
        const FrameData3d &_data)
        : name(_name),
          data(_data)
      {

      }

      public: std::string name;
      public: FrameData<_Scalar, _Dim> data;
    };

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim,
              typename InterfaceLink, typename InterfaceJoint>
    class MockPhysicsEngine
    {

      /// \brief A map from a unique object ID to the corresponding engine link
      /// (if it still exists).
      protected: std::vector<std::weak_ptr<MockLink>> idToEngineLink;

    };
  }
}

#endif
