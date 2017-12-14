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

#ifndef IGNITION_PHYSICS_MINIMALSCENES_HH_
#define IGNITION_PHYSICS_MINIMALSCENES_HH_

#include <memory>

#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/CanWriteData.hh>

namespace ignition
{
  namespace physics
  {
    namespace none
    {
      /// \brief Minimal example of an ignition physics plugin
      /// whose state is a vector of world poses that remain constant
      /// during ForwardStep calls.
      class PrivateMinimalStaticScene;
      class MinimalStaticScene
          : public virtual ignition::physics::ForwardStep,
            public virtual ignition::physics::SetState,
            public ignition::physics::CanWriteRequiredData<
                MinimalStaticScene,
                ignition::physics::ForwardStep::Output>
      {
        public: virtual ~MinimalStaticScene();

        public: MinimalStaticScene();

        public: void Step(Output &_h, ForwardStep::State &_x,
                          const Input &_u) override;

        public: void SetStateTo(const SetState::State &_x) override;

        public: void Write(ignition::physics::WorldPoses &_poses) const;

        private: std::unique_ptr<PrivateMinimalStaticScene> dataPtr;
      };

      /// \brief Minimal example of an ignition physics plugin
      /// whose state is a vector of world poses that subtract 1.0
      /// from the Z position during each ForwardStep call.
      class PrivateMinimalFallingScene;
      class MinimalFallingScene
          : public virtual ignition::physics::ForwardStep,
            public virtual ignition::physics::SetState,
            public ignition::physics::CanWriteRequiredData<
                MinimalFallingScene,
                ignition::physics::ForwardStep::Output>
      {
        public: virtual ~MinimalFallingScene();

        public: MinimalFallingScene();

        public: void Step(Output &_h, ForwardStep::State &_x,
                          const Input &_u) override;

        public: void SetStateTo(const SetState::State &_x) override;

        public: void Write(ignition::physics::WorldPoses &_poses) const;

        private: std::unique_ptr<PrivateMinimalFallingScene> dataPtr;
      };
    }
  }
}

#endif
