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

#ifndef IGNITION_PHYSICS_DART_DOUBLEPENDULUM_HH_
#define IGNITION_PHYSICS_DART_DOUBLEPENDULUM_HH_

#include <memory>

#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/CanWriteData.hh>

namespace ignition
{
  namespace physics
  {
    namespace dart
    {
      class PrivateDoublePendulum;

      class DoublePendulum
          : public virtual ignition::physics::ForwardStep,
            public virtual ignition::physics::SetState,
            public ignition::physics::CanWriteRequiredData<
                DoublePendulum,
                ignition::physics::ForwardStep::Output>
      {
        public: virtual ~DoublePendulum();

        public: DoublePendulum();

        public: void Step(Output &h, ForwardStep::State &x,
                          const Input &u) override;

        public: void SetStateTo(const SetState::State &x) override;

        public: void Write(ignition::physics::JointPositions &positions) const;

        public: void Write(ignition::physics::WorldPoses &poses) const;

        private: std::unique_ptr<PrivateDoublePendulum> dataPtr;
      };
    }
  }
}

#endif
