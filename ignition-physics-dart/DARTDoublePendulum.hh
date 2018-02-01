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

#ifndef IGNITION_PHYSICS_DARTDOUBLEPENDULUM_HH_
#define IGNITION_PHYSICS_DARTDOUBLEPENDULUM_HH_

#include <memory>

#include "DoublePendulum.hh"

namespace ignition
{
  namespace physics
  {
    namespace dart
    {
      class PrivateDARTDoublePendulum;

      class DARTDoublePendulum
          : public virtual ignition::physics::DoublePendulum
      {
        public: virtual ~DARTDoublePendulum();

        public: DARTDoublePendulum();

        public: void Step(Output &h, ForwardStep::State &x,
                          const Input &u) override;

        public: void SetStateTo(const SetState::State &x) override;

        public: void Write(JointPositions &positions) const override;

        public: void Write(WorldPoses &poses) const override;

        private: std::unique_ptr<PrivateDARTDoublePendulum> dataPtr;
      };
    }
  }
}

#endif
