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

#ifndef IGNITION_PHYSICS_DOUBLEPENDULUM_HH_
#define IGNITION_PHYSICS_DOUBLEPENDULUM_HH_

#include <memory>

#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/CanWriteData.hh>

namespace ignition
{
  namespace physics
  {
    class DoublePendulum
        : public virtual ignition::physics::ForwardStep,
          public virtual ignition::physics::SetState,
          public ignition::physics::CanWriteRequiredData<
              DoublePendulum,
              ignition::physics::ForwardStep::Output>
    {
      public: virtual void Write(JointPositions &positions) const = 0;

      /// \brief Write poses for each link in inertial frame (at COM).
      public: virtual void Write(WorldPoses &poses) const = 0;

      public: virtual ~DoublePendulum() = default;
    };
  }
}

#endif
