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

#include <ignition/physics/FrameID.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    #define DETAIL_IMPLEMENT_FRAMEID_COMPARE(op) \
    bool FrameID::operator op (const FrameID &_other) const \
    { \
      return (this->id op _other.id); \
    }

    DETAIL_IMPLEMENT_FRAMEID_COMPARE( == ) // NOLINT
    DETAIL_IMPLEMENT_FRAMEID_COMPARE( < )  // NOLINT
    DETAIL_IMPLEMENT_FRAMEID_COMPARE( > )  // NOLINT
    DETAIL_IMPLEMENT_FRAMEID_COMPARE( <= ) // NOLINT
    DETAIL_IMPLEMENT_FRAMEID_COMPARE( >= ) // NOLINT
    DETAIL_IMPLEMENT_FRAMEID_COMPARE( != ) // NOLINT

    /////////////////////////////////////////////////
    const FrameID &FrameID::World()
    {
      static const FrameID world(ConstructWorld);
      return world;
    }

    /////////////////////////////////////////////////
    std::size_t FrameID::ID() const
    {
      return id;
    }

    /////////////////////////////////////////////////
    bool FrameID::IsWorld() const
    {
      return this->id == World().id;
    }

    /////////////////////////////////////////////////
    bool FrameID::IsReferenceCounted() const
    {
      if (World().id == this->id)
      {
        // The world frame does not require reference counting, so it provides
        // all the same safety guarantees as if it were being reference counted.
        // Therefore we return true, even though the world frame is not
        // technically reference counted.
        return true;
      }

      if (this->ref)
        return true;

      return false;
    }

    /////////////////////////////////////////////////
    FrameID::FrameID(const Identity &_identity)
      : id(_identity.id),
        ref(_identity.ref)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    FrameID::FrameID(WorldConstructorArg)
      : id(0),
        ref(nullptr)
    {
      // Do nothing
    }
  }
}
