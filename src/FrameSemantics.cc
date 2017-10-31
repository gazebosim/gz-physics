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

#include <cassert>

#include <ignition/common/Console.hh>

#include <ignition/physics/FrameSemantics.hh>
#include "BasicObjectImplementation.hh"

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    FrameID FrameSemantics::SpawnFrameID(
        const std::size_t _id,
        const std::shared_ptr<const void> &_ref) const
    {
      return FrameID(_id, _ref);
    }

    /////////////////////////////////////////////////
    class FrameSemantics::Object::Implementation
    {
      public: Implementation(
        const FrameSemantics *const _fs)
        : fs(_fs)
      {
        if (nullptr == fs)
        {
          ignerr << "[FrameSemantics::Object::Object] Attempting to initialize "
                 << "an object with FrameSemantics, but that engine feature is "
                 << "not available. This error message should not be possible. "
                 << "Please report this as a bug!\n";
          assert(false);
        }
      }

      public: const FrameSemantics *const fs;
    };

    /////////////////////////////////////////////////
    FrameID FrameSemantics::Object::GetFrameID() const
    {
      return this->pimpl->fs->SpawnFrameID(
            this->BasicObject::pimpl->id,
            this->BasicObject::pimpl->ref);
    }

    /////////////////////////////////////////////////
    FrameData3d FrameSemantics::Object::FrameDataRelativeTo(
        const FrameID &_relativeTo) const
    {
      return this->FrameDataRelativeTo(_relativeTo, _relativeTo);
    }

    /////////////////////////////////////////////////
    FrameData3d FrameSemantics::Object::FrameDataRelativeTo(
        const FrameID &_relativeTo,
        const FrameID &_inCoordinatesOf) const
    {
      return this->pimpl->fs->Resolve(
            RelativeFrameData3d(this->GetFrameID()),
              _relativeTo,
              _inCoordinatesOf);
    }

    /////////////////////////////////////////////////
    FrameSemantics::Object::Object()
      : pimpl(new Implementation(
                dynamic_cast<FrameSemantics *const>(
                  this->BasicObject::pimpl->features)))
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    FrameSemantics::Object::~Object()
    {
      // This destructor definition must be in a source file for PIMPL to work
    }
  }
}
