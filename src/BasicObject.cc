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

#include <ignition/common/Console.hh>

#include <ignition/physics/EngineFeatures.hh>

#include "BasicObjectImplementation.hh"

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    BasicObject::Implementation::Implementation(
        Feature *const _features,
        const std::size_t _id,
        const std::shared_ptr<const void> &_ref)
      : features(_features),
        id(_id),
        ref(_ref)
    {
      if (!features)
      {
        ignerr << "[BasicObject::BasicObject] Attempting to create a "
               << "BasicObject using the default constructor. This should not "
               << "be possible! Please report this as a bug!\n";
        assert(false);
      }
    }

    /////////////////////////////////////////////////
    std::size_t BasicObject::ObjectID() const
    {
      return this->pimpl->id;
    }

    /////////////////////////////////////////////////
    std::shared_ptr<const void> BasicObject::ObjectReference() const
    {
      return this->pimpl->ref;
    }

    /////////////////////////////////////////////////
    BasicObject::BasicObject(
        Feature *const _features,
        const std::size_t _id,
        const std::shared_ptr<const void> &_ref)
      : pimpl(new Implementation(_features, _id, _ref))
    {
      // Do nothing
    }
  }
}
