/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/physics/Entity.hh>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      /////////////////////////////////////////////////
      Identity Implementation::GenerateIdentity(
          std::size_t _id,
          const std::shared_ptr<const void> &_ref)
      {
        return Identity(_id, _ref);
      }

      /////////////////////////////////////////////////
      Identity Implementation::GenerateInvalidId()
      {
        return Identity();
      }
    }

    /////////////////////////////////////////////////
    Identity::operator bool() const
    {
      return (INVALID_ENTITY_ID != id);
    }

    /////////////////////////////////////////////////
    Identity::operator std::size_t() const
    {
      return id;
    }

    /////////////////////////////////////////////////
    Identity::Identity()
      : id(INVALID_ENTITY_ID),
        ref(nullptr)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    Identity::Identity(
        std::size_t _id,
        const std::shared_ptr<const void> &_ref)
      : id(_id),
        ref(_ref)
    {
      // Do nothing
    }
  }
}
