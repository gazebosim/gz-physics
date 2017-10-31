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

#ifndef IGNITION_PHYSICS_BASICOBJECTIMPLEMENTATION_HH
#define IGNITION_PHYSICS_BASICOBJECTIMPLEMENTATION_HH

#include <ignition/physics/BasicObject.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief Implementation class for the BasicObject
    class BasicObject::Implementation
    {
      public: Implementation(
        Feature *const _features,
        const std::size_t _id,
        const std::shared_ptr<const void> &_ref);

      /// \brief This is a pointer to the physics engine, and it can be used by
      /// the object features to find the engine interfaces that they need in
      /// order to function.
      public: Feature *const features;

      /// \brief This integer ID uniquely identifies the engine object that this
      /// proxy object is referring to. No two proxy objects may use the same ID
      /// unless they are referring to the same instance of an engine object.
      ///
      /// The ID is not allowed to change at any point in the lifetime of the
      /// engine object.
      ///
      /// Note that the ID of 0 is reserved for the "world" object.
      public: const std::size_t id;

      /// \brief This is an optional reference-counting field for the proxy
      /// objects. Not all engines are required to support this field for all
      /// types, so this may be left as a nullptr.
      ///
      /// This reference is not allowed to change at any point in the lifetime
      /// of the engine object.
      public: const std::shared_ptr<const void> ref;
    };
  }
}

#endif
