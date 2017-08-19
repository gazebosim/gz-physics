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

#ifndef IGNITION_PHYSICS_CLONEABLE_HH_
#define IGNITION_PHYSICS_CLONEABLE_HH_

#include <memory>

namespace ignition
{
  namespace physics
  {
    class Cloneable
    {
      /// \brief Default constructor
      public: Cloneable() = default;

      /// \brief Virtual destructor
      public: virtual ~Cloneable() = default;

      /// \brief Do not copy this class directly, use Clone() or Copy() instead.
      public: Cloneable(const Cloneable &doNotCopy) = delete;

      /// \brief Do not copy this class directly, use Clone() or Copy() instead.
      public: Cloneable& operator=(const Cloneable &doNotCopy) = delete;

      /// \brief Implement this function to allow your Cloneable type to be
      /// cloned safely.
      public: virtual std::unique_ptr<Cloneable> Clone() const = 0;

      /// \brief Implement this function to allow your Cloneable type to copy
      /// another Cloneable object. Assume that the fully-derived type of other
      /// is the same as the fully-derived type of this object (i.e. you may use
      /// static_cast instead of dynamic_cast).
      public: virtual void Copy(const Cloneable &other) = 0;

      /// \brief Implement this function to allow your Cloneable type to move
      /// data from another Cloneable object. Assume that the fully-derived
      /// type of other is the same as the fully-derived type of this object.
      public: virtual void Copy(Cloneable &&other) = 0;
    };

    /// \brief Assuming the type T follows the Rule of Five or the Rule of Zero,
    /// create a type that will implement the Copy and Clone functions for it.
    template <typename T>
    class MakeCloneable final : public T, public Cloneable
    {
      /// \brief Perfect-forwarding constructor
      public: template <typename... Args>
      MakeCloneable(Args&&... args);

      /// \brief Copy constructor
      public: MakeCloneable(const MakeCloneable &other);

      /// \brief Move constructor
      public: MakeCloneable(MakeCloneable &&other);

      /// \brief Copy operator
      public: MakeCloneable& operator=(const MakeCloneable &other);

      /// \brief Move operator
      public: MakeCloneable& operator=(MakeCloneable &&other);

      // Documentation inherited
      public: std::unique_ptr<Cloneable> Clone() const override final;

      // Documentation inherited
      public: void Copy(const Cloneable &other) override final;

      // Documentation inherited
      public: void Copy(Cloneable &&other) override final;
    };
  }
}

#include "ignition/physics/detail/Cloneable.hh"

#endif
