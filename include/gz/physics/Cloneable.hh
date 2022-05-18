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

#ifndef GZ_PHYSICS_CLONEABLE_HH_
#define GZ_PHYSICS_CLONEABLE_HH_

#include <memory>

namespace gz
{
  namespace physics
  {
    /// \brief This class allows us to effectively perform type erasure while
    /// still being able to copy, move, and clone the values of objects.
    /// \private
    class Cloneable
    {
      /// \brief Default constructor
      public: Cloneable() = default;

      /// \brief Virtual destructor
      public: virtual ~Cloneable() = default;

      /// \brief Do not copy this class directly, use Clone() or Copy() instead.
      /// \param[in] _doNotCopy Not used.
      public: Cloneable(const Cloneable &_doNotCopy) = delete;

      /// \brief Do not copy this class directly, use Clone() or Copy() instead.
      /// \param[in] _doNotCopy Not used.
      public: Cloneable& operator=(const Cloneable &_doNotCopy) = delete;

      /// \brief Implement this function to allow your Cloneable type to be
      /// cloned safely.
      /// \return Return a pointer to a cloned version of this object.
      public: virtual std::unique_ptr<Cloneable> Clone() const = 0;

      /// \brief Implement this function to allow your Cloneable type to copy
      /// another Cloneable object. Assume that the fully-derived type of other
      /// is the same as the fully-derived type of this object (i.e. you may use
      /// static_cast instead of dynamic_cast).
      /// \param[in] _other Instance to copy into this object.
      public: virtual void Copy(const Cloneable &_other) = 0;

      /// \brief Implement this function to allow your Cloneable type to move
      /// data from another Cloneable object. Assume that the fully-derived
      /// type of other is the same as the fully-derived type of this object.
      /// \param[in] _other Instance to move into this object.
      public: virtual void Copy(Cloneable &&_other) = 0;
    };

    /// \brief Assuming the type T follows the Rule of Five or the Rule of Zero
    /// (see http://en.cppreference.com/w/cpp/language/rule_of_three), this
    /// class creates a type that will implement the Copy and Clone functions
    /// for it.
    ///
    /// \warning In order to minimize overhead, this class does not do any type
    /// safety checks. It should only be used by a class like CompositeData
    /// which takes responsibility for ensuring type safety.
    /// \private
    template <typename T>
    class MakeCloneable final : public T, public Cloneable
    {
      /// \brief Perfect-forwarding constructor
      /// \param[in] _args
      ///   Parameters which will be perfectly forwarded to the constructor of T
      public: template <typename... Args>
      //   This constructor is not marked as explicit because we want it to
      //   allow implicit conversions by design
      MakeCloneable(Args&&... _args); // NOLINT

      /// \brief Copy constructor
      /// \param[in] _other
      ///   Another MakeCloneable<T> object which we will copy
      public: MakeCloneable(const MakeCloneable &_other);

      /// \brief Move constructor
      /// \param[in] _other
      ///   An rvalue-reference to a MakeCloneable<T> object which we will
      ///   consume
      public: MakeCloneable(MakeCloneable &&_other);

      /// \brief Copy operator
      /// \param[in] _other
      ///   Another MakeCloneable<T> object which we will copy
      /// \return A reference to this object
      public: MakeCloneable &operator=(const MakeCloneable &_other);

      /// \brief Move operator
      /// \param[in] _other
      ///   An rvalue-reference to a MakeCloneable<T> object which we will
      ///   consume
      /// \return A reference to this object
      public: MakeCloneable& operator=(MakeCloneable &&_other);

      // Documentation inherited
      public: std::unique_ptr<Cloneable> Clone() const final;

      // Documentation inherited
      public: void Copy(const Cloneable &_other) final;

      // Documentation inherited
      public: void Copy(Cloneable &&_other) final;
    };
  }
}

#include "gz/physics/detail/Cloneable.hh"

#endif
