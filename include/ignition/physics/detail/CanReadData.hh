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

#ifndef IGNITION_PHYSICS_DETAIL_CANREADDATA_HH_
#define IGNITION_PHYSICS_DETAIL_CANREADDATA_HH_

#include "ignition/physics/CanReadData.hh"
#include <cassert>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      /// \brief ReadDataOperation allows us to use CanOperateOnSpecifiedData to
      /// call a Read(~) member function on a class which inherits from either
      /// CanReadRequiredData or CanReadExpectedData (or both).
      template <typename Data, typename Derived, typename CompositeType>
      struct ReadDataOperation
      {
        /// \brief Do the operation
        public: static void Operate(Derived *_yourClass, CompositeType &_data)
        {
          const Data * const query = _data.template Query<Data>();
          assert(query);

          /// READ CAREFULLY: If you have arrived here by way of a compiler
          /// error, then you have neglected to provide a Read(~) member
          /// function for one of the specified data types that you claimed your
          /// class is able to read. The compiler error should clearly indicate
          /// the name of the data type that you are neglecting. Simply
          /// implement a member function in your class which overloads the
          /// Read(~) function to accept the data type which the compiler is
          /// indicating.
          ///
          /// The function signature should look like:
          ///
          /// class YourClass
          /// {
          ///   public:
          ///   /* ... */
          ///     void Read(Data &data);
          ///   /* ... */
          /// };
          ///
          ///    ^^^ READ THE ABOVE EXPLANATION IF YOU CANNOT COMPILE ^^^
          _yourClass->Read(*query);
        }
      };
    }

    /////////////////////////////////////////////////
    template <typename Derived, typename Specification>
    CanReadRequiredData<Derived, Specification>::CanReadRequiredData()
    {
      /// Calling this function in the constructor guarantees that the derived
      /// class contains all the Read(~) functions that it claims to, otherwise
      /// it will not be able to compile. Note that setting the last argument
      /// to true ensures that this function does not actually do anything
      /// besides compile. Instantiating a completely generic CompositeData type
      /// and DataStatusMask is extremely low-cost, so we do not need to worry
      /// about overhead.
      OperateOnSpecifiedData<
          Specification, FindRequired, detail::ReadDataOperation,
          Derived>::template Operate<const CompositeData>(
            static_cast<Derived*>(this), CompositeData{},
            DataStatusMask{}, true);
    }

    /////////////////////////////////////////////////
    template <typename Derived, typename Specification>
    template <typename CompositeType>
    void CanReadRequiredData<Derived, Specification>::ReadRequiredData(
        const CompositeType &_data,
        const ReadOptions &_options)
    {
      DataStatusMask mask;
      if (_options.onlyReadUnqueriedData)
        mask.queried = DataStatusMask::MUST_NOT;

      mask.exist = DataStatusMask::MUST;

      OperateOnSpecifiedData<
          Specification, FindRequired, detail::ReadDataOperation,
          Derived>::Operate(static_cast<Derived*>(this), _data, mask);
    }

    /////////////////////////////////////////////////
    template <typename Derived, typename Specification>
    CanReadExpectedData<Derived, Specification>::CanReadExpectedData()
    {
      /// Calling this function in the constructor guarantees that the derived
      /// class contains all the Read(~) functions that it claims to, otherwise
      /// it will not be able to compile. Note that setting the last argument
      /// to true ensures that this function does not actually do anything
      /// besides compile. Instantiating a completely generic CompositeData type
      /// and DataStatusMask is extremely low-cost, so we do not need to worry
      /// about overhead.
      OperateOnSpecifiedData<
          Specification, FindExpected, detail::ReadDataOperation,
          Derived>::template Operate<const CompositeData>(
            static_cast<Derived*>(this), CompositeData{},
            DataStatusMask{}, true);
    }

    /////////////////////////////////////////////////
    template <typename Derived, typename Specification>
    template <typename CompositeType>
    void CanReadExpectedData<Derived, Specification>::ReadExpectedData(
        const CompositeType &_data,
        const ReadOptions &_options)
    {
      DataStatusMask mask;
      if (_options.onlyReadUnqueriedData)
        mask.queried = DataStatusMask::MUST_NOT;

      mask.exist = DataStatusMask::MUST;

      OperateOnSpecifiedData<
          Specification, FindExpected, detail::ReadDataOperation,
          Derived>::Operate(static_cast<Derived*>(this), _data, mask);
    }
  }
}

#endif
