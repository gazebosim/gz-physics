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

#ifndef IGNITION_PHYSICS_DETAIL_CANWRITEDATA_HH_
#define IGNITION_PHYSICS_DETAIL_CANWRITEDATA_HH_

#include "ignition/physics/CanWriteData.hh"

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      /// \brief WriteDataOperation allows us to use OperateOnSpecifiedData to
      /// call a Write(~) member function on a class which inherits from either
      /// CanWriteRequiredData or CanWriteExpectedData (or both).
      template <typename Data, typename Derived, typename CompositeType>
      struct WriteDataOperation
      {
        /// \brief WriteDataOperation::Operate is where the data writing
        /// operation gets performed.
        /// \param[in,out] _yourClass
        ///   The object which will perform the Write operation.
        /// \param[in,out] _data
        ///   The object which will be written to.
        public: static void Operate(Derived *yourClass, CompositeType &data)
        {
          /// \par
          /// \page WriteCompilationFail Failure to compile WriteDataOperation
          /// READ CAREFULLY: If you have arrived here by way of a compiler
          /// error, then you have neglected to provide a Write(~) member
          /// function for one of the specified data types that you claimed your
          /// class is able to write to. The compiler error should clearly
          /// indicate the name of the data type that you are neglecting. Simply
          /// implement a member function in your class which overloads the
          /// Write(~) function to accept the data type which the compiler is
          /// indicating.
          ///
          /// If you see an error about "const" and "qualifiers", then make sure
          /// that your class's Write(~) function is const-qualified, i.e. put
          /// the keyword const at the end of its declaration in the function
          /// definition:
          ///
          /// \code
          ///    class YourClass
          ///    {
          ///    public:
          ///       // ...
          ///       void Write(const Data &data) const;
          ///       // ...
          ///    };
          /// \endcode
          ///
          ///    ^^^ READ THE ABOVE EXPLANATION IF YOU CANNOT COMPILE ^^^
          yourClass->Write(data.template Get<Data>());
        }
      };
    }

    template <typename Derived, typename Specification>
    CanWriteRequiredData<Derived, Specification>::CanWriteRequiredData()
    {
      CompositeData dummy;
      OperateOnSpecifiedData<
          Specification, FindRequired, detail::WriteDataOperation,
          const Derived>::Operate(static_cast<const Derived*>(this), dummy,
                                  DataStatusMask{}, true);
    }

    template <typename Derived, typename Specification>
    template <typename CompositeType>
    void CanWriteRequiredData<Derived, Specification>::WriteRequiredData(
        CompositeType &_data,
        const WriteOptions &_options) const
    {
      DataStatusMask mask;

      // We've been asked to skip missing data, so we'll tell the data mask that
      // each data type we write must already exist.
      if (_options.skipMissingData)
        mask.exist = DataStatusMask::MUST;

      // We've been asked to only write unqueried data, so we'll tell the data
      // mask that each data type we write must not be already queried.
      if (_options.onlyWriteUnqueriedData)
        mask.queried = DataStatusMask::MUST_NOT;

      OperateOnSpecifiedData<
          Specification, FindRequired, detail::WriteDataOperation,
          const Derived>::Operate(static_cast<const Derived*>(this),
                                  _data, mask);
    }

    template <typename Derived, typename Specification>
    CanWriteExpectedData<Derived, Specification>::CanWriteExpectedData()
    {
      CompositeData dummy;
      OperateOnSpecifiedData<
          Specification, FindExpected, detail::WriteDataOperation,
          const Derived>::Operate(static_cast<const Derived*>(this), dummy,
                                  DataStatusMask{}, true);
    }

    template <typename Derived, typename Specification>
    template <typename CompositeType>
    void CanWriteExpectedData<Derived, Specification>::WriteExpectedData(
        CompositeType &_data,
        const WriteOptions &_options) const
    {
      DataStatusMask mask;

      if (_options.skipMissingData)
        mask.exist = DataStatusMask::MUST;

      if (_options.onlyWriteUnqueriedData)
        mask.queried = DataStatusMask::MUST_NOT;

      OperateOnSpecifiedData<
          Specification, FindExpected, detail::WriteDataOperation,
          const Derived>::Operate(static_cast<const Derived*>(this),
                                  _data, mask);
    }
  }
}

#endif
