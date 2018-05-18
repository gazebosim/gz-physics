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

#ifndef IGNITION_PHYSICS_CANREADDATA_HH_
#define IGNITION_PHYSICS_CANREADDATA_HH_


#include "ignition/physics/OperateOnSpecifiedData.hh"

namespace ignition
{
  namespace physics
  {
    /// \brief ReadOptions provides customization for the ReadRequiredData
    /// and ReadExpectedData functions provided by CanReadRequiredData
    /// and CanReadExpectedData.
    /// \sa CanReadExpectedData::ReadExpectedData()
    /// \sa CanReadRequiredData::ReadRequiredData()
    struct IGNITION_PHYSICS_VISIBLE ReadOptions
    {
      /// \brief If a type has already been queried, do not perform the read
      /// operation on it.
      public: bool onlyReadUnqueriedData;

      /// \brief Default constructor.
      /// \param[in] _onlyUnqueried Whether only unqueried data will be read,
      /// default to true.
      public: explicit ReadOptions(const bool _onlyUnqueried = true);
    };

    /// \brief CanReadRequiredData provides compile-time static analysis to
    /// ensure that the inheriting class provides a Read(~) function overload
    /// for each of the data types that are listed as required in the
    /// Specification. It also provides a function that will invoke Read(~) on
    /// each of the required data types (you may indicate whether or not it
    /// should only invoke them on unqueried data).
    ///
    /// Note that you must pass the name of your class in as the first template
    /// argument when inheriting it (Curiously Recurring Template Pattern CRTP),
    /// e.g.:
    ///
    /// \code
    ///     class MyClass : public CanReadRequiredData<MyClass, MySpecification>
    ///     {
    ///       // ... define my class ...
    ///     };
    /// \endcode
    ///
    /// You may also use CanReadExpectedData if you want to further guarantee
    /// that your class is able to read all the expected data (recommended).
    ///
    /// Note that you are allowed to inherit both CanReadRequiredData and
    /// CanReadExpectedData while passing different Specifications to each, but
    /// you should be aware that ReadRequiredData will only read the data
    /// required by the Specification that is given to CanReadRequiredData.
    /// Likewise, ReadExpectedData will only read the data expected by
    /// the Specification that is given to CanReadExpectedData.
    ///
    /// While you can technically inherit CanReadRequiredData multiple times
    /// and provide each base with a different Specification, this is strongly
    /// discouraged because then you will be left with multiple ambiguous
    /// versions of ReadRequiredData(~). Instead, it is beter to inherit it once
    /// and combine the Specifications using SpecifyData<Specification1,
    /// Specification2, ...>. For example:
    ///
    /// \code
    ///     class MyClass : public CanReadRequiredData<
    ///           MyClass,
    ///           SpecifyData<MySpecification1,
    ///                       MySpecification2,
    ///                       MySpecification3> >
    ///     {
    ///       // ... define my class ...
    ///     };
    /// \endcode
    template <typename Derived, typename Specification>
    class CanReadRequiredData
    {
      /// The ability to compile this constructor ensures that an inherited
      /// class has functions that can read each of the data types required by
      /// the Specification.
      public: CanReadRequiredData();

      /// Call this function to read all the types in _data that are listed as
      /// required in the Specification. Setting _options.onlyReadUnqueriedData
      /// to true will make it so that only data entries that have not been
      /// queried will be passed to the Read(~) function.
      /// If _options.onlyReadUnqueriedData is false, then all data that the
      /// Specification lists as required will be read.
      /// \param[in] _data CompositeData instance to read from.
      /// \param[in] _options ReadOptions for customizing the read operation.
      public: template <typename CompositeType>
      void ReadRequiredData(
          const CompositeType &_data,
          const ReadOptions &_options = ReadOptions());
    };

    /// CanReadExpectedData provides compile-time static analysis to ensure that
    /// the inheriting class provides a Read(~) function overload for each of
    /// the data types that are listed as expected in the Specification. It also
    /// provides a function that will invoke Read(~) on each of the expected
    /// data types (you may indicate whether or not it should only invoke them
    /// on unqueried data).
    ///
    /// Note that you must pass the name of your class in as the first template
    /// argument when inheriting it (Curiously Recurring Template Pattern CRTP),
    /// e.g.:
    ///
    /// \code
    ///     class MyClass : public CanReadExpectedData<MyClass, MySpecification>
    ///     {
    ///       // ... define my class ...
    ///     };
    /// \endcode
    ///
    /// You may instead use CanReadRequiredData if you only want to guarantee
    /// that your class is able to read the required data.
    ///
    /// Note that you are allowed to inherit both CanReadRequiredData and
    /// CanReadExpectedData while passing different Specifications to each, but
    /// you should be aware that ReadRequiredData will only read the data
    /// required by the Specification that is given to CanReadRequiredData.
    /// Likewise, ReadExpectedData will only read the data expected by
    /// the Specification that is given to CanReadExpectedData.
    ///
    /// While you can technically inherit CanReadExpectedData multiple times
    /// and provide each base with a different Specification, this is strongly
    /// discouraged because then you will be left with multiple ambiguous
    /// versions of ReadExpectedData(~). Instead, it is beter to inherit it once
    /// and combine the Specifications using SpecifyData<Specification1,
    /// Specification2, ...>. For example:
    ///
    /// \code
    ///     class MyClass : public CanReadExpectedData<
    ///           MyClass,
    ///           SpecifyData<MySpecification1,
    ///                       MySpecification2,
    ///                       MySpecification3> >
    ///     {
    ///       // ... define my class ...
    ///     };
    /// \endcode
    template <typename Derived, typename Specification>
    class CanReadExpectedData
    {
      /// The ability to compile this constructor ensures that an inherited
      /// class has functions that can read each of the data types expected by
      /// the Specification.
      public: CanReadExpectedData();

      /// Call this function to read all the types in _data that are listed as
      /// expected in the Specification. Setting _options.onlyReadUnqueriedData
      /// to true will make it so that only data entries that have not been
      /// queried will be passed to the Read(~) function. If
      /// _options.onlyReadUnqueriedData is false,
      /// then all data that the Specification lists as expected will be read.
      /// \param[in] _data CompositeData instance to read from.
      /// \param[in] _options ReadOptions for customizing the read operation.
      public: template <typename CompositeType>
      void ReadExpectedData(
          const CompositeType &_data,
          const ReadOptions &_options = ReadOptions());
    };
  }
}

#include "ignition/physics/detail/CanReadData.hh"

#endif
