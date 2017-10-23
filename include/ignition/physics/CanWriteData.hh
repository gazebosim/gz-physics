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

#ifndef IGNITION_PHYSICS_CANWRITEDATA_HH_
#define IGNITION_PHYSICS_CANWRITEDATA_HH_

#include "ignition/physics/OperateOnSpecifiedData.hh"

namespace ignition
{
  namespace physics
  {
    /// \brief A struct that defines options for writing data to a CompositeData
    /// object.
    struct WriteOptions
    {
      /// \brief If a data type is not already part of the CompositeData, then
      /// skip it instead of writing to it when this is true. When this is
      /// false, create a new instance using the default constructor for any
      /// data type that is missing, and then hand it over for writing.
      public: bool skipMissingData;

      /// \brief If a data type has already been queried, do not perform the
      /// write operation on it.
      public: bool onlyWriteUnqueriedData;

      /// \brief Default constructor: New instances will be
      /// default-constructed for any data types that are missing, and we will
      /// only hand over unqueried data types for writing.
      public: explicit WriteOptions(const bool _skipMissing = false,
                                    const bool _onlyUnqueried = true);
    };

    /// \brief CanWriteRequiredData provides compile-time static analysis to
    /// ensure that the inheriting class provides a Write(~) function overload
    /// for each of the data types that are listed as required in the
    /// Specification. It also provides a function that will invoke Write(~) on
    /// each of the required data types (you may indicate whether or not it
    /// should only invoke them on unqueried data).
    ///
    /// Note that this is a CRTP class, so you must pass the name of your class
    /// in as the first template argument when inheriting it, e.g.:
    ///
    ///     class MyClass : public CanWriteRequiredData<MyClass, MySpecification>
    ///     {
    ///       // ... define my class ...
    ///     };
    ///
    /// You may also use CanWriteExpectedData if you want to further guarantee
    /// that your class is able to read all the expected data (recommended).
    ///
    /// Note that you are allowed to inherit both CanWriteRequiredData and
    /// CanWriteExpectedData while passing different Specifications to each, but
    /// you should be aware that WriteRequiredData will only handle the data
    /// required by the Specification that is given to CanWriteRequiredData.
    /// Likewise, CanWriteExpectedData will only handle to the data expected by
    /// the Specification that is given to CanWriteExpectedData.
    ///
    /// While you can technically inherit CanWriteRequiredData multiple times
    /// and provide each base with a different Specification, this is strongly
    /// discouraged because then you will be left with multiple ambiguous
    /// versions of WriteRequiredData(~). Instead, it is beter to inherit it
    /// once and combine the Specifications using SpecifyData<Specification1,
    /// Specification2, ...>. For example:
    ///
    ///     class MyClass : public CanWriteRequiredData<
    ///           SpecifyData<MySpecification1,
    ///                       MySpecification2,
    ///                       MySpecification3> >
    ///     {
    ///       // ... define my class ...
    ///     };
    template <typename Derived, typename Specification>
    class CanWriteRequiredData
    {
      /// \brief The ability to compile this constructor ensures that an
      /// inherited class has functions that can read each of the data types
      /// required by the Specification.
      public: CanWriteRequiredData();

      /// \brief Call this function to read all the types in _data that are
      /// listed as required in the Specification.
      public: template <typename CompositeType>
      void WriteRequiredData(
          CompositeType &_data,
          const WriteOptions &_options = WriteOptions()) const;
    };

    /// CanWriteExpectedData provides compile-time static analysis to ensure that
    /// the inheriting class provides a Write(~) function overload for each of
    /// the data types that are listed as expected in the Specification. It also
    /// provides a function that will invoke Write(~) on each of the expected
    /// data types (you may indicate whether or not it should only invoke them
    /// on unqueried data).
    ///
    /// Note that this is a CRTP class, so you must pass the name of your class
    /// in as the first template argument when inheriting it, e.g.:
    ///
    ///     class MyClass : public CanWriteExpectedData<MyClass, MySpecification>
    ///     {
    ///       // ... define my class ...
    ///     };
    ///
    /// You may instead use CanWriteRequiredData if you only want to guarantee
    /// that your class is able to read the required data.
    ///
    /// Note that you are allowed to inherit both CanWriteRequiredData and
    /// CanWriteExpectedData while passing different Specifications to each, but
    /// you should be aware that WriteRequiredData will only handle the data
    /// required by the Specification that is given to CanWriteRequiredData.
    /// Likewise, CanWriteExpectedData will only handle to the data expected by
    /// the Specification that is given to CanWriteExpectedData.
    ///
    /// While you can technically inherit CanWriteRequiredData multiple times
    /// and provide each base with a different Specification, this is strongly
    /// discouraged because then you will be left with multiple ambiguous
    /// versions of WriteExpectedData(~). Instead, it is beter to inherit it once
    /// and combine the Specifications using SpecifyData<Specification1,
    /// Specification2, ...>. For example:
    ///
    ///     class MyClass : public CanWriteExpectedData<
    ///           SpecifyData<MySpecification1,
    ///                       MySpecification2,
    ///                       MySpecification3> >
    ///     {
    ///       // ... define my class ...
    ///     };
    template <typename Derived, typename Specification>
    class CanWriteExpectedData
    {
      /// The ability to compile this constructor ensures that an inherited
      /// class has functions that can read each of the data types expected by
      /// the Specification.
      public: CanWriteExpectedData();

      /// Call this function to read all the types in _data that are listed as
      /// expected in the Specification. Setting _onlyWriteUnqueriedData to true
      /// will make it so that only data entries that have not been queried will
      /// be passed to the Write(~) function. If _onlyWriteUnqueriedData is false,
      /// then all data that the Specification lists as expected will be read.
      public: template <typename CompositeType>
      void WriteExpectedData(
          CompositeType &_data,
          const WriteOptions &_options = WriteOptions()) const;
    };
  }
}

#include "ignition/physics/detail/CanWriteData.hh"

#endif
