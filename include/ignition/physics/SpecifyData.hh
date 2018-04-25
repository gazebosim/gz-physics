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

#ifndef IGNITION_PHYSICS_SPECIFYDATA_HH_
#define IGNITION_PHYSICS_SPECIFYDATA_HH_

#include "ignition/physics/detail/PrivateSpecifyData.hh"

namespace ignition
{
  namespace physics
  {
    /// \brief The SpecifyData class allows you to form combinations of data
    /// specifications. In other words, you can freely mix invocations to
    /// RequireData and ExpectData. Example usage:
    ///
    ///     using namespace ignition::physics;
    ///
    ///     using MyInputSpecifications = SpecifyData<
    ///         RequireData<
    ///             DesiredPositionInput,
    ///             DesiredVelocityInput>,
    ///         ExpectData<
    ///             ExternalForceInput,
    ///             ProximitySensorInput,
    ///             ForceTorqueSensorInput> >;
    ///
    /// This would define a CompositeData which is required to contain a
    /// DesiredPositionInput data structure and a DesiredVelocityInput data
    /// structure. It is also optimized for ExternalForceInput,
    /// ProximitySensorInput, and ForceTorqueSensorInput data structures, but
    /// they are optional. Whether a data structure is specified as expected or
    /// required, you will be able to get extremely high-speed access to it
    /// through an object that has the type of MyInputSpecifications.
    ///
    /// Specifications can also be composed together. For example, if there is
    /// another specification like:
    ///
    ///     using ComplianceInputSpecifications = SpecifyData<
    ///         RequireData<
    ///             ProximitySensorInput,
    ///             ComplianceParameters>,
    ///         ExpectData<
    ///             ForceTorqueSensorInput,
    ///             CameraSensorInput> >;
    ///
    /// then you can combine these specifications:
    ///
    ///     using CombinedInputSpecifications = SpecifyData<
    ///         MyInputSpecifications,
    ///         ComplianceInputSpecifications>;
    ///
    /// Note that RequireData takes precedence over ExpectData, so
    /// ProximitySensorInput will be promoted to Required when the two
    /// specifications are combined.
    template <typename... Specifications>
    class SpecifyData
    {
      public: virtual ~SpecifyData() = default;
    };

    /// \brief ExpectData is an extension of CompositeData which indicates that
    /// the composite expects to be operating on the data types listed in its
    /// template arguments (DataTypes). All of the expected types will benefit
    /// from very high-speed operations when being accessed from an object of
    /// type ExpectData<DataTypes>. The ordinary CompositeData class needs to
    /// perform a map lookup on the data type whenever one of its functions is
    /// called, but an ExpectData<T> object does not need to perform any map
    /// lookup when performing an operation on T (e.g. .Get<T>(), .Insert<T>(),
    /// .Query<T>(), .Has<T>(), etc).
    ///
    /// Note that there is no guarantee that any of the types listed in
    /// DataTypes will exist in the underlying CompositeData, but it will still
    /// provide very high-speed access for creating and querying even if it is
    /// not yet present.
    template <typename... DataTypes>
    class ExpectData
    {
      public: virtual ~ExpectData() = default;
    };

    /// \brief RequireData is an extension of ExpectData which indicates that
    /// the composite requires the existence of any data types that are listed
    /// in its template arguments (DataTypes).
    ///
    /// Each data type that is listed in DataTypes will be instantiated upon
    /// construction of the RequireData<DataTypes> object, and none of them can
    /// be removed from that object or from any reference to that object.
    ///
    /// Objects that are listed in DataTypes will also benefit from extremely
    /// high-speed accessibility when they are accessed using an object of the
    /// type RequireData<DataTypes>. This is similar to ExpectData<DataTypes>.
    ///
    /// RequireData<DataTypes> also offers a const-qualified Get<R>() function
    /// for all types, R, which are listed in DataTypes.
    template <typename... DataTypes>
    class RequireData
    {
      public: virtual ~RequireData() = default;
    };

    /// \brief Implementation of ExpectData for a single data type.
    template <typename Expected>
    class ExpectData<Expected> : public virtual CompositeData
    {
      /// \brief Default constructor
      public: ExpectData();

      /// \brief Virtual destructor
      public: virtual ~ExpectData() = default;

      /// \brief Provides extremely high-speed access to expected data types and
      /// normal access to unexpected data types.
      public: template <typename Data>
      Data& Get();

      /// \brief Provides extremely high-speed access to creating expected data
      /// types and normal access to unexpected data types.
      public: template <typename Data, typename... Args>
      InsertResult<Data> InsertOrAssign(Args&&... args);

      /// \brief Provides extremely high-speed access to getting or creating
      /// expected data types and normal access to unexpected data types.
      public: template <typename Data, typename... Args>
      InsertResult<Data> Insert(Args&&... args);

      /// \brief Provides extremely high-speed access for removing expected data
      /// types and normal access for unexpected data types.
      public: template <typename Data>
      bool Remove();

      /// \brief Provides extremely high-speed access for querying expected data
      /// types and normal access for unexpected data types.
      public: template <typename Data>
      Data* Query(const QueryMode mode = QueryMode::NORMAL);

      /// \brief Const-qualified version of Query<Data>
      public: template <typename Data>
      const Data* Query(const QueryMode mode = QueryMode::NORMAL) const;

      /// \brief Provides extremely high-speed access for checking the existence
      /// of expected data types and normal access for unexpected data types.
      public: template <typename Data>
      bool Has() const;

      /// \brief Provides extremely high-speed access to the status of expected
      /// data types and normal access for unexpected data types.
      public: template <typename Data>
      DataStatus StatusOf() const;

      /// \brief Provides extremely high-speed access for unquerying expected
      /// data types and normal access for unexpected data types.
      public: template <typename Data>
      bool Unquery() const;

      /// \brief Provides extremely high-speed access for making expected data
      /// types required and normal access for unexpected data types.
      public: template <typename Data, typename... Args>
      Data& MakeRequired(Args&&... args);

      /// \brief Provides extremely high-speed access for checking whether
      /// expected data types are required and normal access for unexpected data
      /// types.
      public: template <typename Data>
      bool Requires() const;

      /// \brief Returns true if the Data type is expected. Data types for which
      /// this is true will be able to enjoy extremely high-speed access.
      public: template <typename Data>
      static constexpr bool Expects();

      /// \brief Provides the implementation for delegating the functions
      /// provided by the ExpectData class
      protected: detail::PrivateExpectData<Expected> privateExpectData;

      // This allows outside users to identify this expected data type at
      // compile time, which can be useful for doing template metaprogramming.
      public: using Specification = ExpectData<Expected>;
      public: using ExpectedData = Expected;
      public: using RequiredData = void;

      public: using SubSpecification1 = void;
      public: using SubSpecification2 = void;
    };

    template <typename Required>
    class RequireData<Required> : public virtual ExpectData<Required>
    {
      // This allows outside users to identify this Required data type at
      // compile time, which can be useful for doing template metaprogramming.
      public: using Specification = RequireData<Required>;
      public: using RequiredData = Required;

      /// \brief Default constructor. Will initialize the Required data type
      /// using its default constructor.
      public: RequireData();

      /// \brief Virtual destructor
      public: virtual ~RequireData() = default;

      // Do not shadow the ExpectData<Required> implementation
      using ExpectData<Required>::Get;

      /// \brief Provides extremely high-speed const-qualified reference access
      /// to data types that are required.
      ///
      /// NOTE: For data types that are not required, this will throw a
      /// compilation error, and you should use Query<Data>() const instead.
      public: template <typename Data>
      const Data& Get() const;

      /// \brief Returns true if the Data type is always required by this more
      /// highly specified CompositeData type.
      public: template <typename Data>
      static constexpr bool AlwaysRequires();

      /// \brief Provides the implementation for delegating the functions
      /// provided by the RequireData class
      protected: detail::PrivateRequireData<Required> privateRequireData;
    };


    // ----------------------- Utility Templates ----------------------------

    /// \brief This provides an upper limit on the number of expected data types
    /// in a CompositeData specification. This is an upper limit because when a
    /// data type is specified multiple times within a specficiation, it will
    /// be counted multiple times. As long as each data type is only specified
    /// once, it will provide an exact count.
    ///
    /// This is a constexpr so it will be evaluated at compile-time and behaves
    /// like a constant literal.
    template <typename Specification>
    constexpr std::size_t CountUpperLimitOfExpectedData();

    /// \brief Same as CountUpperLimitOfExpectedData() except it will count
    /// required data instead.
    template <typename Specification>
    constexpr std::size_t CountUpperLimitOfRequiredData();

    /// \brief Same as CountUpperLimitOfExpectedData() except you can specify
    /// what kind of data to count using SpecFinder. SpecFinder must accept a
    /// data specification (or void) as a template argument and provide a type
    /// called Data. See FindExpected and FindRequired below for examples.
    template <typename Specification, template<typename> class SpecFinder>
    constexpr std::size_t CountUpperLimitOfSpecifiedData();


    /// \brief This allows us to specify that we are interested in expected
    /// data while performing template metaprogramming.
    template <typename Specification>
    struct FindExpected
    {
      using Data = typename Specification::ExpectedData;
    };

    /// \brief This specialization handles the terminating case where we have
    /// reached a leaf node in the specification tree.
    template <>
    struct FindExpected<void>
    {
      using Data = void;
    };


    /// \brief This allows us to specify that we are interested in required
    /// data while performing template metaprogramming.
    template <typename Specification>
    struct FindRequired
    {
      using Data = typename Specification::RequiredData;
    };

    /// \brief This specialization handles the terminating case where we have
    /// reached a leaf node in the specification tree.
    template <>
    struct FindRequired<void>
    {
      using Data = void;
    };

    /// \brief Provides a constexpr field named `value` whose value is true if
    /// and only if Data is expected by Specification
    template <typename Data, typename Specification>
    struct IsExpectedBy : std::integral_constant<
        bool, Specification::template Expects<Data>() > { };

    /// \brief This template specialization allows us to provide a false `value`
    /// when given a void Specification.
    template <typename Data>
    struct IsExpectedBy<Data, void> : std::false_type { };

    /// \brief Provides a constexpr field named `value` whose value is true if
    /// and only if Data is required by Specification
    template <typename Data, typename Specification>
    struct IsRequiredBy : std::integral_constant<
        bool, Specification::template AlwaysRequires<Data>() > { };

    /// \brief This template specialization allows us to provide a false `value`
    /// when given a void Specification.
    template <typename Data>
    struct IsRequiredBy<Data, void> : std::false_type { };
  }
}

#include "ignition/physics/detail/SpecifyData.hh"

#endif
