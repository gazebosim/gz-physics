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

#ifndef GZ_PHYSICS_DETAIL_SPECIFYDATA_HH_
#define GZ_PHYSICS_DETAIL_SPECIFYDATA_HH_

#include <memory>
#include <utility>

#include "gz/physics/SpecifyData.hh"

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename Expected>
    ExpectData<Expected>::ExpectData()
      : CompositeData(),
        privateExpectData(
          this->dataMap.insert(
            std::make_pair(typeid(Expected).name(),
                           CompositeData::DataEntry())).first)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    ExpectData<Expected>::ExpectData(const ExpectData<Expected> &)
        : ExpectData()
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    Data &ExpectData<Expected>::Get()
    {
      return this->ExpectData<Expected>::privateExpectData
          .Get(this, detail::type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data, typename... Args>
    auto ExpectData<Expected>::InsertOrAssign(Args&&... _args)
        -> InsertResult<Data>
    {
      return this->ExpectData<Expected>::privateExpectData
          .InsertOrAssign(this, detail::type<Data>(),
                          std::forward<Args>(_args)...);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data, typename... Args>
    auto ExpectData<Expected>::Insert(Args&&... _args) -> InsertResult<Data>
    {
      return this->ExpectData<Expected>::privateExpectData
          .Insert(this, detail::type<Data>(), std::forward<Args>(_args)...);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    bool ExpectData<Expected>::Remove()
    {
      return this->ExpectData<Expected>::privateExpectData
          .Remove(this, detail::type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    Data *ExpectData<Expected>::Query(const QueryMode _mode)
    {
      return this->ExpectData<Expected>::privateExpectData
          .Query(this, detail::type<Data>(), _mode);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    const Data *ExpectData<Expected>::Query(const QueryMode _mode) const
    {
      return this->ExpectData<Expected>::privateExpectData
          .Query(this, detail::type<Data>(), _mode);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    bool ExpectData<Expected>::Has() const
    {
      return this->ExpectData<Expected>::privateExpectData
          .Has(this, detail::type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    CompositeData::DataStatus ExpectData<Expected>::StatusOf() const
    {
      return this->ExpectData<Expected>::privateExpectData
          .StatusOf(this, detail::type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    bool ExpectData<Expected>::Unquery() const
    {
      return this->ExpectData<Expected>::privateExpectData
          .Unquery(this, detail::type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data, typename... Args>
    Data &ExpectData<Expected>::MakeRequired(Args&&... _args)
    {
      return this->ExpectData<Expected>::privateExpectData
          .MakeRequired(
            this, detail::type<Data>(), std::forward<Args>(_args)...);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    bool ExpectData<Expected>::Requires() const
    {
      return this->ExpectData<Expected>::privateExpectData
          .Requires(this, detail::type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    constexpr bool ExpectData<Expected>::Expects()
    {
      return detail::PrivateExpectData<Expected>::Expects(detail::type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Required>
    RequireData<Required>::RequireData()
      : CompositeData(),
        ExpectData<Required>()
    {
      CompositeData::DataEntry &entry =
          this->ExpectData<Required>::privateExpectData
            .expectedIterator->second;

      // Create the required data in its designated map entry, and mark it as
      // required for runtime checking.
      entry.data = std::unique_ptr<Cloneable>(new MakeCloneable<Required>());
      entry.required = true;
      ++CompositeData::numEntries;
    }

    /////////////////////////////////////////////////
    template <typename Required>
    template <typename Data>
    const Data &RequireData<Required>::Get() const
    {
      const CompositeData::MapOfData::iterator &it =
          this->ExpectData<Required>::privateExpectData
            .expectedIterator;

      return this->RequireData<Required>::privateRequireData.Get(
            this, it, detail::type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Required>
    template <typename Data>
    constexpr bool RequireData<Required>::AlwaysRequires()
    {
      return detail::PrivateRequireData<Required>::AlwaysRequires(
            detail::type<Data>());
    }

    namespace detail
    {
      /// The following templates deal with finding leaf specifiers within a
      /// specification tree. An example of a leaf specifier is ExpectData<T>
      /// or RequireData<T>. Casting a complex specification to its relevant
      /// leaf specifier allows us to utilize the extremely high-speed access
      /// functions provided by the leaf specifier.

      // Forward declaration
      template <typename Data, typename Specification,
                template<typename, typename> class Condition>
      struct SelectSpecifierIfAvailable;

      /// \brief Default definition for this template. This definition will be
      /// invoked when S1 is true and some sub-specifications exist.
      ///
      /// The fact that S1 is true means that SubSpec1 met the conditions for
      /// the type of specifier that we are looking for. Therefore, we will
      /// branch into S1 and search for the exact leaf specifier that we want.
      template <typename Data, typename Specification,
                template<typename, typename> class Condition,
                typename SubSpec1, typename SubSpec2, bool S1>
      struct SelectSpecifierIfAvailableImpl
      {
        using Specifier = typename SelectSpecifierIfAvailable<
                Data, SubSpec1, Condition>::Specifier;
      };

      /// \brief Specialized definition which is invoked when S1 is false and
      /// some sub-specifications exist.
      ///
      /// The fact that S1 is false means that it would be futile to branch
      /// into SubSpec1. Therefore, we will instead branch into SubSpec2 in the
      /// hopes that it contains the specification that we want (even though it
      /// might not actually contain what we want).
      template <typename Data, typename Specification,
                template<typename, typename> class Condition,
                typename SubSpec1, typename SubSpec2>
      struct SelectSpecifierIfAvailableImpl<
          Data, Specification, Condition, SubSpec1, SubSpec2, false>
      {
        using Specifier = typename SelectSpecifierIfAvailable<
                Data, SubSpec2, Condition>::Specifier;
      };

      /// \brief Specialized definition which is invoked when we've reached a
      /// leaf specification. Either this is the specification that we want, or
      /// else the specification that we want does not exist in the original
      /// specification.
      template <typename Data, typename Specification,
                template<typename, typename> class Condition>
      struct SelectSpecifierIfAvailableImpl<
          Data, Specification, Condition, void, void, false>
      {
        using Specifier = Specification;
      };

      /// \brief Find the leaf specifier within Specification which meets the
      /// Condition for Data. If Specification is not able to satisfy the
      /// Condition, then this will simply provide the last leaf specifier
      /// within Specification, whatever that may be.
      template <typename Data, typename Specification,
                template<typename, typename> class Condition>
      struct SelectSpecifierIfAvailable
      {
        using Specifier = typename SelectSpecifierIfAvailableImpl<
            Data, Specification, Condition,
            typename Specification::SubSpecification1,
            typename Specification::SubSpecification2,
            Condition<Data, typename Specification::SubSpecification1>::value>
          ::Specifier;
      };

      /// \brief A version of SelectSpecifierIfAvailable that returns the leaf
      /// specifier which expects Data.
      template <typename Data, typename Specification>
      struct SelectExpectorIfAvailable
      {
        using Expector = typename SelectSpecifierIfAvailable<
                Data, Specification, IsExpectedBy>::Specifier;
      };

      /// \brief A version of SelectSpecifierIfAvailable that returns the leaf
      /// specifier which requires Data
      template <typename Data, typename Specification>
      struct SelectRequirerIfAvailable
      {
        using Requirer = typename SelectSpecifierIfAvailable<
                Data, Specification, IsRequiredBy>::Specifier;
      };
    }

    /////////////////////////////////////////////////
    /// \private
    template <typename DataSpec>
    class SpecifyData<DataSpec> : public virtual DataSpec
    {
      // We provide this template specialization in case someone
      // wants to use the SpecifyData<~> format with only one specification,
      // e.g.:
      //
      // using MySpecification = SpecifyData<RequireData<MyData>>;

      /// \brief Virtual destructor
      public: virtual ~SpecifyData() = default;
    };

    /// \brief Create a function which casts this to the relevant leaf specifier
    /// and then invokes the requested function on it.
    #define DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH( \
      ReturnType, Function, Suffix, CastTo, Args) \
      ReturnType Function Suffix \
      { \
        using Expector = typename detail::SelectExpectorIfAvailable< \
                T, Specification>::Expector; \
        return static_cast<CastTo*>(this)->template Function <T> Args; \
      }

    /////////////////////////////////////////////////
    /// \brief Create a fork in the binary specification tree
    /// \private
    template <typename DataSpec1, typename DataSpec2>
    class SpecifyData<DataSpec1, DataSpec2> :
        public virtual DataSpec1,
        public virtual DataSpec2
    {
      public: virtual ~SpecifyData() = default;

      // These allow outside users to identify the specifications at compile
      // time, which can be useful for template metaprogramming.
      public: using Specification = SpecifyData<DataSpec1, DataSpec2>;
      public: using SubSpecification1 = DataSpec1;
      public: using SubSpecification2 = DataSpec2;

      // These allow us to use SFINAE to tunnel down into deeper specifications
      public: using ExpectedData = void;
      public: using RequiredData = void;

      template <typename T>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          T&, Get, (), Expector, ())

      template <typename T, typename... Args>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          CompositeData::InsertResult<T>, InsertOrAssign, (Args&&... args),
          Expector, (std::forward<Args>(args)...))

      template <typename T, typename... Args>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          CompositeData::InsertResult<T>, Insert, (Args&&... args),
          Expector, (std::forward<Args>(args)...))

      template <typename T>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          bool, Remove, (), Expector, ())

      template <typename T>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          T*, Query,
          (const CompositeData::QueryMode mode =
                CompositeData::QueryMode::NORMAL),
          Expector, (mode))

      template <typename T>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          const T*, Query,
          (const CompositeData::QueryMode mode =
                CompositeData::QueryMode::NORMAL) const,
          const Expector, (mode))

      template <typename T>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          bool, Has,
          () const,
          const Expector, ())

      template <typename T>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          CompositeData::DataStatus, StatusOf,
          () const,
          const Expector, ())

      template <typename T>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          bool, Unquery, () const, const Expector, ())

      template <typename T, typename... Args>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          T&, MakeRequired, (Args&&... args), Expector,
          (std::forward<Args>(args)...))

      template <typename T>
      DETAIL_GZ_PHYSICS_SPECIFYDATA_DISPATCH(
          bool, Requires, () const, const Expector, ())

      template <typename T>
      static constexpr bool Expects()
      {
        return (   DataSpec1::template Expects<T>()
                || DataSpec2::template Expects<T>());
      }

      template <typename T>
      const T &Get() const
      {
        static_assert(AlwaysRequires<T>(), IGNITION_PHYSICS_CONST_GET_ERROR);

        using Requirer = typename detail::SelectRequirerIfAvailable<
                T, Specification>::Requirer;

        return static_cast<const Requirer*>(this)->template Get<T>();
      }

      template <typename T>
      static constexpr bool AlwaysRequires()
      {
        return (   DataSpec1::template AlwaysRequires<T>()
                || DataSpec2::template AlwaysRequires<T>());
      }
    };

    /////////////////////////////////////////////////
    /// Use the SpecifyData fork to combine these expectations
    /// \private
    template <typename DataType1, typename... OtherDataTypes>
    class ExpectData<DataType1, OtherDataTypes...>
        : public virtual SpecifyData<ExpectData<DataType1>,
                                     ExpectData<OtherDataTypes...>>
    {
      public: virtual ~ExpectData() = default;
    };

    /////////////////////////////////////////////////
    /// Use the SpecifyData fork to combine these requirements
    /// \private
    template <typename DataType1, typename... OtherDataTypes>
    class RequireData<DataType1, OtherDataTypes...>
        : public virtual SpecifyData<RequireData<DataType1>,
                                     RequireData<OtherDataTypes...>>
    {
      public: virtual ~RequireData() = default;
    };

    namespace detail
    {
      /// This will get called if the specification has both a Data specified
      /// and also contains sub-specifications. In our current implementation,
      /// this never happens, but we can provide this just in case we change the
      /// implementation.
      template <typename Data, typename SubSpec1, typename SubSpec2,
                template<typename> class SpecFinder>
      struct SpecificationDataCounterImpl
      {
        public: static constexpr std::size_t Count()
        {
          return 1
              + SpecificationDataCounterImpl<
                  typename SpecFinder<SubSpec1>::Data,
                  typename SubSpec1::SubSpecification1,
                  typename SubSpec1::SubSpecification2,
                  SpecFinder>::Count()
              + SpecificationDataCounterImpl<
                  typename SpecFinder<SubSpec2>::Data,
                  typename SubSpec2::SubSpecification1,
                  typename SubSpec2::SubSpecification2,
                  SpecFinder>::Count();
        }
      };

      /// This will get called if the specification does not specify data but
      /// does provide sub-specifications
      template <typename SubSpec1, typename SubSpec2,
                template <typename> class SpecFinder>
      struct SpecificationDataCounterImpl<void, SubSpec1, SubSpec2, SpecFinder>
      {
        public: static constexpr std::size_t Count()
        {
          return 0
              + SpecificationDataCounterImpl<
                  typename SpecFinder<SubSpec1>::Data,
                  typename SubSpec1::SubSpecification1,
                  typename SubSpec1::SubSpecification2,
                  SpecFinder>::Count()
              + SpecificationDataCounterImpl<
                  typename SpecFinder<SubSpec2>::Data,
                  typename SubSpec2::SubSpecification1,
                  typename SubSpec2::SubSpecification2,
                  SpecFinder>::Count();
        }
      };

      /// This will get called if the specification specifies data but does not
      /// provide sub-specifications. We count the data once and terminate
      /// this branch.
      template <typename Data, template<typename> class SpecFinder>
      struct SpecificationDataCounterImpl<Data, void, void, SpecFinder>
      {
        public: static constexpr std::size_t Count()
        {
          return 1;
        }
      };

      /// This will get called if the specification does not specify data and
      /// also does not provide sub-specifications. We count nothing and
      /// terminate this branch.
      template <template <typename> class SpecFinder>
      struct SpecificationDataCounterImpl<void, void, void, SpecFinder>
      {
        public: static constexpr std::size_t Count()
        {
          return 0;
        }
      };
    }

    // Implementation. See gz/physics/SpecifyData.hh for description.
    template <typename Specification, template<typename> class FindSpec>
    constexpr std::size_t CountUpperLimitOfSpecifiedData()
    {
      return detail::SpecificationDataCounterImpl<
          typename FindSpec<Specification>::Data,
          typename Specification::SubSpecification1,
          typename Specification::SubSpecification2,
          FindSpec>::Count();
    }

    // Implementation. See gz/physics/SpecifyData.hh for description.
    template <typename Specification>
    constexpr std::size_t CountUpperLimitOfExpectedData()
    {
      return CountUpperLimitOfSpecifiedData<Specification, FindExpected>();
    }

    // Implementation. See gz/physics/SpecifyData.hh for description.
    template <typename Specification>
    constexpr std::size_t CountUpperLimitOfRequiredData()
    {
      return CountUpperLimitOfSpecifiedData<Specification, FindRequired>();
    }
  }
}

#endif
