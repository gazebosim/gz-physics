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

#ifndef IGNITION_PHYSICS_DETAIL_SPECIFYDATA_HH_
#define IGNITION_PHYSICS_DETAIL_SPECIFYDATA_HH_

#include "ignition/physics/SpecifyData.hh"

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename Expected>
    ExpectData<Expected>::ExpectData()
      : CompositeData(),
        privateExpectData(
          this->dataMap.insert(
            std::make_pair(Expected::IgnPhysicsTypeLabel(),
                           CompositeData::DataEntry())).first)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    Data& ExpectData<Expected>::Get()
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Get(this, type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data, typename... Args>
    Data& ExpectData<Expected>::Create(Args&&... args)
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Create(this, type<Data>(), std::forward<Args>(args)...);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data, typename... Args>
    Data& ExpectData<Expected>::GetOrCreate(Args&&... args)
    {
      return this->template ExpectData<Expected>::privateExpectData
          .GetOrCreate(this, type<Data>(), std::forward<Args>(args)...);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    bool ExpectData<Expected>::Remove()
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Remove(this, type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    Data* ExpectData<Expected>::Query(const QueryMode mode)
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Query(this, type<Data>(), mode);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    const Data* ExpectData<Expected>::Query(const QueryMode mode) const
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Query(this, type<Data>(), mode);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    bool ExpectData<Expected>::Has(const QueryMode mode) const
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Has(this, type<Data>(), mode);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    CompositeData::DataStatus ExpectData<Expected>::StatusOf(
        const QueryMode mode) const
    {
      return this->template ExpectData<Expected>::privateExpectData
          .StatusOf(this, type<Data>(), mode);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    bool ExpectData<Expected>::Unquery() const
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Unquery(this, type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data, typename... Args>
    Data& ExpectData<Expected>::MakeRequired(Args&&... args)
    {
      return this->template ExpectData<Expected>::privateExpectData
          .MakeRequired(
            this, type<Data>(), std::forward<Args>(args)...);
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    bool ExpectData<Expected>::Requires() const
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Requires(this, type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    constexpr bool ExpectData<Expected>::Expects()
    {
      return detail::PrivateExpectData<Expected>::Expects(type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Required>
    RequireData<Required>::RequireData()
      : CompositeData(),
        ExpectData<Required>()
    {
      CompositeData::DataEntry &entry =
          this->template ExpectData<Required>::privateExpectData
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
    const Data& RequireData<Required>::Get() const
    {
      const CompositeData::MapOfData::iterator &it =
          this->template ExpectData<Required>::privateExpectData
            .expectedIterator;

      return this->template RequireData<Required>::privateRequireData.Get(
            this, it, type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Required>
    template <typename Data>
    constexpr bool RequireData<Required>::AlwaysRequires()
    {
      return detail::PrivateRequireData<Required>::AlwaysRequires(
            type<Data>());
    }

    namespace detail
    {
      /////////////////////////////////////////////////
      /// \brief If DS1 is true, then DataSpec1 always requires Data, and so we
      /// will choose this version, because it is guaranteed to contain Data.
      template <typename Data, typename DataSpec1, typename DataSpec2, bool DS1>
      struct ConstGetSelectorImpl
      {
        static const Data& Select(const DataSpec1* spec1)
        {
          return spec1->template Get<Data>();
        }
      };

      /////////////////////////////////////////////////
      /// \brief If DS1 is false, then DataSpec1 does not always require Data,
      /// and we will choose this version. If DataSpec2 also does not always
      /// require Data, then we produce an informative compilation error;
      /// otherwise, we will return the result of calling Get<Data>() on spec2.
      ///
      /// Note that this is template specialization which gets invoked when the
      /// final argument of ConstGetSelectorImpl is false (i.e.
      /// DataSpec1::AwaysRequires<Data>() return false). If the final argument
      /// is true, then we use the default template definition.
      ///
      /// If it turns out that DataSpec2::AlwaysRequires<Data>() is also false,
      /// then we produce an informative compilation error here, instructing
      /// users to call the Query<Data>() function instead.
      template <typename Data, typename DataSpec1, typename DataSpec2>
      struct ConstGetSelectorImpl<Data, DataSpec1, DataSpec2, false>
      {
        static const Data& Select(const DataSpec2* spec2)
        {
          static_assert(DataSpec2::template AlwaysRequires<Data>(),
                        IGNITION_PHYSICS_CONST_GET_ERROR);

          return spec2->template Get<Data>();
        }
      };

      /////////////////////////////////////////////////
      /// \brief ConstGetSelector allows SpecifyData to perform a
      /// const-qualified Get<Data>() operation if either DataSpec1 or DataSpec2
      /// lists Data as a required data type. It uses template specialization to
      /// choose between them, and only compiles Get<Data>() const using the one
      /// for which it is possible. If it is not possible for either, then it
      /// produces a very informative compilation error.
      template <typename Data, typename DataSpec1, typename DataSpec2>
      struct ConstGetSelector :
          public ConstGetSelectorImpl<
              Data, DataSpec1, DataSpec2,
              DataSpec1::template AlwaysRequires<Data>()> { };
    }

    /////////////////////////////////////////////////
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

    #define DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(\
      ReturnType, Function, Suffix, Condition, Args)\
      ReturnType Function Suffix\
      {\
        if (DataSpec2::template Condition<T>())\
          return DataSpec2::template Function <T> Args ;\
        \
        return DataSpec1::template Function <T> Args ;\
      }

    /////////////////////////////////////////////////
    template <typename DataSpec1, typename DataSpec2>
    class SpecifyData<DataSpec1, DataSpec2> :
        public virtual DataSpec1,
        public virtual DataSpec2
    {
      public: virtual ~SpecifyData() = default;

      // These allow outside users to identify the specifications at compile
      // time, which can be useful for template metaprogramming.
      public: using SubSpecification1 = DataSpec1;
      public: using SubSpecification2 = DataSpec2;

      // These allow us to use SFINAE to tunnel down into deeper specifications
      public: using ExpectedData = void;
      public: using RequiredData = void;

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          T&, Get, (), Expects, ())

      template <typename T, typename... Args>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          T&, Create, (Args&&... args), Expects,
          (std::forward<Args>(args)...))

      template <typename T, typename... Args>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          T&, GetOrCreate, (Args&&... args), Expects,
          (std::forward<Args>(args)...))

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          bool, Remove, (), Expects, ())

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          T*, Query,
          (const CompositeData::QueryMode mode =
                CompositeData::QUERY_NORMAL),
          Expects, (mode))

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          const T*, Query,
          (const CompositeData::QueryMode mode =
                CompositeData::QUERY_NORMAL) const,
          Expects, (mode))

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          bool, Has,
          (const CompositeData::QueryMode mode =
                CompositeData::QUERY_NORMAL) const,
          Expects, (mode))

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          CompositeData::DataStatus, StatusOf,
          (const CompositeData::QueryMode mode =
                CompositeData::QUERY_NORMAL) const,
          Expects, (mode))

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          bool, Unquery, () const, Expects, ())

      template <typename T, typename... Args>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          T&, MakeRequired, (Args&&... args), Expects,
          (std::forward<Args>(args)...))

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          bool, Requires, () const, Expects, ())

      template <typename T>
      static constexpr bool Expects()
      {
        return (   DataSpec1::template Expects<T>()
                || DataSpec2::template Expects<T>());
      }

      template <typename T>
      const T& Get() const
      {
        return detail::ConstGetSelector<T, DataSpec1, DataSpec2>::Select(this);
      }

      template <typename T>
      static constexpr bool AlwaysRequires()
      {
        return (   DataSpec1::template AlwaysRequires<T>()
                || DataSpec2::template AlwaysRequires<T>());
      }
    };

    /////////////////////////////////////////////////
    template <typename DataType1, typename... OtherDataTypes>
    class ExpectData<DataType1, OtherDataTypes...>
        : public virtual SpecifyData<ExpectData<DataType1>,
                                     ExpectData<OtherDataTypes...>>
    {
      public: virtual ~ExpectData() = default;
    };

    /////////////////////////////////////////////////
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

    template <typename Specification, template<typename> class FindSpec>
    constexpr std::size_t CountUpperLimitOfSpecifiedData()
    {
      return detail::SpecificationDataCounterImpl<
          typename FindSpec<Specification>::Data,
          typename Specification::SubSpecification1,
          typename Specification::SubSpecification2,
          FindSpec>::Count();
    }

    template <typename Specification>
    constexpr std::size_t CountUpperLimitOfExpectedData()
    {
      return CountUpperLimitOfSpecifiedData<Specification, FindExpected>();
    }

    template <typename Specification>
    constexpr std::size_t CountUpperLimitOfRequiredData()
    {
      return CountUpperLimitOfSpecifiedData<Specification, FindRequired>();
    }
  }
}

#endif
