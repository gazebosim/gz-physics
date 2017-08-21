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

#define IGNITION_PHYSICS_CONST_GET_ERROR \
  "Cannot use the const-qualified Get<Data>() function if " \
  "the type Data is not required by your CompositeData " \
  "specification at compile time. You should use the " \
  "const-qualified Query<Data>() function instead!\n"

// This preprocessor token should only be used by the unittest that is
// responsible for checking that the specialized routines are being used to
// access expected data.
#ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
bool usedExpectedDataAccess;
#endif

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
    Data* ExpectData<Expected>::Query()
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Query(this, type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    const Data* ExpectData<Expected>::Query() const
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Query(this, type<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Expected>
    template <typename Data>
    bool ExpectData<Expected>::Has() const
    {
      return this->template ExpectData<Expected>::privateExpectData
          .Has(this, type<Data>());
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
      template <typename Expected>
      template <typename Data>
      Data& PrivateExpectData<Expected>::Get(
          ExpectData<Expected>* data, type<Data>)
      {
        return data->CompositeData::Get<Data>();
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      Expected& PrivateExpectData<Expected>::Get(
          ExpectData<Expected>* data, type<Expected>)
      {
        #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
        usedExpectedDataAccess = true;
        #endif

        if (!this->expectedIterator->second.data)
        {
          ++data->CompositeData::numEntries;
          this->expectedIterator->second.data =
              std::unique_ptr<Cloneable>(new MakeCloneable<Expected>());
        }

        SetToQueried(this->expectedIterator, data->CompositeData::numQueries);

        return static_cast<MakeCloneable<Expected>&>(
              *this->expectedIterator->second.data);
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename Data, typename... Args>
      Data& PrivateExpectData<Expected>::Create(
          ExpectData<Expected>* data, type<Data>, Args&&... args)
      {
        return data->CompositeData::Create<Data>(std::forward<Args>(args)...);
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename... Args>
      Expected& PrivateExpectData<Expected>::Create(
          ExpectData<Expected>* data, type<Expected>, Args&&... args)
      {
        #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
        usedExpectedDataAccess = true;
        #endif

        if (!this->expectedIterator->second.data)
          ++data->CompositeData::numEntries;

        this->expectedIterator->second.data =
            std::unique_ptr<Cloneable>(new MakeCloneable<Expected>(
                                         std::forward<Args>(args)...));

        SetToQueried(this->expectedIterator, data->CompositeData::numQueries);

        return static_cast<MakeCloneable<Expected>&>(
              *this->expectedIterator->second.data);
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename Data, typename... Args>
      Data& PrivateExpectData<Expected>::GetOrCreate(
          ExpectData<Expected>* data, type<Data>, Args&&... args)
      {
        return data->CompositeData::GetOrCreate<Data>(
              std::forward<Args>(args)...);
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename... Args>
      Expected& PrivateExpectData<Expected>::GetOrCreate(
          ExpectData<Expected>* data, type<Expected>, Args&&...args)
      {
        #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
        usedExpectedDataAccess = true;
        #endif

        if (!this->expectedIterator->second.data)
        {
          ++data->CompositeData::numEntries;
          this->expectedIterator->second.data = std::unique_ptr<Cloneable>(
                new MakeCloneable<Expected>(std::forward<Args>(args)...));
        }

        SetToQueried(this->expectedIterator, data->CompositeData::numQueries);

        return static_cast<MakeCloneable<Expected>&>(
              *this->expectedIterator->second.data);
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename Data>
      bool PrivateExpectData<Expected>::Remove(
          ExpectData<Expected>* data, type<Data>)
      {
        return data->CompositeData::Remove<Data>();
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      bool PrivateExpectData<Expected>::Remove(
          ExpectData<Expected>* data, type<Expected>)
      {
        #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
        usedExpectedDataAccess = true;
        #endif

        if (!this->expectedIterator->second.data)
          return false;

        if (this->expectedIterator->second.required)
          return false;

        if (this->expectedIterator->second.queried)
        {
          --data->CompositeData::numQueries;
          this->expectedIterator->second.queried = false;
        }

        --data->CompositeData::numEntries;
        this->expectedIterator->second.data.reset();
        return true;
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename Data>
      Data* PrivateExpectData<Expected>::Query(
          ExpectData<Expected>* data, type<Data>)
      {
        return data->CompositeData::Query<Data>();
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      Expected* PrivateExpectData<Expected>::Query(
          ExpectData<Expected>* data, type<Expected>)
      {
        #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
        usedExpectedDataAccess = true;
        #endif

        SetToQueried(this->expectedIterator, data->CompositeData::numQueries);

        return static_cast<MakeCloneable<Expected>*>(
              this->expectedIterator->second.data.get());
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename Data>
      const Data* PrivateExpectData<Expected>::Query(
          const ExpectData<Expected>* data, type<Data>) const
      {
        return data->CompositeData::Query<Data>();
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      const Expected* PrivateExpectData<Expected>::Query(
          const ExpectData<Expected>* data, type<Expected>) const
      {
        #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
        usedExpectedDataAccess = true;
        #endif

        SetToQueried(this->expectedIterator, data->CompositeData::numQueries);

        return static_cast<const MakeCloneable<Expected>*>(
              this->expectedIterator->second.data.get());
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename Data>
      bool PrivateExpectData<Expected>::Has(
          const ExpectData<Expected>* data, type<Data>) const
      {
        return (nullptr != this->Query(data, type<Data>()));
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename Data, typename... Args>
      Data& PrivateExpectData<Expected>::MakeRequired(
          ExpectData<Expected>* data, type<Data>, Args&&... args)
      {
        return data->CompositeData::MakeRequired<Data>(
              std::forward<Args>(args)...);
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename... Args>
      Expected& PrivateExpectData<Expected>::MakeRequired(
          ExpectData<Expected>* data, type<Expected>, Args&&... args)
      {
        #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
        usedExpectedDataAccess = true;
        #endif

        this->expectedIterator->second.required = true;

        if (!this->expectedIterator->second.data)
        {
          ++data->CompositeData::numEntries;
          this->expectedIterator->second.data = std::unique_ptr<Cloneable>(
                new MakeCloneable<Expected>(std::forward<Args>(args)...));
        }

        SetToQueried(this->expectedIterator, data->CompositeData::numQueries);

        return static_cast<MakeCloneable<Expected>&>(
              *this->expectedIterator->second.data);
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename Data>
      bool PrivateExpectData<Expected>::Requires(
          const ExpectData<Expected>* data, type<Data>) const
      {
        return data->CompositeData::Requires<Data>();
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      bool PrivateExpectData<Expected>::Requires(
          const ExpectData<Expected>* /*data*/, type<Expected>) const
      {
        #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
        usedExpectedDataAccess = true;
        #endif

        return this->expectedIterator->second.required;
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      template <typename Data>
      constexpr bool PrivateExpectData<Expected>::Expects(type<Data>)
      {
        return false;
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      constexpr bool PrivateExpectData<Expected>::Expects(type<Expected>)
      {
        return true;
      }

      /////////////////////////////////////////////////
      template <typename Expected>
      PrivateExpectData<Expected>::PrivateExpectData(
          const CompositeData::MapOfData::iterator _it)
        : expectedIterator(_it)
      {
        // Do nothing
      }

      /////////////////////////////////////////////////
      template <typename Required>
      template <typename Data>
      const Data& PrivateRequireData<Required>::Get(
          const RequireData<Required>* data,
          const CompositeData::MapOfData::iterator& it,
          type<Data>) const
      {
        static_assert(std::is_same<Data, Required>::type,
                      IGNITION_PHYSICS_CONST_GET_ERROR);

        SetToQueried(it, data->CompositeData::numQueries);

        return static_cast<const MakeCloneable<Required>&>(*it->second.data);
      }

      /////////////////////////////////////////////////
      template <typename Required>
      const Required& PrivateRequireData<Required>::Get(
          const RequireData<Required>* data,
          const CompositeData::MapOfData::iterator& it,
          type<Required>) const
      {
        #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
        usedExpectedDataAccess = true;
        #endif

        SetToQueried(it, data->CompositeData::numQueries);

        return static_cast<const MakeCloneable<Required>&>(*it->second.data);
      }

      /////////////////////////////////////////////////
      template <typename Required>
      template <typename Data>
      constexpr bool PrivateRequireData<Required>::AlwaysRequires(type<Data>)
      {
        return false;
      }

      /////////////////////////////////////////////////
      template <typename Required>
      constexpr bool PrivateRequireData<Required>::AlwaysRequires(
          type<Required>)
      {
        return true;
      }

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
      public: using Specification1 = DataSpec1;
      public: using Specification2 = DataSpec2;

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
          T*, Query, (), Expects, ())

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          const T*, Query, () const, Expects, ())

      template <typename T>
      DETAIL_IGN_PHYSICS_SPECIFYDATA_DISPATCH(
          bool, Has, () const, Expects, ())

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
  }
}

#endif
