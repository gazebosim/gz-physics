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

#ifndef IGNITION_PHYSICS_DETAIL_PRIVATESPECIFYDATA_HH_
#define IGNITION_PHYSICS_DETAIL_PRIVATESPECIFYDATA_HH_

#include "ignition/physics/CompositeData.hh"

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
    // TODO(MXG): Consider putting this in a common header
    template <class T> struct type { };

    // Forward declarations
    template <typename... DataTypes> class ExpectData;
    template <typename... DataTypes> class RequireData;

    namespace detail
    {
      /// \brief This class implements the specialized functions for ExpectData.
      /// The implementations of these functions can be found in
      /// ignition/physics/detail/SpecifyData.hh
      template <typename Expected>
      class PrivateExpectData
      {
        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        Data& Get(ExpectData<Expected>* data, type<Data>)
        {
          return data->CompositeData::template Get<Data>();
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: Expected& Get(ExpectData<Expected>* data, type<Expected>)
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

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data, typename... Args>
        Data& Create(ExpectData<Expected>* data, type<Data>, Args&&... args)
        {
          return data->CompositeData::template Create<Data>(
                std::forward<Args>(args)...);
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: template <typename... Args>
        Expected& Create(ExpectData<Expected>* data,
                         type<Expected>, Args&&... args)
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

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data, typename... Args>
        Data& Insert(ExpectData<Expected>* data,
                          type<Data>, Args&&... args)
        {
          return data->CompositeData::template Insert<Data>(
                std::forward<Args>(args)...);
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: template <typename... Args>
        Expected& Insert(ExpectData<Expected>* data,
                              type<Expected>, Args&&... args)
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

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        bool Remove(ExpectData<Expected>* data, type<Data>)
        {
          return data->CompositeData::template Remove<Data>();
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: bool Remove(ExpectData<Expected>* data, type<Expected>)
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

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        Data* Query(ExpectData<Expected>* data, type<Data>,
                    const CompositeData::QueryMode mode)
        {
          return data->CompositeData::template Query<Data>(mode);
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: Expected* Query(ExpectData<Expected>* data, type<Expected>,
                                const CompositeData::QueryMode mode)
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          if (!this->expectedIterator->second.data)
            return nullptr;

          if (CompositeData::QUERY_NORMAL == mode)
            SetToQueried(this->expectedIterator,
                         data->CompositeData::numQueries);

          return static_cast<MakeCloneable<Expected>*>(
                this->expectedIterator->second.data.get());
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        const Data* Query(const ExpectData<Expected>* data, type<Data>,
                          const CompositeData::QueryMode mode) const
        {
          return data->CompositeData::template Query<Data>(mode);
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: const Expected* Query(
            const ExpectData<Expected>* data, type<Expected>,
            const CompositeData::QueryMode mode) const
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          if (!this->expectedIterator->second.data)
            return nullptr;

          if(CompositeData::QUERY_NORMAL == mode)
            SetToQueried(this->expectedIterator,
                         data->CompositeData::numQueries);

          return static_cast<const MakeCloneable<Expected>*>(
                this->expectedIterator->second.data.get());
        }

        /// \brief Use this->Query to perform the the Has function
        public: template <typename Data>
        bool Has(const ExpectData<Expected>* data, type<Data>,
                 const CompositeData::QueryMode mode) const
        {
          return (nullptr != this->Query(data, type<Data>(), mode));
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        CompositeData::DataStatus StatusOf(
            const ExpectData<Expected>* data, type<Data>,
            const CompositeData::QueryMode mode) const
        {
          return data->CompositeData::template StatusOf<Data>(mode);
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: CompositeData::DataStatus StatusOf(
            const ExpectData<Expected>* data, type<Expected>,
            const CompositeData::QueryMode mode) const
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          // status is initialized to everything being false
          CompositeData::DataStatus status;

          if (!this->expectedIterator->second.data)
            return status;

          status.exists = true;
          status.required = this->expectedIterator->second.required;
          status.queried = this->expectedIterator->second.queried;

          if (CompositeData::QUERY_NORMAL == mode)
            SetToQueried(this->expectedIterator,
                         data->CompositeData::numQueries);

          return status;
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        bool Unquery(const ExpectData<Expected>* data, type<Data>) const
        {
          return data->CompositeData::template Unquery<Data>();
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: bool Unquery(const ExpectData<Expected>* data,
                             type<Expected>) const
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          if (!this->expectedIterator->second.data)
            return false;

          if (!this->expectedIterator->second.queried)
            return false;

          --data->CompositeData::numQueries;
          this->expectedIterator->second.queried = false;

          return true;
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data, typename... Args>
        Data& MakeRequired(ExpectData<Expected>* data,
                           type<Data>, Args&&... args)
        {
          return data->CompositeData::template MakeRequired<Data>(
                std::forward<Args>(args)...);
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: template <typename... Args>
        Expected& MakeRequired(ExpectData<Expected>* data,
                               type<Expected>, Args&&... args)
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

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        bool Requires(const ExpectData<Expected>* data, type<Data>) const
        {
          return data->CompositeData::template Requires<Data>();
        }

        /// \brief Use a low-cost accessor for this Expected data type
        public: bool Requires(const ExpectData<Expected>*, type<Expected>) const
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          return this->expectedIterator->second.required;
        }

        /// \brief Always returns false
        public: template <typename Data>
        static constexpr bool Expects(type<Data>)
        {
          return false;
        }

        /// \brief Always returns true
        public: static constexpr bool Expects(type<Expected>)
        {
          return true;
        }

        template <typename...> friend class ::ignition::physics::ExpectData;

        /// \brief Construct this with the iterator that it is meant to hold
        private: PrivateExpectData(
            const CompositeData::MapOfData::iterator _it)
          : expectedIterator(_it)
        {
          // Do nothing
        }

        public: const CompositeData::MapOfData::iterator expectedIterator;
      };

      template <typename Required>
      class PrivateRequireData
      {
        /// \brief Throw a compilation error, because we cannot guarantee that
        /// this Data type will exist in the CompositeData object
        public: template <typename Data>
        const Data& Get(
            const RequireData<Required>* data,
            const CompositeData::MapOfData::iterator& it,
            type<Data>) const
        {
          static_assert(std::is_same<Data, Required>::type,
                        IGNITION_PHYSICS_CONST_GET_ERROR);

          SetToQueried(it, data->CompositeData::numQueries);

          return static_cast<const MakeCloneable<Required>&>(*it->second.data);
        }

        /// \brief Use a low-cost accessor for this Required data type
        public: const Required& Get(
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

        /// \brief Always returns false
        public: template <typename Data>
        static constexpr bool AlwaysRequires(type<Data>)
        {
          return false;
        }

        /// \brief Always returns true
        public: static constexpr bool AlwaysRequires(type<Required>)
        {
          return true;
        }
      };
    }
  }
}

#endif
