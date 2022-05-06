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

#include <memory>
#include <utility>

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
    // Forward declarations
    /// \private
    template <typename... DataTypes> class ExpectData;
    /// \private
    template <typename... DataTypes> class RequireData;

    namespace detail
    {
      // TODO(MXG): Consider putting this in a common header
      template <class T> struct type { };

      /// \brief This class implements the specialized functions for ExpectData.
      /// The implementations of these functions can be found in
      /// ignition/physics/detail/SpecifyData.hh
      template <typename Expected>
      class PrivateExpectData
      {
        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        Data &Get(ExpectData<Expected> *_data, type<Data>)
        {
          return _data->CompositeData::template Get<Data>();
        }

        /// \brief Use a high-speed accessor for this Expected data type
        public: Expected &Get(ExpectData<Expected> *_data, type<Expected>)
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          if (!this->expectedIterator->second.data)
          {
            ++_data->CompositeData::numEntries;
            this->expectedIterator->second.data =
                std::unique_ptr<Cloneable>(new MakeCloneable<Expected>());
          }

          SetToQueried(this->expectedIterator,
                       _data->CompositeData::numQueries);

          return static_cast<MakeCloneable<Expected>&>(
                *this->expectedIterator->second.data);
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data, typename... Args>
        CompositeData::InsertResult<Data> InsertOrAssign(
            ExpectData<Expected> *_data, type<Data>, Args&&... _args)
        {
          return _data->CompositeData::template InsertOrAssign<Data>(
                std::forward<Args>(_args)...);
        }

        /// \brief Use a high-speed accessor for this Expected data type
        public: template <typename... Args>
        CompositeData::InsertResult<Expected> InsertOrAssign(
            ExpectData<Expected> *_data, type<Expected>, Args&&... args)
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          if (!this->expectedIterator->second.data)
            ++_data->CompositeData::numEntries;

          const bool inserted = !this->expectedIterator->second.data;

          this->expectedIterator->second.data =
              std::unique_ptr<Cloneable>(new MakeCloneable<Expected>(
                                           std::forward<Args>(args)...));

          SetToQueried(this->expectedIterator,
                       _data->CompositeData::numQueries);

          return CompositeData::InsertResult<Expected>{
                static_cast<MakeCloneable<Expected>&>(
                  *this->expectedIterator->second.data),
                inserted};
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data, typename... Args>
        CompositeData::InsertResult<Data> Insert(
            ExpectData<Expected> *_data, type<Data>, Args&&... args)
        {
          return _data->CompositeData::template Insert<Data>(
                std::forward<Args>(args)...);
        }

        /// \brief Use a high-speed accessor for this Expected data type
        public: template <typename... Args>
        CompositeData::InsertResult<Expected> Insert(
            ExpectData<Expected> *_data, type<Expected>, Args&&... args)
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          bool inserted = false;

          if (!this->expectedIterator->second.data)
          {
            ++_data->CompositeData::numEntries;
            this->expectedIterator->second.data = std::unique_ptr<Cloneable>(
                  new MakeCloneable<Expected>(std::forward<Args>(args)...));
            inserted = true;
          }

          SetToQueried(this->expectedIterator,
                       _data->CompositeData::numQueries);

          return CompositeData::InsertResult<Expected>{
                static_cast<MakeCloneable<Expected>&>(
                  *this->expectedIterator->second.data),
                inserted};
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        bool Remove(ExpectData<Expected> *_data, type<Data>)
        {
          return _data->CompositeData::template Remove<Data>();
        }

        /// \brief Use a high-speed accessor for this Expected data type
        public: bool Remove(ExpectData<Expected> *_data, type<Expected>)
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          if (!this->expectedIterator->second.data)
            return true;

          if (this->expectedIterator->second.required)
            return false;

          if (this->expectedIterator->second.queried)
          {
            --_data->CompositeData::numQueries;
            this->expectedIterator->second.queried = false;
          }

          --_data->CompositeData::numEntries;
          this->expectedIterator->second.data.reset();
          return true;
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        Data *Query(ExpectData<Expected> *_data, type<Data>,
                    const CompositeData::QueryMode _mode)
        {
          return _data->CompositeData::template Query<Data>(_mode);
        }

        /// \brief Use a high-speed accessor for this Expected data type
        public: Expected *Query(ExpectData<Expected> *_data, type<Expected>,
                                const CompositeData::QueryMode _mode)
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          if (!this->expectedIterator->second.data)
            return nullptr;

          if (CompositeData::QueryMode::NORMAL == _mode)
          {
            SetToQueried(this->expectedIterator,
                               _data->CompositeData::numQueries);
          }

          return static_cast<MakeCloneable<Expected>*>(
                this->expectedIterator->second.data.get());
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        const Data *Query(const ExpectData<Expected> *_data, type<Data>,
                          const CompositeData::QueryMode mode) const
        {
          return _data->CompositeData::template Query<Data>(mode);
        }

        /// \brief Use a high-speed accessor for this Expected data type
        public: const Expected *Query(
            const ExpectData<Expected> *_data, type<Expected>,
            const CompositeData::QueryMode _mode) const
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          if (!this->expectedIterator->second.data)
            return nullptr;

          if (CompositeData::QueryMode::NORMAL == _mode)
          {
            SetToQueried(this->expectedIterator,
                               _data->CompositeData::numQueries);
          }

          return static_cast<const MakeCloneable<Expected>*>(
                this->expectedIterator->second.data.get());
        }

        /// \brief Use this->Query to perform the the Has function
        public: template <typename Data>
        bool Has(const ExpectData<Expected> *_data, type<Data>) const
        {
          return (nullptr != this->Query(_data, type<Data>(),
                                         CompositeData::QueryMode::SILENT));
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        CompositeData::DataStatus StatusOf(
            const ExpectData<Expected> *_data, type<Data>) const
        {
          return _data->CompositeData::template StatusOf<Data>();
        }

        /// \brief Use a high-speed accessor for this Expected data type
        public: CompositeData::DataStatus StatusOf(
            const ExpectData<Expected>*, type<Expected>) const
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

          return status;
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        bool Unquery(const ExpectData<Expected> *_data, type<Data>) const
        {
          return _data->CompositeData::template Unquery<Data>();
        }

        /// \brief Use a high-speed accessor for this Expected data type
        public: bool Unquery(const ExpectData<Expected> *_data,
                             type<Expected>) const
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          if (!this->expectedIterator->second.data)
            return false;

          if (!this->expectedIterator->second.queried)
            return false;

          --_data->CompositeData::numQueries;
          this->expectedIterator->second.queried = false;

          return true;
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data, typename... Args>
        Data &MakeRequired(ExpectData<Expected> *_data,
                           type<Data>, Args &&..._args)
        {
          return _data->CompositeData::template MakeRequired<Data>(
                std::forward<Args>(_args)...);
        }

        /// \brief Use a high-speed accessor for this Expected data type
        public: template <typename... Args>
        Expected &MakeRequired(ExpectData<Expected> *_data,
                               type<Expected>, Args &&..._args)
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          this->expectedIterator->second.required = true;

          if (!this->expectedIterator->second.data)
          {
            ++_data->CompositeData::numEntries;
            this->expectedIterator->second.data = std::unique_ptr<Cloneable>(
                  new MakeCloneable<Expected>(std::forward<Args>(_args)...));
          }

          SetToQueried(this->expectedIterator,
              _data->CompositeData::numQueries);

          return static_cast<MakeCloneable<Expected>&>(
                *this->expectedIterator->second.data);
        }

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        bool Requires(const ExpectData<Expected> *_data, type<Data>) const
        {
          return _data->CompositeData::template Requires<Data>();
        }

        /// \brief Use a high-speed accessor for this Expected data type
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
        private: explicit PrivateExpectData(
            const CompositeData::MapOfData::iterator _it)
          : expectedIterator(_it)
        {
          // Do nothing
        }

        /// \brief Copy constructor.
        /// Explcitly defaulted for clarity.
        private: PrivateExpectData(const PrivateExpectData&) = default;

        /// \brief Copy assignment operator.
        /// We need to specify a copy constructor because the compiler will not
        /// generate one for us. This is because the generated constructor would
        /// try to copy expectedIterator and fail because it's a const. Since
        /// expectedIterator is already initialized when the copy assignment
        /// operator is used we do nothing here.
        private: PrivateExpectData<Expected> &operator=(
            const PrivateExpectData<Expected> &)
        {
          return *this;
        }

        public: const CompositeData::MapOfData::iterator expectedIterator;
      };

      template <typename Required>
      class PrivateRequireData
      {
        /// \brief Throw a compilation error, because we cannot guarantee that
        /// this Data type will exist in the CompositeData object
        public: template <typename Data>
        const Data &Get(
            const RequireData<Required> *_data,
            const CompositeData::MapOfData::iterator &_it,
            type<Data>) const
        {
          static_assert(std::is_same<Data, Required>::type,
                        IGNITION_PHYSICS_CONST_GET_ERROR);

          SetToQueried(_it, _data->CompositeData::numQueries);

          return static_cast<const MakeCloneable<Required>&>(*_it->second.data);
        }

        /// \brief Use a high-speed accessor for this Required data type
        public: const Required &Get(
            const RequireData<Required> *_data,
            const CompositeData::MapOfData::iterator &_it,
            type<Required>) const
        {
          #ifdef IGNITION_UNITTEST_EXPECTDATA_ACCESS
          usedExpectedDataAccess = true;
          #endif

          SetToQueried(_it, _data->CompositeData::numQueries);

          return static_cast<const MakeCloneable<Required>&>(*_it->second.data);
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
