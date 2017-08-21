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
        Data& Get(ExpectData<Expected>* data, type<Data>);

        /// \brief Use a low-cost accessor for this Expected data type
        public: Expected& Get(ExpectData<Expected>* data, type<Expected>);

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data, typename... Args>
        Data& Create(ExpectData<Expected>* data, type<Data>, Args&&... args);

        /// \brief Use a low-cost accessor for this Expected data type
        public: template <typename... Args>
        Expected& Create(ExpectData<Expected>* data,
                         type<Expected>, Args&&... args);

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data, typename... Args>
        Data& GetOrCreate(ExpectData<Expected>* data,
                          type<Data>, Args&&... args);

        /// \brief Use a low-cost accessor for this Expected data type
        public: template <typename... Args>
        Expected& GetOrCreate(ExpectData<Expected>* data,
                              type<Expected>, Args&&... args);

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        bool Remove(ExpectData<Expected>* data, type<Data>);

        /// \brief Use a low-cost accessor for this Expected data type
        public: bool Remove(ExpectData<Expected>* data, type<Expected>);

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        Data* Query(ExpectData<Expected>* data, type<Data>);

        /// \brief Use a low-cost accessor for this Expected data type
        public: Expected* Query(ExpectData<Expected>* data, type<Expected>);

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        const Data* Query(const ExpectData<Expected>* data, type<Data>) const;

        /// \brief Use a low-cost accessor for this Expected data type
        public: const Expected* Query(const ExpectData<Expected>* data,
                                type<Expected>) const;

        /// \brief Use this->Query to perform the the Has function
        public: template <typename Data>
        bool Has(const ExpectData<Expected>* data, type<Data>) const;

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data, typename... Args>
        Data& MakeRequired(ExpectData<Expected>* data,
                           type<Data>, Args&&... args);

        /// \brief Use a low-cost accessor for this Expected data type
        public: template <typename... Args>
        Expected& MakeRequired(ExpectData<Expected>* data,
                               type<Expected>, Args&&... args);

        /// \brief Delegate the function to the standard CompositeData method
        public: template <typename Data>
        bool Requires(const ExpectData<Expected>* data, type<Data>) const;

        /// \brief Use a low-cost accessor for this Expected data type
        public: bool Requires(const ExpectData<Expected>* data,
                                type<Expected>) const;

        /// \brief Always returns false
        public: template <typename Data>
        static constexpr bool Expects(type<Data>);

        /// \brief Always returns true
        public: static constexpr bool Expects(type<Expected>);

        template <typename...> friend class ExpectData;

        /// \brief Construct this with the iterator that it is meant to hold
        private: PrivateExpectData(
            const CompositeData::MapOfData::iterator _it);

        public: const CompositeData::MapOfData::iterator expectedIterator;
      };

      template <typename Required>
      class PrivateRequireData
      {
        /// \brief Throw a compilation error, because we cannot guarantee that
        /// this Data type will exist in the CompositeData object
        public: template <typename Data>
        const Data& Get(
            const RequireData<Required>*,
            const CompositeData::MapOfData::iterator&,
            type<Data>) const;

        /// \brief Use a low-cost accessor for this Required data type
        public: const Required& Get(
            const RequireData<Required>* data,
            const CompositeData::MapOfData::iterator& it,
            type<Required>) const;

        /// \brief Always returns false
        public: template <typename Data>
        static constexpr bool AlwaysRequires(type<Data>);

        /// \brief Always returns true
        public: static constexpr bool AlwaysRequires(type<Required>);
      };
    }
  }
}

#endif
