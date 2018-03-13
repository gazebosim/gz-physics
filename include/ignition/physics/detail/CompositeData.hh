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

#ifndef IGNITION_PHYSICS_DETAIL_COMPOSITEDATA_HH_
#define IGNITION_PHYSICS_DETAIL_COMPOSITEDATA_HH_

#include "ignition/physics/CompositeData.hh"

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      /////////////////////////////////////////////////
      /// \brief Helper function to set the query flag of previously unqueried
      /// data entries. The template argument is to support both iterator&
      /// and const_iterator& argument types. This helper functions lets us
      /// avoid hard-to-spot typos on this frequently performed task.
      template <typename IteratorType>
      void SetToQueried(const IteratorType &it, std::size_t &numQueries)
      {
        if (!it->second.queried)
        {
          ++numQueries;
          it->second.queried = true;
        }
      }
    }

    /// \brief Struct which contains information about a data type within the
    /// CompositeData.
    struct CompositeData::DataEntry
    {
      /// \brief Default constructor
      public: DataEntry();

      /// \brief Constructor that accepts an rvalue reference and a
      /// requirement setting
      public: DataEntry(
        std::unique_ptr<Cloneable> &&_data,
        bool _required);

      /// \brief Data that is being held at this entry. nullptr means the
      /// CompositeData does not have data for this entry
      public: std::unique_ptr<Cloneable> data;

      /// \brief Flag for whether the type of data at this entry is considered
      /// to be required. This can be made true during the lifetime of the
      /// CompositeData, but it must never be changed from true to false.
      public: bool required;

      /// \brief Flag for whether this data entry has been queried since
      /// either (1) it was created using Copy(~), =, or the CompositeData
      /// constructor, or (2) the last time ResetQueries() was called,
      /// whichever was more recent. Functions that can mark an entry as
      /// queried include Get(), Create(), GetOrCreate(), Query(), and Has().
      public: mutable bool queried;
    };

    /////////////////////////////////////////////////
    template <typename Data>
    Data &CompositeData::Get()
    {
      const MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(typeid(Data).name(), DataEntry())).first;

      if (!it->second.data)
      {
        ++this->numEntries;
        it->second.data = std::unique_ptr<Cloneable>(new MakeCloneable<Data>());
      }

      detail::SetToQueried(it, this->numQueries);

      return static_cast<MakeCloneable<Data>&>(*it->second.data);
    }

    /////////////////////////////////////////////////
    template <typename Data, typename... Args>
    Data& CompositeData::Create(Args&&... args)
    {
      const MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(typeid(Data).name(), DataEntry())).first;

      if (!it->second.data)
        ++this->numEntries;

      it->second.data = std::unique_ptr<Cloneable>(
            new MakeCloneable<Data>(std::forward<Args>(args)...));

      detail::SetToQueried(it, this->numQueries);

      return static_cast<MakeCloneable<Data>&>(*it->second.data);
    }

    /////////////////////////////////////////////////
    template <typename Data, typename... Args>
    Data& CompositeData::GetOrCreate(Args&&... args)
    {
      const MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(typeid(Data).name(), DataEntry())).first;

      if (!it->second.data)
      {
        ++this->numEntries;
        it->second.data = std::unique_ptr<Cloneable>(
              new MakeCloneable<Data>(std::forward<Args>(args)...));
      }

      detail::SetToQueried(it, this->numQueries);

      return static_cast<MakeCloneable<Data>&>(*it->second.data);
    }

    /////////////////////////////////////////////////
    template <typename Data>
    bool CompositeData::Remove()
    {
      const MapOfData::iterator it =
          this->dataMap.find(typeid(Data).name());

      if (this->dataMap.end() == it || !it->second.data)
        return true;

      // Do not remove it if it's required
      if (it->second.required)
        return false;

      // Decrement the query count if it had been queried
      if (it->second.queried)
      {
        --this->numQueries;
        it->second.queried = false;
      }

      --this->numEntries;
      it->second.data.reset();
      return true;
    }

    /////////////////////////////////////////////////
    template <typename Data>
    Data* CompositeData::Query(const QueryMode mode)
    {
      const MapOfData::const_iterator it =
          this->dataMap.find(typeid(Data).name());

      if (this->dataMap.end() == it)
        return nullptr;

      if (!it->second.data)
        return nullptr;

      if (QUERY_NORMAL == mode)
        detail::SetToQueried(it, this->numQueries);

      return static_cast<MakeCloneable<Data>*>(it->second.data.get());
    }

    /////////////////////////////////////////////////
    template <typename Data>
    const Data* CompositeData::Query(const QueryMode mode) const
    {
      const MapOfData::const_iterator it =
          this->dataMap.find(typeid(Data).name());

      if (this->dataMap.end() == it)
        return nullptr;

      if (!it->second.data)
        return nullptr;

      if (QUERY_NORMAL == mode)
        detail::SetToQueried(it, this->numQueries);

      return static_cast<const MakeCloneable<Data>*>(it->second.data.get());
    }

    /////////////////////////////////////////////////
    template <typename Data>
    bool CompositeData::Has(const QueryMode mode) const
    {
      return (nullptr != this->Query<Data>(mode));
    }

    /////////////////////////////////////////////////
    template <typename Data>
    CompositeData::DataStatus CompositeData::StatusOf(
        const QueryMode mode) const
    {
      // status is initialized to everything being false
      DataStatus status;

      const MapOfData::const_iterator it =
          this->dataMap.find(typeid(Data).name());

      if (this->dataMap.end() == it)
        return status;

      if (!it->second.data)
        return status;

      status.exists = true;
      status.required = it->second.required;
      status.queried = it->second.queried;

      if (QUERY_NORMAL == mode)
        detail::SetToQueried(it, this->numQueries);

      return status;
    }

    /////////////////////////////////////////////////
    template <typename Data>
    bool CompositeData::Unquery() const
    {
      const MapOfData::const_iterator it =
          this->dataMap.find(typeid(Data).name());

      if (this->dataMap.end() == it)
        return false;

      if (!it->second.data)
        return false;

      if (!it->second.queried)
        return false;

      --this->numQueries;
      it->second.queried = false;

      return true;
    }

    /////////////////////////////////////////////////
    template <typename Data, typename... Args>
    Data& CompositeData::MakeRequired(Args&&... args)
    {
      const MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(typeid(Data).name(), DataEntry())).first;

      it->second.required = true;
      if (!it->second.data)
      {
        ++this->numEntries;
        it->second.data = std::unique_ptr<Cloneable>(
              new MakeCloneable<Data>(std::forward<Args>(args)...));
      }

      detail::SetToQueried(it, this->numQueries);

      return static_cast<MakeCloneable<Data>&>(*it->second.data);
    }

    /////////////////////////////////////////////////
    template <typename Data>
    bool CompositeData::Requires() const
    {
      const MapOfData::const_iterator it =
          this->dataMap.find(typeid(Data).name());

      if (this->dataMap.end() == it)
        return false;

      return it->second.required;
    }

    /////////////////////////////////////////////////
    template <typename Data>
    constexpr bool CompositeData::Expects()
    {
      return false;
    }

    /////////////////////////////////////////////////
    template <typename Data>
    constexpr bool CompositeData::AlwaysRequires()
    {
      return false;
    }
  }
}

#endif
