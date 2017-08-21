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

    /////////////////////////////////////////////////
    template <typename Data>
    Data &CompositeData::Get()
    {
      MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(Data::IgnPhysicsTypeLabel(), DataEntry())).first;

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
      MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(Data::IgnPhysicsTypeLabel(), DataEntry())).first;

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
      MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(Data::IgnPhysicsTypeLabel(), DataEntry())).first;

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
      MapOfData::iterator it = this->dataMap.find(Data::IgnPhysicsTypeLabel());

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
    Data* CompositeData::Query()
    {
      MapOfData::const_iterator it =
          this->dataMap.find(Data::IgnPhysicsTypeLabel());

      if (this->dataMap.end() == it)
        return nullptr;

      detail::SetToQueried(it, this->numQueries);

      return static_cast<MakeCloneable<Data>*>(it->second.data.get());
    }

    /////////////////////////////////////////////////
    template <typename Data>
    const Data* CompositeData::Query() const
    {
      MapOfData::const_iterator it =
          this->dataMap.find(Data::IgnPhysicsTypeLabel());

      if (this->dataMap.end() == it)
        return nullptr;

      detail::SetToQueried(it, this->numQueries);

      return static_cast<const MakeCloneable<Data>*>(it->second.data.get());
    }

    /////////////////////////////////////////////////
    template <typename Data>
    bool CompositeData::Has() const
    {
      return (nullptr != this->Query<Data>());
    }

    /////////////////////////////////////////////////
    template <typename Data, typename... Args>
    Data& CompositeData::MakeRequired(Args&&... args)
    {
      MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(Data::IgnPhysicsTypeLabel(), DataEntry())).first;

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
      MapOfData::const_iterator it =
          this->dataMap.find(Data::IgnPhysicsTypeLabel());

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
