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
    /////////////////////////////////////////////////
    template <typename Data>
    Data &CompositeData::Get()
    {
      MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(Data::IgnPhysicsTypeLabel, DataEntry())).first;

      if(!it->second.data)
      {
        it->second.data = std::unique_ptr<Cloneable>(new MakeCloneable<Data>());
      }

      return static_cast<MakeCloneable<Data>&>(*it->second.data);
    }

    /////////////////////////////////////////////////
    template <typename Data, typename... Args>
    Data& CompositeData::Create(Args&&... args)
    {
      MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(Data::IgnPhysicsTypeLabel, DataEntry())).first;

      it->second.data = std::unique_ptr<Cloneable>(
            new MakeCloneable<Data>(std::forward<Args>(args)...));

      return static_cast<MakeCloneable<Data>&>(*it->second.data);
    }

    /////////////////////////////////////////////////
    template <typename Data, typename... Args>
    Data& CompositeData::GetOrCreate(Args&&... args)
    {
      MapOfData::iterator it = this->dataMap.insert(
            std::make_pair(Data::IgnPhysicsTypeLabel, DataEntry())).first;

      if(!it->second.data)
      {
        it->second.data = std::unique_ptr<Cloneable>(
              new MakeCloneable<Data>(std::forward<Args>(args)...));
      }

      return static_cast<MakeCloneable<Data>&>(*it->second.data);
    }

    /////////////////////////////////////////////////
    template <typename Data>
    void CompositeData::Remove()
    {
      this->Extract<Data>(false);
    }

    /////////////////////////////////////////////////
    template <typename Data>
    std::unique_ptr<Data> CompositeData::Extract(bool copyIfRequired)
    {
      MapOfData::iterator it = this->dataMap.find(Data::IgnPhysicsTypeLabel);
      if(this->dataMap.end() == it)
        return nullptr;

      if(it->second.required)
      {
        if(copyIfRequired)
          return std::unique_ptr<Data>(
                static_cast<Data*>(it->second.data->Clone().release()));

        return nullptr;
      }

      return std::unique_ptr<Data>(
            static_cast<MakeCloneable<Data>*>(it->second.data.release()));
    }

    /////////////////////////////////////////////////
    template <typename Data>
    Data* CompositeData::Query()
    {
      MapOfData::const_iterator it =
          this->dataMap.find(Data::IgnPhysicsTypeLabel);

      if(this->dataMap.end() == it)
        return nullptr;

      return static_cast<MakeCloneable<Data>*>(it->second.data.get());
    }

    /////////////////////////////////////////////////
    template <typename Data>
    const Data* CompositeData::Query() const
    {
      MapOfData::const_iterator it =
          this->dataMap.find(Data::IgnPhysicsTypeLabel);

      if(this->dataMap.end() == it)
        return nullptr;

      return it->second.data.get();
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
            std::make_pair(Data::IgnPhysicsTypeLabel, DataEntry())).first;

      it->second.required = true;
      if(!it->second.data)
      {
        it->second.data = std::unique_ptr<Cloneable>(
              new MakeCloneable<Data>(std::forward<Args>(args)...));
      }

      return static_cast<MakeCloneable<Data>&>(*it->second.data);
    }

    /////////////////////////////////////////////////
    template <typename Data>
    bool CompositeData::IsRequired() const
    {
      MapOfData::iterator it = this->dataMap.find(Data::IgnPhysicsTypeLabel);

      if(this->dataMap.end() == it)
        return false;

      return it->second.required;
    }
  }
}

#endif
