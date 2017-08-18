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


#include "ignition/physics/CompositeData.hh"

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename SenderType>
    static void StandardDataCopy(
        const CompositeData::MapOfData::iterator &_receiver,
        SenderType _sender,
        const bool _mergeRequirements)
    {
      _receiver->second.data->Copy(*_sender->second.data);
      if(_mergeRequirements && _sender->second.required)
        _receiver->second.required = true;
    }

    /////////////////////////////////////////////////
    template <typename SenderType>
    static void StandardDataClone(
        const CompositeData::MapOfData::iterator &_receiver,
        SenderType _sender,
        const bool _mergeRequirements)
    {
      _receiver->second = CompositeData::DataEntry(
            _sender->second.data->Clone(),
            _mergeRequirements && _sender->second.required);
    }

    /////////////////////////////////////////////////
    template <typename SenderType>
    static void StandardDataCreate(
        CompositeData::MapOfData &_toMap,
        SenderType _sender,
        const bool _mergeRequirements)
    {
      _toMap[_sender->first] = CompositeData::DataEntry(
            _sender->second.data->Clone(),
            _mergeRequirements && _sender->second.required);
    }

    /////////////////////////////////////////////////
    template <typename SenderType>
    static void MoveData(
        const CompositeData::MapOfData::iterator &_receiver,
        SenderType _sender,
        const bool _mergeRequirements)
    {
      _receiver->second = CompositeData::DataEntry(
            std::unique_ptr<Cloneable>(_sender->second.data.release()),
            _mergeRequirements && _sender->second.required);
    }

    /////////////////////////////////////////////////
    template <typename SenderType>
    static void MoveDataCreate(
        CompositeData::MapOfData &_toMap,
        SenderType _sender,
        const bool _mergeRequirements)
    {
      _toMap[_sender->first] = CompositeData::DataEntry(
            std::unique_ptr<Cloneable>(_sender->second.data.release()),
            _mergeRequirements && _sender->second.required);
    }

    /////////////////////////////////////////////////
    template <typename SenderType>
    using DataTransferFnc = void(*)(
            const CompositeData::MapOfData::iterator&,
            SenderType,
            const bool);

    /////////////////////////////////////////////////
    template <typename SenderType>
    using DataCreateFnc = void(*)(
            CompositeData::MapOfData&,
            SenderType,
            const bool);

    /////////////////////////////////////////////////
    template <typename SenderType, typename FromMapType>
    static void CopyMapData(
        CompositeData::MapOfData &_toMap,
        FromMapType _fromMap,
        const CompositeData::CopyOption _option,
        const bool _mergeRequirements,
        DataTransferFnc<SenderType> CopyDataFnc,
        DataTransferFnc<SenderType> CloneDataFnc,
        DataCreateFnc<SenderType> CreateDataFnc)
    {
      CompositeData::MapOfData::iterator receiver = _toMap.begin();
      auto sender = _fromMap.begin();

      const bool intersect =
             CompositeData::HARD_INTERSECT == _option
          || CompositeData::SOFT_INTERSECT == _option;

      const bool merge =
             CompositeData::HARD_MERGE
          || CompositeData::SOFT_MERGE
          || CompositeData::IDENTICAL;

      const bool hardCopy =
             CompositeData::HARD_MERGE == _option
          || CompositeData::HARD_INTERSECT == _option
          || CompositeData::IDENTICAL == _option;

      const bool removeMissing =
             CompositeData::HARD_INTERSECT == _option
          || CompositeData::SOFT_INTERSECT == _option
          || CompositeData::IDENTICAL == _option;

      while (_fromMap.end() != sender)
      {
        if (_toMap.end() == receiver)
        {
          if (intersect)
            break;

          // If we've reached the end of this CompositeData's map, then we
          // should just add each remaining entry from sender. We don't have to
          // worry about conflicts, because this CompositeData cannot already
          // have any of these types.
          CreateDataFnc(_toMap, sender, _mergeRequirements);

          ++sender;
        }
        else if (receiver->first == sender->first)
        {
          if (sender->second.data)
          {
            // If the sender has an entry whose key matches this one...

            if (receiver->second.data)
            {
              if (hardCopy)
              {
                // If we already have an instance, we should copy instead of
                // allocating a clone
                CopyDataFnc(receiver, sender, _mergeRequirements);
              }
              else
              {
                // If this is a SOFT_MERGE or a SOFT_INTERSECT, then no data
                // should be copied, but we should merge the requirements if
                // requested.

                if (_mergeRequirements && sender->second.required)
                  receiver->second.required = true;
              }
            }
            else if (!receiver->second.data)
            {
              if(merge)
              {
                // If we don't already have an instance, we should clone it.
                CloneDataFnc(receiver, sender, _mergeRequirements);
              }

              // If this is a HARD_INTERSECT or a SOFT_INTERSECT, then no data
              // should be copied and no requirements altered.
            }
          }
          else
          {
            // If the sender does not have an object at this entry...

            if (removeMissing)
            {
              if (!receiver->second.required)
              {
                // If this data isn't required, delete it
                receiver->second.data = nullptr;
              }
            }

            // Note that this data cannot be required by the sender if they do
            // not have it, so we do not need to worry about changing the
            // requirement flag.
          }

          ++sender;
          ++receiver;
        }
        else if (receiver->first < sender->first)
        {
          // If the receiver has some data that the sender does not...

          if (removeMissing)
          {
            if (!receiver->second.required)
            {
              // If the data isn't required, delete it
              receiver->second.data = nullptr;
            }
          }

          ++receiver;
        }
        else
        {
          if (sender->second.data)
          {
            // If the receiver has a higher key value, then the receiving map
            // does not contain an entry that matches this entry of the sending
            // map, and therefore the entry must be created if we are doing a
            // merge.

            if (merge)
            {
              CreateDataFnc(_toMap, sender, _mergeRequirements);
            }
          }

          ++sender;
        }
      }

      if (removeMissing)
      {
        // Remove any remaining data structures in the receiver which do not
        // correspond to any entries that were in the sender.
        while (_toMap.end() != receiver)
        {
          if(!receiver->second.required)
            receiver->second.data = nullptr;

          ++receiver;
        }
      }
    }

    /////////////////////////////////////////////////
    CompositeData& CompositeData::Copy(
        const CompositeData &_other,
        const CopyOption _option,
        const bool _mergeRequirements)
    {
      using SenderType = CompositeData::MapOfData::const_iterator;
      CopyMapData<SenderType>(
            this->dataMap, _other.dataMap, _option, _mergeRequirements,
            &StandardDataCopy<SenderType>,
            &StandardDataClone<SenderType>,
            &StandardDataCreate<SenderType>);

      return *this;
    }

    CompositeData& CompositeData::Copy(
        CompositeData &&_other,
        const CopyOption _option,
        const bool _mergeRequirements)
    {
      using SenderType = CompositeData::MapOfData::iterator;
      CopyMapData<SenderType>(
            this->dataMap, _other.dataMap, _option, _mergeRequirements,
            &MoveData<SenderType>,
            &MoveData<SenderType>,
            &MoveDataCreate<SenderType>);

      return *this;
    }

    /////////////////////////////////////////////////
    CompositeData& CompositeData::operator=(const CompositeData &_other)
    {
      return this->Copy(_other);
    }

    /////////////////////////////////////////////////
    CompositeData& CompositeData::operator=(CompositeData &&_other)
    {
      return this->Copy(std::move(_other));
    }

    /////////////////////////////////////////////////
    CompositeData::DataEntry::DataEntry()
      : required(false)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    CompositeData::DataEntry::DataEntry(
        std::unique_ptr<Cloneable> &&_data,
        bool _required)
      : data(std::move(_data)),
        required(_required)
    {
      // Do nothing
    }
  }
}
