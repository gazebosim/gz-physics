/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <cassert>

#include "ignition/physics/CompositeData.hh"

namespace ignition
{
  namespace physics
  {

    /////////////////////////////////////////////////
    /// \brief Mark an entry as unqueried and decrement the query counter if
    /// the entry was originally marked as queried
    static void RemoveQuery(
        const CompositeData::MapOfData::iterator &_it, std::size_t &_numQueries)
    {
      if (_it->second.queried)
      {
        --_numQueries;
        _it->second.queried = false;
      }
    }

    /////////////////////////////////////////////////
    /// \brief Remove an entry from a map and adjust the counters
    static void RemoveEntryUnlessRequired(
        const CompositeData::MapOfData::iterator &_receiver,
        std::size_t &_numEntries, std::size_t &_numQueries)
    {
      if (!_receiver->second.required)
      {
        // If the data isn't required, delete it
        _receiver->second.data.reset();
        --_numEntries;
        RemoveQuery(_receiver, _numQueries);
      }
    }

    /////////////////////////////////////////////////
    /// \brief Use this to copy data efficiently from one existing data instance
    /// to another existing data instance
    template <typename SenderType>
    static void StandardDataCopy(
        const CompositeData::MapOfData::iterator &_receiver,
        SenderType _sender,
        const bool _mergeRequirements,
        std::size_t &/*_numEntries*/,
        std::size_t &/*_numQueries*/)
    {
      _receiver->second.data->Copy(*_sender->second.data);
      if (_mergeRequirements && _sender->second.required)
        _receiver->second.required = true;
    }

    /////////////////////////////////////////////////
    /// \brief Use this to clone data into a map entry which has an entry for
    /// the data type but does not currently have an instance
    template <typename SenderType>
    static void StandardDataClone(
        const CompositeData::MapOfData::iterator &_receiver,
        SenderType _sender,
        const bool _mergeRequirements,
        std::size_t &_numEntries,
        std::size_t &/*_numQueries*/)
    {
      assert(!_receiver->second.data &&
             "Calling StandardCloneData on a data entry that already exists. "
             "This should not be possible! Please report this bug!");

      _receiver->second = CompositeData::DataEntry(
            _sender->second.data->Clone(),
            _mergeRequirements && _sender->second.required);

      ++_numEntries;
    }

    /////////////////////////////////////////////////
    /// \brief Use this to create a new data entry and instance for a map which
    /// did not previously contain an entry for the data type
    template <typename SenderType>
    static void StandardDataCreate(
        CompositeData::MapOfData &_toMap,
        SenderType _sender,
        const bool _mergeRequirements,
        std::size_t &_numEntries)
    {
      const bool inserted = _toMap.insert(
            std::make_pair(
              _sender->first,
              CompositeData::DataEntry(
                _sender->second.data->Clone(),
                _mergeRequirements && _sender->second.required))).second;

      (void)(inserted);
      assert(inserted &&
             "Calling StandardDataCreate on a data entry that already exists. "
             "This should not be possible! Please report this bug!");

      ++_numEntries;
    }

    /////////////////////////////////////////////////
    /// \brief Use move semantics for a more efficient version of
    /// StandardDataCopy and StandardDataClone
    template <typename SenderType>
    static void MoveData(
        const CompositeData::MapOfData::iterator &_receiver,
        SenderType _sender,
        const bool _mergeRequirements,
        std::size_t &_numEntries,
        std::size_t &/*_numQueries*/)
    {
      if (!_receiver->second.data)
        ++_numEntries;

      _receiver->second.data.reset(_sender->second.data.release());
      _receiver->second.required =
          _mergeRequirements && _sender->second.required;
    }

    /////////////////////////////////////////////////
    /// \brief Use move semantics for a more efficient version of
    /// StandardDataCreate
    template <typename SenderType>
    static void MoveDataCreate(
        CompositeData::MapOfData &_toMap,
        SenderType _sender,
        const bool _mergeRequirements,
        std::size_t &_numEntries)
    {
      const bool inserted = _toMap.insert(
            std::make_pair(
              _sender->first,
              CompositeData::DataEntry(
                std::unique_ptr<Cloneable>(_sender->second.data.release()),
                _mergeRequirements && _sender->second.required))).second;

      (void)(inserted);
      assert(inserted &&
             "Calling MoveDataCreate on a data entry that already exists. This "
             "should not be possible! Please report this bug!");

      ++_numEntries;
    }

    /////////////////////////////////////////////////
    template <typename SenderType>
    using DataTransferFnc = void(*)(
            const CompositeData::MapOfData::iterator&,
            SenderType, const bool, std::size_t&, std::size_t&);

    /////////////////////////////////////////////////
    template <typename SenderType>
    using DataCreateFnc = void(*)(
            CompositeData::MapOfData&,
            SenderType, const bool, std::size_t&);

    /////////////////////////////////////////////////
    template <typename SenderType, typename FromMapType>
    static void CopyMapData(
        std::size_t &_numEntries,
        std::size_t &_numQueries,
        CompositeData::MapOfData &_toMap,
        FromMapType _fromMap,
        const bool _mergeData,
        const bool _mergeRequirements,
        DataTransferFnc<SenderType> CopyDataFnc,
        DataTransferFnc<SenderType> CloneDataFnc,
        DataCreateFnc<SenderType> CreateDataFnc)
    {
      CompositeData::MapOfData::iterator receiver = _toMap.begin();
      auto sender = _fromMap.begin();

      while (_fromMap.end() != sender)
      {
        if (_toMap.end() == receiver)
        {
          // If we've reached the end of this CompositeData's map, then we
          // should just add each remaining entry from sender. We don't have to
          // worry about conflicts, because this CompositeData cannot already
          // have any of these types.
          if (sender->second.data)
          {
            CreateDataFnc(_toMap, sender, _mergeRequirements, _numEntries);
          }

          ++sender;
        }
        else if (receiver->first == sender->first)
        {
          if (sender->second.data)
          {
            // If the sender has an entry whose key matches this one...

            if (receiver->second.data)
            {
              // If we already have an instance, we should copy instead of
              // allocating a clone
              CopyDataFnc(receiver, sender, _mergeRequirements,
                          _numEntries, _numQueries);
            }
            else if (!receiver->second.data)
            {
              assert(!receiver->second.queried &&
                     "An entry which was supposed to be empty is marked as "
                     "queried. This should be impossible!");

              // If we don't already have an instance, we should clone it.
              CloneDataFnc(receiver, sender, _mergeRequirements,
                           _numEntries, _numQueries);
            }
          }
          else
          {
            // If the sender does not have an object at this entry...

            if (!_mergeData)
            {
              RemoveEntryUnlessRequired(receiver, _numEntries, _numQueries);
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

          if (!_mergeData)
          {
            RemoveEntryUnlessRequired(receiver, _numEntries, _numQueries);
          }

          ++receiver;
        }
        else
        {
          if (sender->second.data)
          {
            // If the receiver has a higher key value, then the receiving map
            // does not contain an entry that matches this entry of the sending
            // map, and therefore the entry must be created.
            CreateDataFnc(_toMap, sender, _mergeRequirements, _numEntries);
          }

          ++sender;
        }
      }

      if (!_mergeData)
      {
        // Remove any remaining data structures in the receiver which do not
        // correspond to any entries that were in the sender.
        while (_toMap.end() != receiver)
        {
          RemoveEntryUnlessRequired(receiver, _numEntries, _numQueries);
          ++receiver;
        }
      }
    }

    /////////////////////////////////////////////////
    CompositeData::CompositeData()
      : numEntries(0),
        numQueries(0)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    CompositeData::DataStatus::DataStatus()
      : exists(false),
        queried(false),
        required(false)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    std::size_t CompositeData::EntryCount() const
    {
      assert(numEntries <= dataMap.size() &&
             "The recorded number of entries is greater than the size of the "
             "dataMap, but that should be impossible!");
      return numEntries;
    }

    /////////////////////////////////////////////////
    std::size_t CompositeData::UnqueriedEntryCount() const
    {
      assert(numEntries >= numQueries &&
             "The recorded number of queries is greater than the recorded "
             "number of entries, but that should be impossible!");
      return numEntries - numQueries;
    }

    /////////////////////////////////////////////////
    void CompositeData::ResetQueries() const
    {
      numQueries = 0;

      for (auto &entry : dataMap)
        entry.second.queried = false;
    }

    /////////////////////////////////////////////////
    std::set<std::string> CompositeData::AllEntries() const
    {
      if (EntryCount() == 0)
        return std::set<std::string>();

      std::set<std::string> entries;

      for (const auto &entry : dataMap)
      {
        if (entry.second.data)
          entries.insert(entries.end(), entry.first);
      }

      return entries;
    }

    /////////////////////////////////////////////////
    std::set<std::string> CompositeData::UnqueriedEntries() const
    {
      if (UnqueriedEntryCount() == 0)
        return std::set<std::string>();

      std::set<std::string> unqueried;

      for (const auto &entry : dataMap)
      {
        if (entry.second.data && !entry.second.queried)
          unqueried.insert(unqueried.end(), entry.first);
      }

      return unqueried;
    }

    /////////////////////////////////////////////////
    CompositeData& CompositeData::Copy(
        const CompositeData &_other,
        const bool _mergeRequirements)
    {
      using SenderType = CompositeData::MapOfData::const_iterator;
      CopyMapData<SenderType, const CompositeData::MapOfData&>(
            numEntries, numQueries,
            this->dataMap, _other.dataMap,
            false, _mergeRequirements,
            &StandardDataCopy<SenderType>,
            &StandardDataClone<SenderType>,
            &StandardDataCreate<SenderType>);

      return *this;
    }

    /////////////////////////////////////////////////
    CompositeData& CompositeData::Copy(
        CompositeData &&_other,
        const bool _mergeRequirements)
    {
      using SenderType = CompositeData::MapOfData::iterator;
      CopyMapData<SenderType, CompositeData::MapOfData&&>(
            numEntries, numQueries,
            this->dataMap, std::move(_other.dataMap),
            false, _mergeRequirements,
            &MoveData<SenderType>,
            &MoveData<SenderType>,
            &MoveDataCreate<SenderType>);

      return *this;
    }

    /////////////////////////////////////////////////
    CompositeData &CompositeData::Merge(
        const CompositeData &_other,
        const bool _mergeRequirements)
    {
      using SenderType = CompositeData::MapOfData::const_iterator;
      CopyMapData<SenderType, const CompositeData::MapOfData&>(
            numEntries, numQueries,
            this->dataMap, _other.dataMap,
            true, _mergeRequirements,
            &StandardDataCopy<SenderType>,
            &StandardDataClone<SenderType>,
            &StandardDataCreate<SenderType>);

      return *this;
    }

    /////////////////////////////////////////////////
    CompositeData &CompositeData::Merge(
        CompositeData &&_other,
        const bool _mergeRequirements)
    {
      using SenderType = CompositeData::MapOfData::iterator;
      CopyMapData<SenderType, CompositeData::MapOfData&&>(
            numEntries, numQueries,
            this->dataMap, std::move(_other.dataMap),
            true, _mergeRequirements,
            &MoveData<SenderType>,
            &MoveData<SenderType>,
            &MoveDataCreate<SenderType>);

      return *this;
    }

    /////////////////////////////////////////////////
    CompositeData::CompositeData(const CompositeData &_other)
      : CompositeData()
    {
      this->Copy(_other);
    }

    /////////////////////////////////////////////////
    CompositeData::CompositeData(CompositeData &&_other)
      : CompositeData()
    {
      this->Copy(std::move(_other));
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
      : required(false),
        queried(false)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    CompositeData::DataEntry::DataEntry(
        std::unique_ptr<Cloneable> &&_data,
        bool _required)
      : data(std::move(_data)),
        required(_required),
        queried(false)
    {
      // Do nothing
    }
  }
}
