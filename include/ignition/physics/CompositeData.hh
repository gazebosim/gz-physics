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

#ifndef IGNITION_PHYSICS_COMPOSITEDATA_HH_
#define IGNITION_PHYSICS_COMPOSITEDATA_HH_

#include <string>
#include <map>
#include <set>

#include "ignition/physics/Cloneable.hh"
#include "ignition/common/System.hh"

namespace ignition
{
  namespace physics
  {
    // Forward declarations
    namespace detail
    {
      template <typename> class PrivateExpectData;
      template <typename> class PrivateRequireData;
    }

    /// \brief The CompositeData class allows arbitrary data structures to be
    /// composed together, copied, and moved with type erasure.
    class IGNITION_COMMON_VISIBLE CompositeData
    {
      /// \brief Default constructor. Creates an empty CompositeData object.
      public: CompositeData();

      /// \brief Virtual destructor
      public: virtual ~CompositeData() = default;

      /// \brief Get a reference to a Data object. If an object of the Data type
      /// does not already exist in this CompositeData, then create one using
      /// its default constructor. This function will fail to compile if a
      /// default constructor is not available for the Data type.
      ///
      /// When you have a `const CompositeData` object, you must use the
      /// function CompositeData::query<Data>() in order to retrieve objects.
      public: template <typename Data>
      Data& Get();

      /// \brief Create a Data type object with the provided arguments. If a
      /// Data object is already present in this CompositeData, it will be
      /// deleted and replaced with the newly constructed object. This returns
      /// a reference to the newly constructed objected.
      ///
      /// Note that this will invalidate ALL previously obtained references to
      /// this Data type from this CompositeData object. References to other
      /// types of data from this CompositeData object will not be affected.
      public: template <typename Data, typename... Args>
      Data& Create(Args&&... args);

      /// \brief If a Data type object is available in this CompositeData,
      /// this will return a reference to it. Otherwise, it will create a new
      /// Data type object with the arguments provided, and return a reference
      /// to the newly created object.
      public: template <typename Data, typename... Args>
      Data& GetOrCreate(Args&&... args);

      /// \brief This will remove a Data type object from this CompositeData and
      /// delete it if one is present. Otherwise, it will do nothing. Data that
      /// are marked as required will not be removed.
      ///
      /// If the data was successfully removed or did not exist to begin with,
      /// this returns true. If the data was marked as required and therefore
      /// not removed, this returns false.
      public: template <typename Data>
      bool Remove();

      /// \brief Query this CompositeData for an object of type T. If it
      /// contains an object of type T, it gets returned as a T*. Otherwise, a
      /// nullptr is returned.
      public: template <typename Data>
      Data* Query();

      /// \brief Const-qualified version of Query. This can be used to retrieve
      /// data from a `const CompositeData`.
      public: template <typename Data>
      const Data* Query() const;

      /// \brief Returns true if this CompositeData has an object of type T.
      public: template <typename Data>
      bool Has() const;

      /// \brief Marks the specified type of Data as required, creates one with
      /// the given arguments if it did not exist, and returns a reference to
      /// it.
      ///
      /// Warning: This cannot be undone. Once a Data type is marked as
      /// required, it will continue to be required for this object throughout
      /// the rest of its lifespan.
      public: template <typename Data, typename... Args>
      Data& MakeRequired(Args&&... args);

      /// \brief Returns true if the specified Data type is required by this
      /// CompositeData object. Otherwise, returns false.
      public: template <typename Data>
      bool Requires() const;

      /// \brief When called from a generic CompositeData type, this always
      /// returns false. More highly specified CompositeData types that use
      /// ExpectData or RequireData may return true if the type of Data is
      /// expected.
      public: template <typename Data>
      static constexpr bool Expects();

      /// \brief When called from a generic CompositeData type, this always
      /// returns false. Static (Always) requirements are determined at
      /// compile-time and cannot be changed at runtime. Using the RequireData
      /// class can make this return true for more highly specified
      /// CompositeData types.
      ///
      /// NOTE: This should never be used to check whether Data is required on
      /// a specific instance, because the requirements that are placed on an
      /// instance can be changed at runtime. This should only be used to check
      /// whether a certain data type is always required for a certain
      /// specification of CompositeData.
      public: template <typename Data>
      static constexpr bool AlwaysRequires();

      /// \brief Returns the number of data entries currently contained in this
      /// CompositeData. Runs with O(1) complexity.
      std::size_t NumEntries() const;

      /// \brief Returns the number of entries in this CompositeData which have
      /// not been queried (Get, Create, GetOrCreate, Query, Has, and
      /// MakeRequired) since the data was created (not using the aforementioned
      /// functions) or since the last call to ResetQueries(), whichever is more
      /// recent. Runs with O(1) complexity.
      ///
      /// Unqueried data entries might be created by copy/move construction,
      /// copy/move assignment operation, or the Copy(~) function. Using the
      /// copy/move operator or the Copy(~) function will reset the query flag
      /// on any data that gets copied or moved over.
      std::size_t NumUnqueriedEntries() const;

      /// \brief Reset the query flags on all data entries. This will make it
      /// appear as though no entries have ever been queried.
      ///
      /// It is good practice to call this function before returning a
      /// CompositeData from a function and handing it off to another segment of
      /// a pipeline, because sometimes the compiler inappropriately elides the
      /// copy/move constructor/operators and passes along the state of the
      /// queries, even though it should not.
      void ResetQueries() const;

      /// \brief Returns an ordered set of the data entries in this
      /// CompositeData which have not been queried. This can be useful for
      /// reporting runtime warnings about any unsupported data types that have
      /// been given to you.
      std::set<std::string> UnqueriedDataEntries() const;


      /// \brief Options that determine the behavior of the Copy() function
      enum CopyOption
      {
        /// \brief Make this CompositeData identical to the other. If this
        /// CompositeData contains data types that the other does not, they will
        /// be deleted, unless they are marked as required.
        IDENTICAL = 0,

        /// \brief Merge in the data from the other CompositeData, but do not
        /// delete any data structures that are unique to this one. For any data
        /// structures that are held by both CompositeData objects, the data
        /// from the other CompositeData will overwrite the data from this one.
        HARD_MERGE,

        /// \brief Any data structure types in the other CompositeData object
        /// that  are not already present in this one will be copied over. Any
        /// data structures that were already present in this CompositeData
        /// object will remain unchanged.
        SOFT_MERGE,

        /// \brief Any data structure types in this CompositeData object that
        /// are not held by the other CompositeData object will be deleted. Any
        /// data structures that are held by both will get copied over from the
        /// other. Required data objects will not be deleted.
        HARD_INTERSECT,

        /// \brief Any data structure types in this CompositeData object that
        /// are not held by the other CompositeData object will be deleted. All
        /// other data will remain unchanged. Required data objects will not
        /// be deleted.
        SOFT_INTERSECT
      };

      /// \brief Alter this CompositeData object based on the option that is
      /// provided. The other CompositeData will remain unchanged.
      ///
      /// If mergeRequirements is set to true, this object will also take on the
      /// requirements specified by _other. Any objects that are already marked
      /// as required in this CompositeData will remain required.
      public: CompositeData& Copy(const CompositeData &_other,
                                  const CopyOption _option = IDENTICAL,
                                  const bool _mergeRequirements = false);

      /// \brief A version of Copy() that takes advantage of move semantics.
      public: CompositeData& Copy(CompositeData &&_other,
                                  const CopyOption _option = IDENTICAL,
                                  const bool _mergeRequirements = false);

      /// \brief Copy constructor. Same as Copy(_other).
      public: CompositeData(const CompositeData &_other);

      /// \brief Move constructor. Same as Copy(_other).
      public: CompositeData(CompositeData &&_other);

      /// \brief Copy operator. Same as Copy(_other).
      public: CompositeData& operator=(const CompositeData &_other);

      /// \brief Move operator. Same as Copy(_other).
      public: CompositeData& operator=(CompositeData &&_other);

      /// \brief Struct which contains information about
      public: struct DataEntry
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

      /// Map from the name of a data object to its entry
      public: using MapOfData = std::map<std::string, DataEntry>;
      /// \brief Map from the name of a data object type to its type
      protected: MapOfData dataMap;

      /// \brief Total number of data entries currently in this CompositeData.
      /// Note that this may differ from the size of dataMap, because some
      /// entries in dataMap will be referring to nullptrs.
      private: std::size_t numEntries;

      /// \brief Total number of unique queries which have been performed since
      /// either construction or the last call to ResetQueries().
      private: mutable std::size_t numQueries;

      template <typename> friend class detail::PrivateExpectData;
      template <typename> friend class detail::PrivateRequireData;
    };
  }
}

#include "ignition/physics/detail/CompositeData.hh"

#endif
