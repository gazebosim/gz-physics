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

#include "ignition/physics/Cloneable.hh"

namespace ignition
{
  namespace physics
  {
    /// \brief The CompositeData class allows arbitrary data structures to be
    /// composed together, copied, and moved with type erasure.
    class CompositeData
    {
      /// \brief Default constructor
      public: CompositeData() = default;

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
      public: template <typename Data, typename... Args>
      Data& Create(Args&&... args);

      /// \brief If a Data type object is available in this CompositeData,
      /// this will return a reference to it. Otherwise, it will create a new
      /// Data type object with the arguments provided, and return a reference
      /// to the newly created object.
      public: template <typename Data, typename... Args>
      Data& GetOrCreate(Args&&... args);

      /// \brief This will remove a Data type object from this CompositeData and
      /// delete it if one is present. Otherwise, it will do nothing. This is
      /// equivalent to calling Extract<Data>(false).
      public: template <typename Data>
      void Remove();

      /// \brief This will remove a Data type object from this CompositeData if
      /// one exists and return it. Otherwise, it will return a nullptr.
      ///
      /// If the object is marked as required by the CompositeData, then a copy
      /// of it will be returned if copyIfRequired is set to true. Otherwise,
      /// if the object is required and copyIfRequired is set to false, it will
      /// return a nullptr.
      public: template <typename Data>
      std::unique_ptr<Data> Extract(bool copyIfRequired = true);

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
      /// required, it will continue to be required for the entire lifespan of
      /// this CompositeData.
      public: template <typename Data, typename... Args>
      Data& MakeRequired(Args&&... args);

      /// \brief Returns true if the specified Data type is required by this
      /// CompositeData object. Otherwise, returns false.
      public: template <typename Data>
      bool IsRequired() const;


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
      /// requirements specified by other. Any objects that are already marked
      /// as required in this CompositeData will remain required.
      public: CompositeData& Copy(const CompositeData &_other,
                                  const CopyOption _option = IDENTICAL,
                                  const bool _mergeRequirements = false);

      /// \brief A version of Copy() that takes advantage of move semantics.
      public: CompositeData& Copy(CompositeData &&_other,
                                  const CopyOption _option = IDENTICAL,
                                  const bool _mergeRequirements = false);

      /// \brief Copy operator. Same as Copy(other).
      public: CompositeData& operator=(const CompositeData &_other);

      /// \brief Move operator. Same as Copy(other).
      public: CompositeData& operator=(CompositeData &&_other);

      /// \brief Struct which contains information about
      public: struct DataEntry
      {
        public: DataEntry();

        public: DataEntry(
          std::unique_ptr<Cloneable> &&_data,
          bool _required);

        public: std::unique_ptr<Cloneable> data;
        public: bool required;
      };

      /// Map from the name of a data object to its entry
      public: using MapOfData = std::map<std::string, DataEntry>;
      /// \brief Map from the name of a data object type to its type
      protected: MapOfData dataMap;
    };
  }
}

#include "ignition/physics/detail/CompositeData.hh"

#endif
