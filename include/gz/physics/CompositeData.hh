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

#ifndef GZ_PHYSICS_COMPOSITEDATA_HH_
#define GZ_PHYSICS_COMPOSITEDATA_HH_

#include <string>
#include <map>
#include <set>

#include <gz/utilities/SuppressWarning.hh>

#include "gz/physics/Cloneable.hh"
#include "gz/physics/Export.hh"

namespace gz
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
    class IGNITION_PHYSICS_VISIBLE CompositeData
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
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <iostream>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     // Create a data structure called MyData
      ///     struct MyData
      ///     {
      ///       std::string myString;
      ///     };
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///
      ///       // An object of type MyData is created using the default
      ///       // constructor of MyData. The MyData object will be stored
      ///       // inside of the object called "composite", and we can grab a
      ///       // mutable reference to it.
      ///       MyData &data = composite.Get<MyData>();
      ///
      ///       // We can modify the MyData object inside of "composite" using
      ///       // the mutable reference that we grabbed.
      ///       data.myString = "I modified this data";
      ///
      ///       // This will print out "I modified this data", because
      ///       // Get<MyData>() will retrieve the instance of MyData that was
      ///       // created by the first call of Get<MyData>().
      ///       std::cout << composite.Get<MyData>().myString << std::endl;
      ///     }
      /// \endcode
      ///
      /// \tparam Data
      ///   The type of data entry to get
      ///
      /// \return a reference to to a Data-type object that is stored within
      /// this CompositeData.
      public: template <typename Data>
      Data &Get();

      /// \brief This struct is the return type of the various Insert...<T>()
      /// functions. It returns a reference to the data entry for the Data type,
      /// and it indicates whether the insertion operation actually occurred.
      public: template <typename Data>
      struct InsertResult
      {
        /// \brief A reference to the Data entry within the CompositeData
        /// object.
        public: Data &data;

        /// \brief True if the operation resulted in inserting a new data entry.
        ///
        /// For Insert<Data>(), false means that nothing was inserted.
        ///
        /// For InsertOrAssign<Data>(), false means that the Data entry which
        /// used to be in the CompositeData has been assigned the new value.
        public: const bool inserted;

        // We explicitly delete the copy assignment operator, because we don't
        // want someone to inadvertently assign a value to the data that's being
        // referenced (users must do so explicitly by calling on the `data`
        // member variable). Note that this operator is already implicitly
        // deleted by having the const bool member variable, but we are also
        // deleting it explicitly for clarity.
        public: InsertResult &operator=(const InsertResult&) = delete;
      };

      /// \brief This will attempt to insert a new Data entry into the
      /// CompositeData object, forwarding _args to the constructor of the
      /// entry. If an entry already exists for this Data type, then nothing
      /// will be inserted.
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <iostream>
      ///     #include <cassert>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     // Create a data structure called MyData
      ///     struct MyData
      ///     {
      ///       MyData(const std::string &_arg = "")
      ///         : myString(_arg)
      ///       {
      ///         // Intentionally blank
      ///       }
      ///
      ///       std::string myString;
      ///     };
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///
      ///       // Create an object of type MyData and store it in "composite".
      ///       // A reference to the newly created object is returned.
      ///       MyData &data = composite.Insert<MyData>("some argument").data;
      ///
      ///       // This will print out "some argument", because the string in
      ///       // MyData was initialized with that argument.
      ///       std::cout << composite.Get<MyData>().myString << std::endl;
      ///
      ///       // Modify the object
      ///       data.myString = "I modified this data";
      ///
      ///       // This will print out "I modified this data" because we used a
      ///       // mutable reference to change the value of MyData::myString.
      ///       std::cout << composite.Get<MyData>().myString << std::endl;
      ///
      ///       // This will not modify the MyData instance that's being held by
      ///       // "composite", because the instance already exists. It will
      ///       // just return the instance as a reference and ignore the
      ///       // argument that has been passed in.
      ///       MyData &altData = composite.Insert<MyData>(
      ///                           "another argument").data;
      ///
      ///       // The reference to "data" is still perfectly valid, and in fact
      ///       // its underlying pointer is equal to the underlying pointer of
      ///       // "altData".
      ///       assert((&data) == (&altData));
      ///
      ///       // This will print out "I modified this data" because there have
      ///       // not been any function calls that can alter the value of
      ///       // myString in the time since that modification was made.
      ///       std::cout << composite.Insert<MyData>().data.myString
      ///                 << std::endl;
      ///     }
      /// \endcode
      ///
      /// \tparam Data
      ///   The type name for the data entry
      /// \tparam Args
      ///   This will be inferred from _args; you should not typically set this
      ///   explicitly.
      ///
      /// \param[in] _args
      ///   The arguments to use for construction. These will get wrapped in a
      ///   Data(...) constructor. If _args is left blank, the default
      ///   constructor will be used.
      ///
      /// \return An InsertResult<Data> which contains a reference to the data
      /// entry (either the one that is newly inserted or the one that already
      /// existed). InsertResult::inserted will be true if the entry was
      /// inserted by this function, or false if the entry already existed.
      ///
      /// \sa InsertOrAssign
      public: template <typename Data, typename... Args>
      InsertResult<Data> Insert(Args &&..._args);

      /// \brief Attempt to insert a Data-type entry. If a Data-type entry did
      /// not already exist, it will be constructed by copying (or moving)
      /// the given arguments. If a Data-type entry already existed, the
      /// existing entry will be assigned the value of Data(_args...).
      ///
      /// Any previously existing valid references to the Data entry will remain
      /// valid, even after this function is called.
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <iostream>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     // Create a data structure called MyData
      ///     struct MyData
      ///     {
      ///       MyData(const std::string &_arg = "")
      ///         : myString(_arg)
      ///       {
      ///         // Intentionally blank
      ///       }
      ///
      ///       std::string myString;
      ///     };
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///
      ///       // Create an object of type MyData and store it in "composite".
      ///       // A reference to the newly created object is returned.
      ///       MyData &data = composite.InsertOrAssign<MyData>(
      ///                         "some argument").data;
      ///
      ///       // This will print out "some argument", because the string in
      ///       // MyData was initialized with that argument.
      ///       std::cout << composite.Get<MyData>().myString << std::endl;
      ///
      ///       // Modify the object
      ///       data.myString = "I modified this data";
      ///
      ///       // This will print out "I modified this data" because we used a
      ///       // mutable reference to change the value of MyData::myString.
      ///       std::cout << composite.Get<MyData>().myString << std::endl;
      ///
      ///       // Assign the MyData entry a new, default-constructed value.
      ///       composite.InsertOrAssign<MyData>();
      ///
      ///       // This will print out nothing but a newline.
      ///       std::cout << composite.Get<MyData>().myString << std::endl;
      ///     }
      /// \endcode
      ///
      /// \tparam Data
      ///   The type name for the data entry
      /// \tparam Args
      ///   This will be inferred from _args; you should not typically set this
      ///   explicitly.
      ///
      /// \param[in] _args
      ///   The arguments to use for construction or assignment. These will get
      ///   wrapped in a Data(...) constructor. If _args is left blank, the
      ///   default constructor will be used.
      ///
      /// \return an InsertResult<Data> which contains a reference to the
      /// Data-type entry of this CompositeData. InsertResult<Data>::inserted
      /// will be true iff a Data-type entry did not already exist in this
      /// CompositeData. If the value was instead assigned,
      /// InsertResult<Data>::inserted will be false.
      ///
      /// \sa Insert
      public: template <typename Data, typename... Args>
      InsertResult<Data> InsertOrAssign(Args &&... _args);

      /// \brief This will remove a Data-type object from this CompositeData and
      /// delete it if one is present. Otherwise, it will do nothing. Data-types
      /// that are marked as required will not (and cannot) be removed.
      ///
      /// If the data was successfully removed or did not exist to begin with,
      /// this returns true. If the data was marked as required and therefore
      /// not removed, this returns false.
      ///
      /// \warning Calling this function will permanently invalidate all
      /// existing references to the Data-type entry of this CompositeData, i.e.
      /// the references that get returned by Get<Data>(), Insert<Data>(~),
      /// InsertOrAssign<Data>(~), or Query<Data>().
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <iostream>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     // Create a data structure called MyData
      ///     struct MyData
      ///     {
      ///       MyData(const std::string &_arg = "")
      ///         : myString(_arg)
      ///       {
      ///         // Intentionally blank
      ///       }
      ///
      ///       std::string myString;
      ///     };
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///
      ///       // Create an object of type MyData and store it in "composite".
      ///       // Append .data to get a reference to the new data.
      ///       MyData &data = composite.Insert<MyData>("some argument").data;
      ///
      ///       // Print out "some argument"
      ///       std::cout
      ///         << composite.Insert<MyData>("another argument").data.myString
      ///         << std::endl;
      ///
      ///       // Remove the MyData object. Note that "data" is now INVALID and
      ///       // must never be used again after this function call.
      ///       composite.Remove<MyData>();
      ///
      ///       // Print out "another argument"
      ///       std::cout
      ///         << composite.Insert<MyData>("another argument").data.myString
      ///         << std::endl;
      ///     }
      /// \endcode
      ///
      /// \tparam Data
      ///   The type of data entry to remove
      ///
      /// \return true iff the CompositeData no longer contains this Data type.
      public: template <typename Data>
      bool Remove();

      /// \brief Use these flags in Query(), Has(), and StatusOf() to change
      /// their effects on the meta info of the data being queried.
      ///
      /// See UnqueriedEntries() for more on the "queried" flag.
      enum class QueryMode : int
      {
        /// \brief Performing the operation will cause an unqueried Data's
        /// status to flip to queried. Data that is already marked as queried
        /// will be unaffected.
        NORMAL = 0,

        /// \brief Performing the operation has no effect on whether data is
        /// marked as queried.
        SILENT
      };

      /// \brief Query this CompositeData for a Data-type entry. If it contains
      /// a Data-type object, it gets returned as a Data*. Otherwise, a nullptr
      /// is returned.
      ///
      /// If _mode is set to QueryMode::SILENT, then calling this function will
      /// not cause the "queried" flag to change (see UnqueriedEntries() for
      /// more on the "queried" flag).
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <iostream>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     struct MyDataWithoutDefault
      ///     {
      ///       // This struct does not allow us to use the default constructor
      ///       MyDataWithoutDefault() = delete;
      ///
      ///       // We must call this constructor if we want to make an object of
      ///       // this type.
      ///       MyDataWithoutDefault(const int inputValue)
      ///         : myValue(inputValue)
      ///       {
      ///         // Intentionally blank
      ///       }
      ///
      ///       int myValue;
      ///     };
      ///
      ///     void SetValueIfAvailable(const int _value,
      ///                              CompositeData &_composite)
      ///     {
      ///       // This CANNOT compile because MyDataWithoutDefault does not
      ///       // provide a default constructor
      ///       // MyDataWithoutDefault &data =
      ///       //    composite.Get<MyDataWithoutDefault>();
      ///
      ///       // Get a pointer to a MyDataWithoutDefault instance if one is
      ///       // available, otherwise we get a nullptr.
      ///       MyDataWithoutDefault *data =
      ///         _composite.Query<MyDataWithoutDefault>();
      ///
      ///       if(data)
      ///         data->myValue = _value;
      ///     }
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///
      ///       // This will do nothing, because "composite" does not contain
      ///       // an object of type MyDataWithoutDefault.
      ///       SetValueIfAvailable(320, composite);
      ///
      ///       // We can create a MyDataWithoutDefault object by calling the
      ///       // Insert function and passing it an argument for the object's
      ///       // constructor.
      ///       composite.Insert<MyDataWithoutDefault>(123);
      ///
      ///       // This will print out "123" because the object will not be
      ///       // re-created by Insert. We can call Insert because it
      ///       // can use the argument we pass in to create an instance of
      ///       // MyDataWithoutDefault if an instance of it did not already
      ///       // exist, unlike Get which can only call the default
      ///       // constructor.
      ///       std::cout
      ///         << composite.Insert<MyDataWithoutDefault>(1).data.myValue
      ///         << std::endl;
      ///
      ///       // This will set myValue to 5 because "composite" contains an
      ///       // instance of MyDataWithoutDefault.
      ///       SetValueIfAvailable(5, composite);
      ///
      ///       // This will print out "5" because that was the value set by
      ///       // SetValueIfAvailable.
      ///       std::cout
      ///         << composite.Insert<MyDataWithoutDefault>(3).data.myValue
      ///         << std::endl;
      ///     }
      /// \endcode
      ///
      /// \tparam Data
      ///   The type of data entry to query for
      ///
      /// \param[in] _mode
      ///   Specify how this call should affect the query flag of the entry.
      ///   The default behavior is strongly recommended, unless you know what
      ///   you are doing. See QueryMode for more info.
      ///
      /// \return a pointer to the Data entry if this CompositeData has one.
      /// Otherwise, this returns a nullptr.
      public: template <typename Data>
      Data *Query(const QueryMode _mode = QueryMode::NORMAL);

      /// \brief Const-qualified version of Query. This can be used to retrieve
      /// data from a `const CompositeData`.
      ///
      /// If "mode" is set to QueryMode::SILENT, then calling this function will
      /// not cause the "queried" flag to change (see UnqueriedEntries() for
      /// more on the "queried" flag).
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <iostream>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     struct MyData
      ///     {
      ///       std::string myString;
      ///     };
      ///
      ///     void PrintStringIfAvailable(const CompositeData &_composite)
      ///     {
      ///       // The following Get<T>() CANNOT compile because "composite" is
      ///       // a const-qualified reference, but Get<T>() is NOT a
      ///       // const-qualified member function. Get<T>() cannot be
      ///       // const-qualified because it needs to create an instance of
      ///       // type T if an instance of that type was not already available
      ///       // in the CompositeData object, and that would mean modifying
      ///       // the CompositeData object, which violates const-correctness.
      ///       // This is similarly the case for Insert<T>(~) and
      ///       // InsertOrAssign<T>(~).
      ///       //
      ///       // MyData &data = _composite.Get<T>(); // error!
      ///
      ///       // Instead, we use the const-qualified Query<T>() function which
      ///       // can return a const-qualified pointer to the data instance.
      ///       const MyData *data = _composite.Query<MyData>();
      ///
      ///       if(data)
      ///         std::cout << data->myString << std::endl;
      ///       else
      ///         std::cout << "No string available" << std::endl;
      ///     }
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///
      ///       // This will print "No string available" because "composite"
      ///       // does not have a MyData instance yet.
      ///       PrintStringIfAvailable(composite);
      ///
      ///       // Use the default constructor to create MyData, and use the
      ///       // reference it returns to set the value of MyData::myString.
      ///       composite.Get<MyData>().myString = "here is a string";
      ///
      ///       // This will print "here is a string".
      ///       PrintStringIfAvailable(composite);
      ///     }
      /// \endcode
      ///
      /// \tparam Data
      ///   The type of data entry to query for
      ///
      /// \param[in] _mode
      ///   Specify how this call should affect the query flag of the entry.
      ///   The default behavior is strongly recommended, unless you know what
      ///   you are doing. See QueryMode for more info.
      ///
      /// \return a const-qualified pointer to the Data entry if this
      /// CompositeData has one. Otherwise, this returns a nullptr.
      public: template <typename Data>
      const Data *Query(const QueryMode mode = QueryMode::NORMAL) const;

      /// \brief Returns true if this CompositeData has an object of type Data,
      /// otherwise returns false. This is literally equivalent to
      /// (nullptr != Query<Data>(QueryMode::SILENT)).
      ///
      /// \tparam Data
      ///   The type of data entry to check for
      ///
      /// \return true if this CompositeData contains a Data-type entry.
      public: template <typename Data>
      bool Has() const;

      /// \brief Struct that describes the status of data.
      struct IGNITION_PHYSICS_VISIBLE DataStatus
      {
        /// \brief If the data exists in the CompositeData, this will be true,
        /// otherwise it is false.
        public: bool exists;

        /// \brief If the data was marked as queried BEFORE calling StatusOf
        /// (regardless of what QueryMode is used), this will be true, otherwise
        /// it is false. See UnqueriedEntries() for more on the "queried" flag.
        public: bool queried;

        /// \brief If the data is marked as required, this will be true,
        /// otherwise it is false.
        public: bool required;

        /// \brief Default constructor. Initializes everything to false.
        DataStatus();
      };

      /// \brief Returns a DataStatus object that describes the status of the
      /// requested data type.
      ///
      /// \tparam Data
      ///   The type of data entry to check the status of
      ///
      /// \return a DataStatus for the requested entry.
      public: template <typename Data>
      DataStatus StatusOf() const;

      /// \brief Returns true if this CompositeData has a Data-type object
      /// which was marked as queried, and that object is now marked as
      /// unqueried. If an object of that type does not exist or it was already
      /// unqueried, this returns false.
      ///
      /// This function is const-qualified because the query flags are declared
      /// as mutable.
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <cassert>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     struct MyData
      ///     {
      ///       std::string myString;
      ///     };
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///
      ///       CompositeData::DataStatus status = composite.StatusOf<MyData>();
      ///
      ///       // An instance of MyData should not exist, be required, or be
      ///       // queried.
      ///       assert(!status.exists);
      ///       assert(!status.queried);
      ///       assert(!status.required);
      ///
      ///       composite.Insert<MyData>();
      ///       status = composite.StatusOf<MyData>();
      ///
      ///       // MyData should now be queried, because explicitly creating an
      ///       // object will mark it as queried.
      ///       assert(status.exists);
      ///       assert(status.queried);
      ///
      ///       // Turn off the "queried" flag for MyData.
      ///       composite.Unquery<MyData>();
      ///
      ///       status = composite.StatusOf<MyData>();
      ///       assert(status.exists);
      ///       // It should be unqueried because we turned off its query flag
      ///       // using Unquery<MyData>().
      ///       assert(!status.queried);
      ///     }
      /// \endcode
      ///
      /// \tparam Data
      ///   The type of data entry whose query flag should be cleared
      ///
      /// \return true if the query flag is changing from queried to unqueried,
      /// otherwise false.
      public: template <typename Data>
      bool Unquery() const;

      /// \brief Marks the specified type of Data as required, creates one with
      /// the given arguments if it did not exist, and returns a reference to
      /// it.
      ///
      /// Warning: This cannot be undone. Once a Data type is marked as
      /// required, it will continue to be required for this object throughout
      /// the rest of the CompositeData object's lifespan.
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <cassert>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     struct MyData
      ///     {
      ///       MyData(const std::string &arg = "")
      ///         : myString(arg)
      ///       {
      ///         // Intentionally blank
      ///       }
      ///
      ///       std::string myString;
      ///     };
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///
      ///       // Create an instance of MyData and mark it as required
      ///       composite.MakeRequired<MyData>("this is required");
      ///
      ///       // Now it cannot be removed from "composite"
      ///       assert(!composite.Remove<MyData>());
      ///
      ///       // Once "composite" goes out of scope, the MyData instance will
      ///       // be deleted as normal, because its lifecycle is still tied to
      ///       // the CompositeData object.
      ///     }
      /// \endcode
      ///
      /// \tparam Data
      ///   The type of data entry that should be marked as required
      /// \tparam Args
      ///   This will be inferred from _args; you should not typically set this
      ///   explicitly.
      ///
      /// \param[in] _args
      ///   The arguments to use for construction, if a Data-type entry did not
      ///   already exist within this CompositeData.
      ///
      /// \return a reference to the Data-type entry
      public: template <typename Data, typename... Args>
      Data &MakeRequired(Args &&..._args);

      /// \brief Returns true if the specified Data type is required by this
      /// CompositeData object. Otherwise, returns false.
      ///
      /// For more on required data, see MakeRequired<Data>().
      ///
      /// \tparam Data
      ///   The type of data entry whose requirements are being checked
      ///
      /// \return true iff the specified Data type is required by this
      /// CompositeData
      public: template <typename Data>
      bool Requires() const;

      /// \brief When called from a generic CompositeData type, this always
      /// returns false. More highly specified CompositeData types that use
      /// ExpectData or RequireData may return true if the type of Data is
      /// expected.
      ///
      /// \tparam Data
      ///   The type of data whose expectation is being checked
      ///
      /// \return When called on a basic CompositeData object, this is always
      /// false.
      public: template <typename Data>
      static constexpr bool Expects();

      /// \brief When called from a generic CompositeData type, this always
      /// returns false. Static (Always) requirements are determined at
      /// compile-time and cannot be changed at runtime. Using the RequireData
      /// class can make this return true for more highly specified
      /// CompositeData types.
      ///
      /// \warning This should never be used to check whether Data is required
      /// on a specific instance, because the requirements that are placed on an
      /// instance can be changed at runtime. This should only be used to check
      /// whether a certain data type is always required for a certain
      /// specification of CompositeData.
      ///
      /// \tparam Data
      ///   The type of data whose requirement we are checking at compile time
      ///
      /// \return When called on a basic CompositeData object, this is always
      /// false.
      public: template <typename Data>
      static constexpr bool AlwaysRequires();

      /// \brief Check how many data entries are in this CompositeData. Runs
      /// with O(1) complexity.
      ///
      /// \return the number of data entries currently contained in this
      /// CompositeData.
      public: std::size_t EntryCount() const;

      /// \brief Check how many data entries in this CompositeData have not been
      /// queried. See UnqueriedEntries() for more information about the
      /// "queried" flag. Runs with O(1) complexity.
      ///
      /// \return the number of entries in this CompositeData which have
      /// not been queried.
      public: std::size_t UnqueriedEntryCount() const;

      /// \brief Reset the query flags on all data entries. This will make it
      /// appear as though no entries have ever been queried. See
      /// UnqueriedEntries() for more information about the "queried" flag.
      ///
      /// \attention It is good practice to call this function before returning
      /// a CompositeData from a function and handing it off to another segment
      /// of a pipeline, because sometimes the compiler inappropriately elides
      /// the copy/move constructor/operators and passes along the state of the
      /// queries, even though it should not.
      public: void ResetQueries() const;

      /// \brief Get an ordered set of all data entries in this CompositeData.
      /// Runs with O(N) complexity.
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <iostream>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     struct MyData1
      ///     {
      ///       /* ... put some data here ... */
      ///     };
      ///
      ///     struct MyData2
      ///     {
      ///       /* ... put some data here ... */
      ///     };
      ///
      ///     struct MyData3
      ///     {
      ///       /* ... put some data here ... */
      ///     };
      ///
      ///     void PrintStuff(const CompositeData &composite)
      ///     {
      ///       std::cout << "Entry types:" << std::endl;
      ///       for (const std::string &label : composite.AllEntries())
      ///       {
      ///         std::cout << "  " << label << std::endl;
      ///       }
      ///     }
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///       composite.Insert<MyData1>();
      ///       composite.Insert<MyData2>();
      ///       composite.Insert<MyData3>();
      ///
      ///       PrintStuff(composite);
      ///
      ///       // The function PrintStuff will print the names of each struct.
      ///     }
      /// \endcode
      ///
      /// \return an ordered (alphabetical) set of all data entries in this
      /// CompositeData.
      public: std::set<std::string> AllEntries() const;

      /// \brief Get an ordered (alphabetical) set of the data entries in
      /// this CompositeData which have not been queried (Get, Insert,
      /// InsertOrAssign, Query, and MakeRequired all perform querying) since
      /// the data was implicitly created (e.g. by a copy or move operation) or
      /// since the last call to ResetQueries(), whichever is more recent. Runs
      /// with O(N) complexity.
      ///
      /// Unqueried data entries might be created by copy/move construction,
      /// copy/move assignment operation, or the Copy(~) function. Using the
      /// copy/move operator or the Copy(~) function will reset the query flag
      /// on any data that gets copied or moved over.
      ///
      /// \note This function can be useful for reporting runtime warnings about
      /// any unsupported data types that have been given to you.
      ///
      /// Example usage:
      ///
      /// \code
      ///     #include <iostream>
      ///     #include <gz/physics/CompositeData.hh>
      ///
      ///     using namespace gz::physics;
      ///
      ///     struct MyData1
      ///     {
      ///       /* ... put some data here ... */
      ///     };
      ///
      ///     struct MyData2
      ///     {
      ///       /* ... put some data here ... */
      ///     };
      ///
      ///     struct MyData3
      ///     {
      ///       /* ... put some data here ... */
      ///     };
      ///
      ///     void DoStuff(CompositeData &composite)
      ///     {
      ///       MyData1 &data1 = composite.Get<MyData1>();
      ///       /* ... do something with data1 ... */
      ///
      ///       MyData2 &data2 = composite.Get<MyData2>();
      ///       /* ... do something with data2 ... */
      ///
      ///
      ///       const std::set<std::string> &unqueried =
      ///                                     composite.UnqueriedEntries();
      ///       if(unqueried.size() > 0)
      ///       {
      ///         std::cout << "I don't know what to do with "
      ///                   << "the following type(s) of data:\n";
      ///         for(const std::string &label : unqueried)
      ///           std::cout << " -- " << label << "\n";
      ///         std::cout << std::endl;
      ///       }
      ///     }
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///       composite.Insert<MyData1>();
      ///       composite.Insert<MyData2>();
      ///       composite.Insert<MyData3>();
      ///
      ///       composite.ResetQueries();
      ///       DoStuff(composite);
      ///
      ///       // The function DoStuff will print out that it does not know
      ///       // what to do with MyData3.
      ///     }
      /// \endcode
      ///
      /// \return an ordered set of the names of unqueried data entries in this
      /// CompositeData.
      public: std::set<std::string> UnqueriedEntries() const;

      /// \brief Make this CompositeData a copy of _other. However, any data
      /// entries in this CompositeData which are marked as required will not be
      /// removed.
      ///
      /// \param[in] _other Another CompositeData
      /// \param[in] _mergeRequirements If true, this object will also take on
      /// the requirements specified by _other. Any objects that are already
      /// marked as required in this CompositeData will remain required. If
      /// false, the requirements of this CompositeData object are unaffected.
      /// \return A reference to this object
      ///
      /// \sa Copy(CompositeData &&, bool)
      /// \sa Merge()
      /// \sa operator=()
      public: CompositeData &Copy(const CompositeData &_other,
                                  const bool _mergeRequirements = false);

      /// \brief An alternative to Copy(const CompositeData &, bool) that takes
      /// advantage of move semantics.
      public: CompositeData &Copy(CompositeData &&_other,
                                  const bool _mergeRequirements = false);

      /// \brief Merge the data from _other into this CompositeData. If there
      /// are any conflicting data entries, the entry from _other will take
      /// precedence.
      ///
      /// \param[in] _other Another CompositeData
      /// \param[in] _mergeRequirements If true, this object will also take on
      /// the requirements specified by _other. Any objects that are already
      /// marked as required in this CompositeData will remain required. If
      /// false, the requirements of this CompositeData object are unaffected.
      /// \return A reference to this object
      ///
      /// \sa Merge(CompositeData &&)
      /// \sa Copy()
      public: CompositeData &Merge(const CompositeData &_other,
                                   const bool _mergeRequirements = false);

      /// \brief An alternative to Merge(const CompositeData &, bool) that takes
      /// advantage of move semantics.
      public: CompositeData &Merge(CompositeData &&_other,
                                   const bool _mergeRequirements = false);

      /// \brief Copy constructor. Same as Copy(_other).
      public: CompositeData(const CompositeData &_other);

      /// \brief Move constructor. Same as Copy(_other).
      public: CompositeData(CompositeData &&_other);

      /// \brief Copy operator. Same as Copy(_other).
      public: CompositeData &operator=(const CompositeData &_other);

      /// \brief Move operator. Same as Copy(_other).
      public: CompositeData &operator=(CompositeData &&_other);

      /// \brief Struct which contains information about a data type within the
      /// CompositeData. See gz/physics/detail/CompositeData.hh for the
      /// definition. This class is public so that helper functions can use it
      /// without being friends of the class.
      /// \private
      public: struct DataEntry;

      // We make this typedef public so that helper functions can use it without
      // being friends of the class.
      public: using MapOfData = std::map<std::string, DataEntry>;

      IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Map from the label of a data object type to its entry
      protected: MapOfData dataMap;
      IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING

      /// \brief Total number of data entries currently in this CompositeData.
      /// Note that this may differ from the size of dataMap, because some
      /// entries in dataMap will be referring to nullptrs.
      protected: std::size_t numEntries;

      /// \brief Total number of unique queries which have been performed since
      /// either construction or the last call to ResetQueries().
      protected: mutable std::size_t numQueries;

      // Declare friendship
      template <typename> friend class detail::PrivateExpectData;
      template <typename> friend class detail::PrivateRequireData;
    };
  }
}

#include "gz/physics/detail/CompositeData.hh"

#endif
