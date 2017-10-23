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
    class  CompositeData
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
      ///     #include <iostream>
      ///     #include <ignition/physics/CompositeData.hh>
      ///
      ///     using namespace ignition::physics;
      ///
      ///     // Create a data structure called MyData
      ///     struct MyData
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData)
      ///
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
      ///       MyData& data = composite.Get<MyData>();
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
      ///
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
      ///
      /// Example usage:
      ///
      ///     #include <iostream>
      ///     #include <ignition/physics/CompositeData.hh>
      ///
      ///     using namespace ignition::physics;
      ///
      ///     // Create a data structure called MyData
      ///     struct MyData
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData)
      ///
      ///       MyData(const std::string& arg = "")
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
      ///       // Create an object of type MyData and store it in "composite".
      ///       // A reference to the newly created object is returned.
      ///       MyData& data = composite.Create<MyData>("some argument");
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
      ///       // Create a new object of type MyData, deleting the old one.
      ///       // Note that the reference named "data" will be INVALIDATED and
      ///       // MUST NOT be used again after this function call.
      ///       composite.Create<MyData>();
      ///
      ///       // This will print out nothing but a newline, because the last
      ///       // call to Create<MyData>() will have replaced the old modified
      ///       // instance of MyData which was constructed without any
      ///       // arguments.
      ///       std::cout << composite.Get<MyData>().myString << std::endl;
      ///     }
      ///
      public: template <typename Data, typename... Args>
      Data& Create(Args&&... args);

      /// \brief If a Data type object is available in this CompositeData,
      /// this will return a reference to it. Otherwise, it will create a new
      /// Data type object with the arguments provided, and return a reference
      /// to the newly created object.
      ///
      /// Example usage:
      ///
      ///     #include <iostream>
      ///     #include <cassert>
      ///     #include <ignition/physics/CompositeData.hh>
      ///
      ///     using namespace ignition::physics;
      ///
      ///     // Create a data structure called MyData
      ///     struct MyData
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData)
      ///
      ///       MyData(const std::string& arg = "")
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
      ///       // Create an object of type MyData and store it in "composite".
      ///       // A reference to the newly created object is returned.
      ///       MyData& data = composite.GetOrCreate<MyData>("some argument");
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
      ///       MyData& altData = composite.GetOrCreate<MyData>("another argument");
      ///
      ///       // The reference to "data" is still perfectly VALID, and in fact
      ///       // its underlying pointer is equal to the underlying pointer of
      ///       // "altData".
      ///       assert( (&data) == (&altData) );
      ///
      ///       // This will print out "I modified this data" because there have
      ///       // not been any function calls that can alter the value of
      ///       // myString in the time since that modification was made.
      ///       std::cout << composite.GetOrCreate<MyData>().myString << std::endl;
      ///     }
      ///
      public: template <typename Data, typename... Args>
      Data& GetOrCreate(Args&&... args);

      /// \brief This will remove a Data type object from this CompositeData and
      /// delete it if one is present. Otherwise, it will do nothing. Data types
      /// that are marked as required will not (and cannot) be removed.
      ///
      /// If the data was successfully removed or did not exist to begin with,
      /// this returns true. If the data was marked as required and therefore
      /// not removed, this returns false.
      ///
      /// Example usage:
      ///
      ///     #include <iostream>
      ///     #include <ignition/physics/CompositeData.hh>
      ///
      ///     using namespace ignition::physics;
      ///
      ///     // Create a data structure called MyData
      ///     struct MyData
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData)
      ///
      ///       MyData(const std::string& arg = "")
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
      ///       // Create an object of type MyData and store it in "composite".
      ///       // A reference to the newly created object is returned.
      ///       MyData& data = composite.GetOrCreate<MyData>("some argument");
      ///
      ///       // Print out "some argument"
      ///       std::cout << composite.GetOrCreate<MyData>("another argument").myString << std::endl;
      ///
      ///       // Remove the MyData object. Note that "data" is now INVALID and
      ///       // must never be used again after this function call.
      ///       composite.Remove<MyData>();
      ///
      ///       // Print out "another argument"
      ///       std::cout << composite.GetOrCreate<MyData>("another argument").myString << std::endl;
      ///     }
      ///
      public: template <typename Data>
      bool Remove();

      /// \brief Use these flags in Query(), Has(), and StatusOf() to change
      /// their effects on the meta info of the data being queried.
      ///
      /// See UnqueriedEntries() for more on the "queried" flag.
      enum QueryMode
      {
        /// \brief Performing the operation will cause an unqueried Data's
        /// status to flip to queried. Data that is already marked as queried
        /// will be unaffected.
        QUERY_NORMAL = 0,

        /// \brief Performing the operation has no effect on whether data is
        /// marked as queried.
        QUERY_SILENT
      };

      /// \brief Query this CompositeData for a Data type object. If it contains
      /// an instance of a Data type object, it gets returned as a Data*.
      /// Otherwise, a nullptr is returned.
      ///
      /// If "mode" is set to QUERY_SILENT, then calling this function will not
      /// cause the "queried" flag to change (see UnqueriedEntries() for more
      /// on the "queried" flag).
      ///
      /// Example usage:
      ///
      ///     #include <iostream>
      ///     #include <ignition/physics/CompositeData.hh>
      ///
      ///     using namespace ignition::physics;
      ///
      ///     struct MyDataWithoutDefault
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyDataWithoutDefault)
      ///
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
      ///     void SetValueIfAvailable(const int value, CompositeData& composite)
      ///     {
      ///       // This CANNOT compile because MyDataWithoutDefault does not
      ///       // provide a default constructor
      ///       //MyDataWithoutDefault& data = composite.Get<MyDataWithoutDefault>();
      ///
      ///       // Get a pointer to a MyDataWithoutDefault instance if one is
      ///       // available, otherwise we get a nullptr.
      ///       MyDataWithoutDefault* data = composite.Query<MyDataWithoutDefault>();
      ///
      ///       if(data)
      ///         data->myValue = value;
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
      ///       // Create function and passing it an argument for the object's
      ///       // constructor.
      ///       composite.Create<MyDataWithoutDefault>(123);
      ///
      ///       // This will print out "123" because the object will not be
      ///       // re-created by GetOrCreate. We can call GetOrCreate because it
      ///       // can use the argument we pass in to create an instance of
      ///       // MyDataWithoutDefault if an instance of it did not already
      ///       // exist, unlike Get which can only call the default
      ///       // constructor.
      ///       std::cout << composite.GetOrCreate<MyDataWithoutDefault>(1).myValue << std::endl;
      ///
      ///       // This will set myValue to 5 because "composite" contains an
      ///       // instance of MyDataWithoutDefault.
      ///       SetValueIfAvailable(5, composite);
      ///
      ///       // This will print out "5" because that was the value set by
      ///       // SetValueIfAvailable.
      ///       std::cout << composite.GetOrCreate<MyDataWithoutDefault>(3).myValue << std::endl;
      ///     }
      ///
      public: template <typename Data>
      Data* Query(const QueryMode mode = QUERY_NORMAL);

      /// \brief Const-qualified version of Query. This can be used to retrieve
      /// data from a `const CompositeData`.
      ///
      /// If "mode" is set to QUERY_SILENT, then calling this function will not
      /// cause the "queried" flag to change (see UnqueriedEntries() for more
      /// on the "queried" flag).
      ///
      /// Example usage:
      ///
      ///     #include <iostream>
      ///     #include <ignition/physics/CompositeData.hh>
      ///
      ///     using namespace ignition::physics;
      ///
      ///     struct MyData
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData)
      ///
      ///       std::string myString;
      ///     };
      ///
      ///     void PrintStringIfAvailable(const CompositeData& composite)
      ///     {
      ///       // The following Get<T>() CANNOT compile because "composite" is
      ///       // a const-qualified reference, but Get<T>() is NOT a
      ///       // const-qualified member function. Get<T>() cannot be
      ///       // const-qualified because it needs to create an instance of
      ///       // type T if an instance of that type was not already available
      ///       // in the CompositeData object, and that would mean modifying
      ///       // the CompositeData object, which violates const-correctness.
      ///       // This is similarly the case for Create<T>(~) and GetOrCreate<T>(~).
      ///       //
      ///       //MyData& data = composite.Get<T>(); // error!
      ///
      ///       // Instead, we use the const-qualified Query<T>() function which
      ///       // can return a const-qualified pointer to the data instance.
      ///       const MyData* data = composite.Query<MyData>();
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
      ///
      public: template <typename Data>
      const Data* Query(const QueryMode mode = QUERY_NORMAL) const;

      /// \brief Returns true if this CompositeData has an object of type Data,
      /// otherwise returns false. This is literally equivalent to
      /// (nullptr != Query<Data>(mode)).
      ///
      /// If "mode" is set to QUERY_SILENT, then calling this function will not
      /// cause the "queried" flag to change (see UnqueriedEntries() for more
      /// on the "queried" flag).
      public: template <typename Data>
      bool Has(const QueryMode mode = QUERY_NORMAL) const;

      /// \brief Struct that describes the status of data.
      struct DataStatus
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
      /// Note that using QUERY_NORMAL will mark the data as queried, but the
      /// DataStatus that is returned will indicate the query status prior to
      /// calling this function, so it might not reflect the current status.
      /// Using QUERY_SILENT will guarantee that the returned status reflects
      /// the current status. We use this behavior because otherwise
      /// QUERY_NORMAL would cause information to get lost. With the behavior
      /// the way it is, you can always ascertain both (1) what the query status
      /// was before calling this function, and (2) what it is now that the
      /// function has been called (see UnqueriedEntries() for more on the
      /// "queried" flag).
      ///
      /// See Unquery<Data>() and MakeRequired<Data>() for example usage.
      public: template <typename Data>
      DataStatus StatusOf(const QueryMode mode = QUERY_NORMAL) const;

      /// \brief Returns true if this CompositeData has a Data type object
      /// which was marked as queried, and that object is now marked as
      /// unqueried. If an object of that type does not exist or it was already
      /// unqueried, this returns false.
      ///
      /// This function is const-qualified because the query flags are declared
      /// as mutable.
      ///
      /// Example usage:
      ///
      ///     #include <cassert>
      ///     #include <ignition/physics/CompositeData.hh>
      ///
      ///     using namespace ignition::physics;
      ///
      ///     struct MyData
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData)
      ///
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
      ///       composite.Create<MyData>();
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
      ///       status = composite.StatusOf<MyData>(CompositeData::QUERY_SILENT);
      ///       // It should be unqueried because we turned off its query flag
      ///       // using Unquery<MyData>().
      ///       assert(!status.queried);
      ///
      ///       // Since we used QUERY_SILENT mode in our last call to
      ///       // StatusOf<MyData>(), MyData will still be considered unqueried
      ///       // in "composite" right now.
      ///
      ///       status = composite.StatusOf<MyData>(CompositeData::QUERY_NORMAL);
      ///       // The queried flag held by "status" should still be false
      ///       // because it reflects whether MyData was queried BEFORE the
      ///       // function StatusOf<MyData>() was called.
      ///       assert(!status.queried);
      ///
      ///       // Since we used QUERY_NORMAL mode in our last call to
      ///       // StatusOf<MyData>(), MyData will now be considered queried
      ///       // inside of "composite".
      ///
      ///       status = composite.StatusOf<MyData>();
      ///       assert(status.queried);
      ///     }
      ///
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
      ///     #include <cassert>
      ///     #include <ignition/physics/CompositeData.hh>
      ///
      ///     using namespace ignition::physics;
      ///
      ///     struct MyData
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData)
      ///
      ///       MyData(const std::string& arg = "")
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
      ///
      public: template <typename Data, typename... Args>
      Data& MakeRequired(Args&&... args);

      /// \brief Returns true if the specified Data type is required by this
      /// CompositeData object. Otherwise, returns false.
      ///
      /// For more on required data, see MakeRequired<Data>().
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
      /// not been queried. See UnqueriedEntries() for more information about
      /// the "queried" flag.
      std::size_t NumUnqueriedEntries() const;

      /// \brief Reset the query flags on all data entries. This will make it
      /// appear as though no entries have ever been queried. See
      /// UnqueriedEntries() for more information about the "queried" flag.
      ///
      /// It is good practice to call this function before returning a
      /// CompositeData from a function and handing it off to another segment of
      /// a pipeline, because sometimes the compiler inappropriately elides the
      /// copy/move constructor/operators and passes along the state of the
      /// queries, even though it should not.
      void ResetQueries() const;

      /// \brief Returns an ordered (alphabetical) set of the data entries in
      /// this CompositeData which have not been queried (Get, Create,
      /// GetOrCreate, Query, Has, and MakeRequired all perform querying) since
      /// the data was created (not using the aforementioned functions) or since
      /// the last call to ResetQueries(), whichever is more recent. Runs with
      /// O(1) complexity.
      ///
      /// Unqueried data entries might be created by copy/move construction,
      /// copy/move assignment operation, or the Copy(~) function. Using the
      /// copy/move operator or the Copy(~) function will reset the query flag
      /// on any data that gets copied or moved over. This can be useful for
      /// reporting runtime warnings about any unsupported data types that have
      /// been given to you.
      ///
      /// Example usage:
      ///
      ///     #include <iostream>
      ///     #include <ignition/physics/CompositeData.hh>
      ///
      ///     using namespace ignition::physics;
      ///
      ///     struct MyData1
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData1)
      ///       /* ... put some data here ... */
      ///     };
      ///
      ///     struct MyData2
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData2)
      ///       /* ... put some data here ... */
      ///     };
      ///
      ///     struct MyData3
      ///     {
      ///       IGN_PHYSICS_DATA_LABEL(MyData3)
      ///       /* ... put some data here ... */
      ///     };
      ///
      ///     void DoStuff(CompositeData& composite)
      ///     {
      ///       MyData1& data1 = composite.Get<MyData1>();
      ///       /* ... do something with data1 ... */
      ///
      ///       MyData2& data2 = composite.Get<MyData2>();
      ///       /* ... do something with data2 ... */
      ///
      ///
      ///       const std::set<std::string>& unqueried = composite.UnqueriedEntries();
      ///       if(unqueried.size() > 0)
      ///       {
      ///         std::cout << "I don't know what to do with the following type(s) of data:\n";
      ///         for(const std::string& label : unqueried)
      ///           std::cout << " -- " << label << "\n";
      ///         std::cout << std::endl;
      ///       }
      ///     }
      ///
      ///     int main()
      ///     {
      ///       CompositeData composite;
      ///       composite.Create<MyData1>();
      ///       composite.Create<MyData2>();
      ///       composite.Create<MyData3>();
      ///
      ///       composite.ResetQueries();
      ///       DoStuff(composite);
      ///
      ///       // The function DoStuff will print out that it does not know
      ///       // what to do with MyData3.
      ///     }
      ///
      std::set<std::string> UnqueriedEntries() const;


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

      /// \brief Struct which contains information about a data type within the
      /// CompositeData. See ignition/physics/detail/CompositeData.hh for the
      /// definition.
      public: struct DataEntry;

      public: using MapOfData = std::map<std::string, DataEntry>;
      /// \brief Map from the label of a data object type to its entry
      protected: MapOfData dataMap;

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

#include "ignition/physics/detail/CompositeData.hh"

#endif
