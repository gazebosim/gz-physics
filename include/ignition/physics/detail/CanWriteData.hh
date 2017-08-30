#ifndef IGNITION_PHYSICS_DETAIL_CANWRITEDATA_HH_
#define IGNITION_PHYSICS_DETAIL_CANWRITEDATA_HH_

#include "ignition/physics/CanWriteData.hh"

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      template <typename Data, typename Derived, typename CompositeType>
      struct WriteDataOperation
      {
        public: static void Operate(Derived *yourClass, CompositeType &data)
        {
          /// READ CAREFULLY: If you have arrived here by way of a compiler
          /// error, then you have neglected to provide a Write(~) member
          /// function for one of the specified data types that you claimed your
          /// class is able to write to. The compiler error should clearly
          /// indicate the name of the data type that you are neglecting. Simply
          /// implement a member function in your class which overloads the
          /// Write(~) function to accept the data type which the compiler is
          /// indicating.
          ///
          /// If you see an error about "const" and "qualifiers", then make sure
          /// that your class's Write(~) function is const-qualified, i.e. put
          /// the keyword const at the end of its declaration in the function
          /// definition:
          ///
          /// class YourClass
          /// {
          ///   public:
          ///   /* ... */
          ///     void Write(const Data &data) const;
          ///   /* ... */
          /// };
          ///
          ///    ^^^ READ THE ABOVE EXPLANATION IF YOU CANNOT COMPILE ^^^
          yourClass->Write(data.template Get<Data>());
        }
      };
    }

    template <typename Derived, typename Specification>
    CanWriteRequiredData<Derived, Specification>::CanWriteRequiredData()
    {
      CompositeData dummy;
      OperateOnSpecifiedData<
          Specification, FindRequired, detail::WriteDataOperation,
          const Derived>::Operate(static_cast<const Derived*>(this), dummy,
                                  DataStatusMask{}, true);
    }

    template <typename Derived, typename Specification>
    template <typename CompositeType>
    void CanWriteRequiredData<Derived, Specification>::WriteRequiredData(
        CompositeType &_data,
        const WriteOptions &_options) const
    {
      DataStatusMask mask;

      if (_options.skipMissingData)
        mask.exist = DataStatusMask::MUST;

      if (_options.onlyWriteUnqueriedData)
        mask.queried = DataStatusMask::MUST_NOT;

      OperateOnSpecifiedData<
          Specification, FindRequired, detail::WriteDataOperation,
          const Derived>::Operate(static_cast<const Derived*>(this),
                                  _data, mask);
    }

    template <typename Derived, typename Specification>
    CanWriteExpectedData<Derived, Specification>::CanWriteExpectedData()
    {
      CompositeData dummy;
      OperateOnSpecifiedData<
          Specification, FindExpected, detail::WriteDataOperation,
          const Derived>::Operate(static_cast<const Derived*>(this), dummy,
                                  DataStatusMask{}, true);
    }

    template <typename Derived, typename Specification>
    template <typename CompositeType>
    void CanWriteExpectedData<Derived, Specification>::WriteExpectedData(
        CompositeType &_data,
        const WriteOptions &_options) const
    {
      DataStatusMask mask;

      if (_options.skipMissingData)
        mask.exist = DataStatusMask::MUST;

      if (_options.onlyWriteUnqueriedData)
        mask.queried = DataStatusMask::MUST_NOT;

      OperateOnSpecifiedData<
          Specification, FindExpected, detail::WriteDataOperation,
          const Derived>::Operate(static_cast<const Derived*>(this),
                                  _data, mask);
    }
  }
}

#endif
