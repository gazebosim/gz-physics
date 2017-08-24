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

#ifndef IGNITION_PHYISCS_OPERATEONSPECIFIEDDATA_HH_
#define IGNITION_PHYISCS_OPERATEONSPECIFIEDDATA_HH_

#include "ignition/physics/SpecifyData.hh"

namespace ignition
{
  namespace physics
  {
    /// \brief This allows us to specify that we are interested in expected
    /// data while performing template metaprogramming.
    template <typename Specification>
    struct FindExpected
    {
      using Data = typename Specification::ExpectedData;
    };

    /// \brief This specialization handles the terminating case where we have
    /// reached a leaf node in the specification tree.
    template <>
    struct FindExpected<void>
    {
      using Data = void;
    };


    /// \brief This allows us to specify that we are interested in required
    /// data while performing template metaprogramming.
    template <typename Specification>
    struct FindRequired
    {
      using Data = typename Specification::RequiredData;
    };

    /// \brief This specialization handles the terminating case where we have
    /// reached a leaf node in the specification tree.
    template <>
    struct FindRequired<void>
    {
      using Data = void;
    };

    /// \brief This struct can be used to tweak the behavior of
    /// OperateOnSpecifiedData at run-time.
    struct IGNITION_COMMON_VISIBLE OperationMask
    {
      enum Condition
      {
        MUST = 0,
        MUST_NOT,
        EITHER
      };

      /// \brief MUST means the type must exist in the CompositeData in order
      /// to be operated on. MUST_NOT means it must not exist (this can be
      /// used to decide whether to create the object).
      public: Condition exist;

      /// \brief MUST means the type must be queried. MUST_NOT means it must
      /// not be queried.
      public: Condition queried;

      /// \brief MUST means the type must be required by the CompositeData.
      /// MUST_NOT means it must not be required.
      ///
      /// Superficially, this might appear to overlap with the role of
      /// FindRequired and FindExpected, but those templates are filtering
      /// data types at compile-time whereas this will filter them during
      /// runtime. Requirements can be promoted during run-time, so they won't
      /// necessarily match the static compile-time requirements, and a user
      /// might want to dynamically choose which requirement level to operate
      /// on during run-time.
      public: Condition required;

      /// \brief Default constructor. Everything is set to EITHER so that
      /// nothing is masked.
      public: OperationMask(const Condition e = EITHER,
                            const Condition q = EITHER,
                            const Condition r = EITHER);

      /// \brief Test whether a single condition is satisfied
      public: static bool ConditionSatisfied(
          const OperationMask::Condition condition,
          const bool value);

      /// \brief Test whether all conditions of this OperationMask are
      /// satisfied by the DataStatus.
      public: bool Satisfied(const CompositeData::DataStatus &status) const;
    };

    /// \brief OperateOnSpecifiedData allows us to statically analyze
    /// whether a class (Performer) can perform an operation on all the
    /// relevant types within a CompositeData Specification. It also provides
    /// functions for performing that operation.
    ///
    /// Specification can be created using SpecifyData, ExpectData, and
    /// RequireData.
    ///
    /// SpecFinder is a templated class that can be passed a Specification and
    /// return a Data type (or return void if Specification does not contain
    /// the desired data specification). This is already implemented as
    /// FindExpected and FindRequired, but custom SpecFinders are permitted.
    ///
    /// Operation is a templated class which accepts template arguments of
    /// Data, Performer, and CompositeType. It must also provide a static
    /// function called Operate which accepts arguments of type Performer* and
    /// CompositeData&.
    ///
    /// Performer is the type of class which is expected to perform the
    /// operation.
    template <typename Specification,
              template<typename> class SpecFinder,
              template<typename, typename, typename> class Operation,
              class Performer>
    class OperateOnSpecifiedData
    {
      /// \brief Operate is the recommended entry point for using
      /// OperateOnSpecifiedData. This can be given a performer, a
      /// CompositeType object, and an OperationMask to determine its
      /// behavior.
      ///
      /// Setting onlyCompile to true will short-circuit the entire operation.
      /// This can be useful for static analysis. The function will cost very
      /// nearly nothing during run-time, but it will prevent code from
      /// compiling if its conditions are not met.
      public: template <typename CompositeType>
      static void Operate(
          Performer *performer,
          CompositeType &data,
          const OperationMask &mask,
          const bool onlyCompile = false);

      /// \brief When SubSpecification is able to provide one of the desired
      /// data specifications, this overload gets called.
      public: template <typename Data, typename SubSpecification,
                        typename CompositeType>
      static void SubOperate(
          type<Data>, type<SubSpecification>,
          Performer *performer, CompositeType &data,
          const OperationMask &mask);

      /// \brief When SubSpecification could not provide one of the desired
      /// data specifications, we will instead search it for
      /// sub-specifications.
      public: template <typename SubSpecification, typename CompositeType>
      static void SubOperate(
          type<void>, type<SubSpecification>,
          Performer *performer, CompositeType &data,
          const OperationMask &mask);

      /// \brief When SubSpecification does not have the desired data
      /// specifications, nor has any sub-specifications, we terminate here.
      public: template <typename CompositeType>
      static void SubOperate(
          type<void>, type<void>,
          Performer*, CompositeType&,
          const OperationMask&);
    };
  }
}

#include "ignition/physics/detail/OperateOnSpecifiedData.hh"

#endif
