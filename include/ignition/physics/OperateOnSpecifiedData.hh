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

#include <unordered_set>

#include "ignition/physics/SpecifyData.hh"
#include "ignition/physics/DataStatusMask.hh"

namespace ignition
{
  namespace physics
  {
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
      /// The data structure that will be used to ensure that the same data does
      /// not get operated on twice, even if it is listed twice (redundantly) in
      /// the specification.
      using History = std::unordered_set<std::string>;

      /// \brief Operate is the recommended entry point for using
      /// OperateOnSpecifiedData. This can be given a performer, a CompositeType
      /// object, and a DataStatusMask to determine its behavior.
      ///
      /// Setting onlyCompile to true will short-circuit the entire operation.
      /// This can be useful for static analysis. The function will cost very
      /// nearly nothing during run-time, but it will prevent code from
      /// compiling if its conditions are not met.
      public: template <typename CompositeType>
      static void Operate(
          Performer *performer,
          CompositeType &data,
          const DataStatusMask &mask,
          const bool onlyCompile = false);

      /// \brief When SubSpecification is able to provide one of the desired
      /// data specifications, this overload gets called.
      public: template <typename Data, typename SubSpecification,
                        typename CompositeType>
      static void SubOperate(
          type<Data>, type<SubSpecification>,
          Performer *performer, CompositeType &data,
          const DataStatusMask &mask,
          History &history);

      /// \brief When SubSpecification could not provide one of the desired data
      /// specifications, we will instead search it for sub-specifications.
      public: template <typename SubSpecification, typename CompositeType>
      static void SubOperate(
          type<void>, type<SubSpecification>,
          Performer *performer, CompositeType &data,
          const DataStatusMask &mask,
          History &history);

      /// \brief When SubSpecification does not have the desired data
      /// specifications, nor has any sub-specifications, we terminate here.
      public: template <typename CompositeType>
      static void SubOperate(
          type<void>, type<void>,
          Performer*, CompositeType&,
          const DataStatusMask&,
          History&);
    };
  }
}

#include "ignition/physics/detail/OperateOnSpecifiedData.hh"

#endif
