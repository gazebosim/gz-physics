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

#ifndef GZ_PHYISCS_OPERATEONSPECIFIEDDATA_HH_
#define GZ_PHYISCS_OPERATEONSPECIFIEDDATA_HH_

#include <string>
#include <unordered_set>

#include "gz/physics/SpecifyData.hh"
#include "gz/physics/DataStatusMask.hh"

namespace gz
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
      /// \brief Operate is the recommended entry point for using
      /// OperateOnSpecifiedData. This can be given a performer, a CompositeType
      /// object, and a DataStatusMask to determine its behavior.
      ///
      /// For each type in Specification, the _performer will query _data for
      /// the type, and then perform Operation on that data type. The _mask
      /// information is used to tune the behavior to only perform the
      /// operations on components of _data that meet the criteria.
      ///
      /// Setting onlyCompile to true will short-circuit the entire operation.
      /// This can be useful for static analysis. The function will cost very
      /// nearly nothing during run-time, but it will prevent code from
      /// compiling if its conditions are not met.
      ///
      /// \param[in,out] _performer
      ///   The object that will perform the operation.
      /// \param[in,out] _data
      ///   The object that will be operated on.
      /// \param[in] _mask
      ///   An object to tune which data components are operated on.
      /// \param[in] _onlyCompile
      ///   Have this function compile to make sure that it can compile, but do
      ///   not perform any operations.
      public: template <typename CompositeType>
      static void Operate(
          Performer *_performer,
          CompositeType &_data,
          const DataStatusMask &_mask,
          const bool _onlyCompile = false);


      // -------------------- Private API -----------------------

      /// The data structure that will be used to ensure that the same data does
      /// not get operated on twice, even if it is listed twice (redundantly) in
      /// the specification.
      private: using History = std::unordered_set<std::string>;

      /// \brief When SubSpecification is able to provide one of the desired
      /// data specifications, this overload gets called.
      private: template <typename Data, typename SubSpecification,
                        typename CompositeType>
      static void SubOperate(
          detail::type<Data>, detail::type<SubSpecification>,
          Performer *_performer, CompositeType &_data,
          const DataStatusMask &_mask,
          History &_history);

      /// \brief When SubSpecification could not provide one of the desired data
      /// specifications, we will instead search it for sub-specifications.
      private: template <typename SubSpecification, typename CompositeType>
      static void SubOperate(
          detail::type<void>, detail::type<SubSpecification>,
          Performer *_performer, CompositeType &_data,
          const DataStatusMask &_mask,
          History &_history);

      /// \brief When SubSpecification does not have the desired data
      /// specifications, nor has any sub-specifications, we terminate here.
      private: template <typename CompositeType>
      static void SubOperate(
          detail::type<void>, detail::type<void>,
          Performer*, CompositeType&,
          const DataStatusMask&,
          History&);
    };
  }
}

#include "gz/physics/detail/OperateOnSpecifiedData.hh"

#endif
