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

#ifndef GZ_PHYSICS_DATASTATUSMASK_HH_
#define GZ_PHYSICS_DATASTATUSMASK_HH_

#include "gz/physics/CompositeData.hh"

namespace ignition
{
  namespace physics
  {
    /// \brief This struct encodes criteria for CompositeData::DataStatus
    /// so that Read and Write operations can be done for specific types
    /// of data (ie. required and unqueried or only existing data).
    /// It is used by OperateOnSpecifiedData at run-time.
    /// \sa CompositeData::DataStatus
    /// \sa OperateOnSpecifiedData
    struct IGNITION_PHYSICS_VISIBLE DataStatusMask
    {
      /// \brief Specify a condition as MUST be true, MUST_NOT be true,
      /// or EITHER.
      enum Condition
      {
        /// \brief MUST be true
        MUST = 0,
        /// \brief MUST_NOT be true (ie. must be false)
        MUST_NOT,
        /// \brief EITHER true or false
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
      /// \param[in] _e Exist condition.
      /// \param[in] _q Queried condition.
      /// \param[in] _r Required condition.
      public: DataStatusMask(const Condition _e = EITHER,
                             const Condition _q = EITHER,
                             const Condition _r = EITHER);

      /// \brief Test whether a single condition is satisfied:
      /// - true if _condition == EITHER
      /// - true if _condition == MUST and boolean _value == true
      /// - true if _condition == MUST_NOT and boolean _value == false
      /// - false otherwise
      /// \param[in] _condition Condition requirement for variable check.
      /// \param[in] _value Boolean value to check.
      /// \return True if condition is satisfied, otherwise false.
      public: static bool ConditionSatisfied(
          const DataStatusMask::Condition _condition,
          const bool _value);

      /// \brief Test whether all conditions of this DataStatusMask are
      /// satisfied by the DataStatus.
      /// \param[in] _status DataStatus to evaluate.
      /// \return True if _status satisfies these conditions.
      public: bool Satisfied(const CompositeData::DataStatus &_status) const;
    };
  }
}


#endif
