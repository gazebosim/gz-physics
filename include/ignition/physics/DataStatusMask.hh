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

#ifndef IGNITION_PHYSICS_DATASTATUSMASK_HH_
#define IGNITION_PHYSICS_DATASTATUSMASK_HH_

#include "ignition/physics/CompositeData.hh"

namespace ignition
{
  namespace physics
  {
    /// \brief This struct can be used to tweak the behavior of
    /// OperateOnSpecifiedData at run-time.
    struct  DataStatusMask
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
      public: DataStatusMask(const Condition e = EITHER,
                             const Condition q = EITHER,
                             const Condition r = EITHER);

      /// \brief Test whether a single condition is satisfied
      public: static bool ConditionSatisfied(
          const DataStatusMask::Condition condition,
          const bool value);

      /// \brief Test whether all conditions of this DataStatusMask are
      /// satisfied by the DataStatus.
      public: bool Satisfied(const CompositeData::DataStatus &status) const;
    };
  }
}


#endif
