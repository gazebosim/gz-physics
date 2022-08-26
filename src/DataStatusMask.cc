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

#include "gz/physics/DataStatusMask.hh"

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    DataStatusMask::DataStatusMask(const Condition _e,
                                   const Condition _q,
                                   const Condition _r)
      : exist(_e),
        queried(_q),
        required(_r)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    bool DataStatusMask::ConditionSatisfied(
        const DataStatusMask::Condition _condition,
        const bool _value)
    {
      return ((DataStatusMask::MUST == _condition && _value)
           || (DataStatusMask::MUST_NOT == _condition && !_value)
           || (DataStatusMask::EITHER == _condition));
    }

    /////////////////////////////////////////////////
    bool DataStatusMask::Satisfied(
        const CompositeData::DataStatus &_status) const
    {
      return (ConditionSatisfied(this->exist, _status.exists)
           && ConditionSatisfied(this->queried, _status.queried)
           && ConditionSatisfied(this->required, _status.required));
    }
  }
}
