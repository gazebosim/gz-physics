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

#ifndef IGNITION_PHYSICS_DETAIL_OPERATEONSPECIFIEDDATA_HH_
#define IGNITION_PHYSICS_DETAIL_OPERATEONSPECIFIEDDATA_HH_

#include "ignition/physics/OperateOnSpecifiedData.hh"

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    #define IGN_PHYSICS_OPERATEONSPECIFIEDDATA_TEMPLATES \
    template <typename Specification, \
            template<typename> class SpecFinder, \
            template<typename, typename, typename> class Operation, \
            class Performer>

    #define IGN_PHYSICS_OPERATEONSPECIFIEDDATA_PREFIX \
    void OperateOnSpecifiedData<Specification, SpecFinder, Operation, Performer>

    /////////////////////////////////////////////////
    IGN_PHYSICS_OPERATEONSPECIFIEDDATA_TEMPLATES
    template <typename CompositeType>
    IGN_PHYSICS_OPERATEONSPECIFIEDDATA_PREFIX::Operate(
        Performer *performer, CompositeType &data,
        const DataStatusMask &mask, const bool onlyCompile)
    {
      if (onlyCompile)
        return;

      History history;
      history.reserve(CountUpperLimitOfSpecifiedData<Specification, SpecFinder>());

      SubOperate(type<typename SpecFinder<Specification>::Data>(),
                 type<Specification>(), performer, data, mask, history);
    }

    /////////////////////////////////////////////////
    IGN_PHYSICS_OPERATEONSPECIFIEDDATA_TEMPLATES
    template <typename Data, typename SubSpecification, typename CompositeType>
    IGN_PHYSICS_OPERATEONSPECIFIEDDATA_PREFIX::SubOperate(
        type<Data>, type<SubSpecification>,
        Performer *performer, CompositeType &data,
        const DataStatusMask &mask,
        History &history)
    {
      // This gets called when SubSpecification is able to provide one of the
      // desired data type specifications.

      if (mask.Satisfied(data.template StatusOf<Data>(
                           CompositeData::QUERY_SILENT)))
      {
        const bool inserted = history.insert(
              Data::IgnPhysicsTypeLabel()).second;

        if(inserted)
        {
          // We have found a specified type that matches what we want, so we
          // will call operate on it.
          Operation<Data, Performer, CompositeType>::Operate(performer, data);
        }
      }

      // Here we call the version of this function that performs branching,
      // just in case this SubSpecification also contains SubSpecifications
      // of its own.
      //
      // It's worth noting that in our current implementation, this will not
      // do anything, because every node in a specification will only (a)
      // specify a data type, or (b) provide two sub-specifications, but it
      // will never do both (a) and (b). Calling this function here is a
      // good idea just in case (1) we decide to change the way we structure
      // the specification classes, or (2) a user/developer wants to
      // leverage this class for their own custom specification framework
      // which does not make all the same assumptions as ours. It also
      // doesn't affect performance, because it will simply compile to a
      // no-op.
      SubOperate(type<void>(), type<SubSpecification>(),
                 performer, data, mask, history);
    }

    /////////////////////////////////////////////////
    IGN_PHYSICS_OPERATEONSPECIFIEDDATA_TEMPLATES
    template <typename SubSpecification, typename CompositeType>
    IGN_PHYSICS_OPERATEONSPECIFIEDDATA_PREFIX::SubOperate(
        type<void>, type<SubSpecification>,
        Performer *performer, CompositeType &data,
        const DataStatusMask &mask, History &history)
    {
      // SubSpecification did not specify a type that matches what we want,
      // so we will search it to see if it has its own sub-specifications.

      using Sub1 =  typename SubSpecification::SubSpecification1;
      SubOperate(type<typename SpecFinder<Sub1>::Data>(), type<Sub1>(),
                 performer, data, mask, history);

      using Sub2 = typename SubSpecification::SubSpecification2;
      SubOperate(type<typename SpecFinder<Sub2>::Data>(), type<Sub2>(),
                 performer, data, mask, history);
    }

    /////////////////////////////////////////////////
    IGN_PHYSICS_OPERATEONSPECIFIEDDATA_TEMPLATES
    template <typename CompositeType>
    IGN_PHYSICS_OPERATEONSPECIFIEDDATA_PREFIX::SubOperate(
        type<void>, type<void>, Performer*,
        CompositeType&, const DataStatusMask&, History&)
    {
      // We reached a leaf in the specification, so we are done with this
      // branch. Do nothing.
    }
  }
}

#endif
