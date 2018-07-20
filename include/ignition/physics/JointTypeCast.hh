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

#ifndef IGNITION_PHYSICS_JOINTTYPECAST_HH_
#define IGNITION_PHYSICS_JOINTTYPECAST_HH_

#include <ignition/physics/Feature.hh>

namespace ignition
{
  namespace physics
  {
    class IGNITION_PHYSICS_VISIBLE JointTypeCast : public virtual Feature
    {
      template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: template<template <typename> class TypeSelector>
        auto CastTo() ->
        std::unique_ptr<typename JointType::Using<PolicyT, FeaturesT>>;


        public: template<template <typename> class TypeSelector>
        auto CastTo() const ->
        std::unique_ptr<const typename JointType::Using<PolicyT, FeaturesT>>;
      };


    };
  }
}

#endif
