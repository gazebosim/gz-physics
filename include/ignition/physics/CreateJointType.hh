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

#ifndef IGNITION_PHYSICS_CREATEJOINTTYPE_HH_
#define IGNITION_PHYSICS_CREATEJOINTTYPE_HH_

#include <memory>

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/Geometry.hh>
#include <ignition/physics/TemplateHelpers.hh>
#include <ignition/physics/detail/CreateJointType.hh>

/// \brief Given a joint type named CustomJointType, this macro creates the
/// following classes:
///
/// class CustomJointTypeCast
///  - A Feature class that allows plain Joint objects to downcast themselves to
///    CustomJointType, as long as it is truly an instance of a CustomJointType.
///
/// template<P, F> class CustomJointType
///  - An Entity class that includes the API of both the plain Joint class and
///    the CustomJointType, as defined by FeaturePolicy P and FeatureList F.
///
/// template <F> class CustomJointType3d
/// template <F> class CustomJointType2d
/// template <F> class CustomJointType3f
/// template <F> class CustomJointType2f
///  - Similar to CustomJointType<P,F>, except P is replaced with the predefined
///    FeaturePolicies.
#define IGN_PHYSICS_CREATE_JOINT_TYPE( CustomJointType ) \
  DETAIL_IGN_PHYSICS_CREATE_JOINT_TYPE( CustomJointType )

#endif
