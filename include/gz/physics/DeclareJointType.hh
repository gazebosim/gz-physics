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

#ifndef GZ_PHYSICS_DECLAREJOINTTYPE_HH_
#define GZ_PHYSICS_DECLAREJOINTTYPE_HH_

#include <gz/physics/FeatureList.hh>
#include <gz/physics/detail/DeclareDerivedType.hh>

/// \brief Given a joint type named CustomJointType, this macro creates the
/// following classes:
///
/// class CustomJointTypeCast
///  - A Feature class that allows plain Joint objects to downcast themselves to
///    CustomJointType, as long as it is truly an instance of a CustomJointType.
///    This class provides the function Joint::CastToCustomJointType() when
///    added to a Joint's FeatureList.
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
///    Feature Policies.
///
/// Physics engine plugin developers must implement the virtual function
///
/// \code
/// gz::physics::Identity CastToCustomJointType(const Identity &_id) const
/// \endcode
///
/// if their physics engine plugin wants to be able to provide CustomJointType
/// features.
#define GZ_PHYSICS_DECLARE_JOINT_TYPE(CustomJointType) \
  DETAIL_GZ_PHYSICS_DECLARE_DERIVED_TYPE(Joint, CustomJointType)

#endif
