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

#ifndef GZ_PHYSICS_DECLARESHAPETYPE_HH_
#define GZ_PHYSICS_DECLARESHAPETYPE_HH_

#include <gz/physics/FeatureList.hh>
#include <gz/physics/detail/DeclareDerivedType.hh>

/// \brief Given a shape type named CustomShapeType, this macro creates the
/// following classes:
///
/// class CustomShapeTypeCast
///  - A Feature class that allows plain Shape objects to downcast themselves to
///    CustomShapeType, as long as it is truly an instance of a CustomShapeType.
///    This class provides the function Shape::CastToCustomShapeType() when
///    added to a Shape's FeatureList.
///
/// template<P, F> class CustomShapeType
///  - An Entity class that includes the API of both the plain Shape class and
///    the CustomShapeType, as defined by FeaturePolicy P and FeatureList F.
///
/// template <F> class CustomShapeType3d
/// template <F> class CustomShapeType2d
/// template <F> class CustomShapeType3f
/// template <F> class CustomShapeType2f
///  - Similar to CustomShapeType<P,F>, except P is replaced with the predefined
///    Feature Policies.
///
/// Physics engine plugin developers must implement the virtual function
///
/// \code
/// gz::physics::Identity CastToCustomShapeType(const Identity &_id) const
/// \endcode
///
/// if their physics engine plugin wants to be able to provide CustomShapeType
/// features.
#define IGN_PHYSICS_DECLARE_SHAPE_TYPE(CustomShapeType) \
  DETAIL_IGN_PHYSICS_DECLARE_DERIVED_TYPE(Shape, CustomShapeType)

#endif
