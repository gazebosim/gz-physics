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

#ifndef IGNITION_PHYSICS_DETAIL_PREFEATURE_HH_
#define IGNITION_PHYSICS_DETAIL_PREFEATURE_HH_

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      // Forward declarations
      template <typename...> class CombineLists;
      template <bool, typename> struct SelfConflict;

      template <template<typename> class Extractor, typename List>
      struct Extract;
    }
  }
}

// Macros for generating EngineTemplate, LinkTemplate, etc
#define DETAIL_IGN_PHYSICS_MAKE_EXTRACTION_WITH_POLICY(X, P) \
  template <typename List> \
  using X ## P = X ## Template<FeaturePolicy ## P, List>;

#define DETAIL_IGN_PHYSICS_MAKE_EXTRACTION(X) \
  template <typename T> \
  struct X ## Extractor \
  { \
    public: template<typename P> \
    using type = typename T::template X<P>; \
  }; \
  \
  template <typename Policy, typename List> \
  using X ## Template = \
      typename detail::Extract<X ## Extractor, List>::template type<Policy>; \
  DETAIL_IGN_PHYSICS_MAKE_EXTRACTION_WITH_POLICY(X, 3d) \
  DETAIL_IGN_PHYSICS_MAKE_EXTRACTION_WITH_POLICY(X, 2d) \
  DETAIL_IGN_PHYSICS_MAKE_EXTRACTION_WITH_POLICY(X, 3f) \
  DETAIL_IGN_PHYSICS_MAKE_EXTRACTION_WITH_POLICY(X, 2f)

#endif
