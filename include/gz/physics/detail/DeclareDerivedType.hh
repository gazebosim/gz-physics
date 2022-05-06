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

#ifndef GZ_PHYSICS_DETAIL_DECLAREDERIVEDTYPE_HH_
#define GZ_PHYSICS_DETAIL_DECLAREDERIVEDTYPE_HH_

#include <memory>
#include <tuple>
#include <utility>

#define DETAIL_IGN_PHYSICS_PREDEFINE_DERIVED_POLICY(Derived, P) \
  template <typename FeaturesT> \
  using Derived ## P = Derived<FeaturePolicy ## P, FeaturesT>; \
  template <typename FeaturesT> \
  using Derived ## P ## Ptr = \
    ::ignition::physics::EntityPtr<Derived<FeaturePolicy ## P, FeaturesT>>; \
  template <typename FeaturesT> \
  using Const ## Derived ## P ## Ptr = \
    ::ignition::physics::EntityPtr<const Derived<FeaturePolicy##P, FeaturesT>>;

#define DETAIL_IGN_PHYSICS_DECLARE_DERIVED_TYPE(Base, Derived) \
  struct Derived ## Cast : public virtual ::ignition::physics::Feature \
  { \
    IGN_PHYSICS_CREATE_SELECTOR(Derived) \
    class Derived ## Identifier { }; \
    \
    public: template <typename PolicyT, typename FeaturesT> \
    class Using : \
      public virtual ::ignition::physics::detail::ExtractAPI< \
        Select ## Derived, FeaturesT>:: \
            template type<PolicyT, FeaturesT>, \
      public virtual ::ignition::physics::detail::ExtractAPI< \
        ::ignition::physics::detail:: Select ## Base, FeaturesT>:: \
            template type<PolicyT, FeaturesT> \
    { \
      public: using Identifier = Derived ## Identifier; \
      public: using UpcastIdentifiers = std::tuple< \
          Derived ## Identifier, \
          /* Allow this derived type to be upcast to plain base types */ \
          ::ignition::physics::detail:: Base ## Identifier>; \
      public: using EntityBase = \
          ::ignition::physics::Entity<PolicyT, FeaturesT>; \
    \
      public: Using(const std::shared_ptr<typename EntityBase::Pimpl> &_pimpl, \
                    const ::ignition::physics::Identity &_identity) \
        : EntityBase(_pimpl, _identity) { } \
      public: Using() = default; \
      public: Using(const Using&) = default; \
      public: Using(Using&&) noexcept = default; \
      \
      /* We customize these operators because of virtual inheritance */ \
      public: Using &operator=(const Using &_other) \
      { \
        static_cast<EntityBase&>(*this) = _other; \
        return *this; \
      } \
      public: Using &operator=(Using &&_other) noexcept \
      { \
        static_cast<EntityBase&>(*this) = std::move(_other); \
        return *this; \
      } \
    }; \
  \
    template <typename PolicyT, typename FeaturesT> \
    class Base : \
      public virtual ::ignition::physics::Feature:: Base <PolicyT, FeaturesT> \
    { \
      public: using CastReturnType = \
          ::ignition::physics::EntityPtr<Using<PolicyT, FeaturesT>>; \
      public: using ConstCastReturnType = \
          ::ignition::physics::EntityPtr<const Using<PolicyT, FeaturesT>>; \
      public: CastReturnType CastTo ## Derived() \
      { \
        return CastReturnType(this->pimpl, \
          this->template Interface<Derived ## Cast>() \
            ->CastTo ## Derived(this->identity)); \
      } \
    \
      public: ConstCastReturnType \
      CastTo ## Derived() const \
      { \
        return ConstCastReturnType(this->pimpl, \
          this->template Interface<Derived ## Cast>() \
            ->CastTo ## Derived(this->identity)); \
      } \
    }; \
    \
    template <typename PolicyT> \
    class Implementation \
      : public virtual ::ignition::physics::Feature::Implementation<PolicyT> \
    { \
      public: virtual ::ignition::physics::Identity CastTo ## Derived( \
        const Identity &_id) const = 0; \
    }; \
  }; \
  \
  template <typename PolicyT, typename FeaturesT> \
  using Derived = Derived ## Cast::Using<PolicyT, FeaturesT>; \
  template <typename PolicyT, typename FeaturesT> \
  using Derived ## Ptr = \
      ::ignition::physics::EntityPtr<Derived<PolicyT, FeaturesT>>; \
  template <typename PolicyT, typename FeaturesT> \
  using Const ## Derived ## Ptr = \
      ::ignition::physics::EntityPtr<const Derived<PolicyT, FeaturesT>>; \
  DETAIL_IGN_PHYSICS_PREDEFINE_DERIVED_POLICY(Derived, 3d) \
  DETAIL_IGN_PHYSICS_PREDEFINE_DERIVED_POLICY(Derived, 2d) \
  DETAIL_IGN_PHYSICS_PREDEFINE_DERIVED_POLICY(Derived, 3f) \
  DETAIL_IGN_PHYSICS_PREDEFINE_DERIVED_POLICY(Derived, 2f)

#endif
