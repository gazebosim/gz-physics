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

#ifndef IGNITION_PHYSICS_DETAIL_CREATEJOINTTYPE_HH_
#define IGNITION_PHYSICS_DETAIL_CREATEJOINTTYPE_HH_

#include <tuple>

#define DETAIL_IGN_PHYSICS_PREBAKE_JOINT_POLICY(X, P) \
  template <typename FeaturesT> \
  using X ## P = X<FeaturePolicy ## P, FeaturesT>;

#define DETAIL_IGN_PHYSICS_CREATE_JOINT_TYPE(X) \
  struct X ## Cast : public ::ignition::physics::Feature \
  { \
    IGN_PHYSICS_CREATE_SELECTOR(X) \
    class X ## Identifier { }; \
    \
    public: template <typename PolicyT, typename FeaturesT> \
    class Using : \
      public virtual detail::Aggregate<Select ## X, FeaturesT> \
                               ::template type<PolicyT, FeaturesT>, \
      public virtual detail::Aggregate<detail::JointSelector, FeaturesT> \
                               ::template type<PolicyT, FeaturesT> \
    { \
      public: using Identifier = X ## Identifier; \
      public: using UpcastIdentifiers = std::tuple< \
          X ## Identifier, \
          /* Allow this joint type to be upcast to plain Joint types */ \
          ::ignition::physics::detail::JointIdentifier>; \
      public: using Base = ::ignition::physics::Entity<PolicyT, FeaturesT>; \
    \
      public: Using(const std::shared_ptr<typename Base::Pimpl> &_pimpl, \
                    const ::ignition::physics::Identity &_identity) \
        : Entity<PolicyT, FeaturesT>(_pimpl, _identity) { } \
    }; \
  \
    template <typename PolicyT, typename FeaturesT> \
    class Joint : \
      public virtual ::ignition::physics::Feature::Joint<PolicyT, FeaturesT> \
    { \
      public: std::unique_ptr<Using<PolicyT, FeaturesT>> CastTo ## X() \
      { \
        const ::ignition::physics::Identity id = \
            this->template Interface<X ## Cast>()->CastTo ## X(this->identity);\
        \
        if (!id) \
          return nullptr; \
        \
        return std::make_unique<Using<PolicyT, FeaturesT>>(this->pimpl, id); \
      } \
    \
      public: std::unique_ptr<const Using<PolicyT, FeaturesT>> \
      CastTo ## X() const \
      { \
        const ::ignition::physics::Identity id = \
            this->template Interface<X ## Cast>()->CastTo ## X(this->identity);\
        \
        if (!id) \
          return nullptr; \
        \
        return std::make_unique<const Using<PolicyT, FeaturesT>>( \
            this->pimpl, id); \
      } \
    }; \
    \
    template <typename PolicyT> \
    class Implementation \
      : public virtual ::ignition::physics::Feature::Implementation<PolicyT> \
    { \
      public: virtual ::ignition::physics::Identity CastTo ## X( \
        std::size_t _id) = 0; \
      public: virtual ::ignition::physics::Identity CastTo ## X( \
        std::size_t _id) const = 0; \
    }; \
  }; \
  \
  template <typename FeaturePolicyT, typename FeaturesT> \
  using X = X ## Cast::Using<FeaturePolicyT, FeaturesT>; \
  DETAIL_IGN_PHYSICS_PREBAKE_JOINT_POLICY(X, 3d) \
  DETAIL_IGN_PHYSICS_PREBAKE_JOINT_POLICY(X, 2d) \
  DETAIL_IGN_PHYSICS_PREBAKE_JOINT_POLICY(X, 3f) \
  DETAIL_IGN_PHYSICS_PREBAKE_JOINT_POLICY(X, 2f)

#endif
