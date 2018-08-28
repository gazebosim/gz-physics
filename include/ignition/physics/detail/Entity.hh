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

#ifndef IGNITION_PHYSICS_DETAIL_ENTITY_HH_
#define IGNITION_PHYSICS_DETAIL_ENTITY_HH_

#include <memory>
#include <tuple>

#include <ignition/plugin/SpecializedPluginPtr.hh>
#include <ignition/physics/Entity.hh>
#include <ignition/physics/TemplateHelpers.hh>
#include <ignition/plugin/TemplateHelpers.hh>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      /////////////////////////////////////////////////
      /// \private This class is used to determine what type of
      /// SpecializedPluginPtr should be used by the entities provided by a
      /// plugin.
      template <typename Policy, typename Features>
      struct DeterminePlugin;

      /// \private Implementation of DeterminePluginType
      template <typename Policy, typename... Features>
      struct DeterminePlugin<Policy, std::tuple<Features...>>
      {
        using type = plugin::SpecializedPluginPtr<
            typename Features::template Implementation<Policy>...>;
      };

      /////////////////////////////////////////////////
      /// \private Inspired by https://stackoverflow.com/a/26288164
      /// This class provides a static constexpr member named `value` which is
      /// true if T is one of the entries of Tuple, and false otherwise.
      template <typename T, typename Tuple>
      struct TupleContainsBase;

      /// \private This specialization implements TupleContainsBase. It only
      /// works if Tuple is a std::tuple; any other type for the second template
      /// argument will fail to compile.
      template <typename T, typename... Types>
      struct TupleContainsBase<T, std::tuple<Types...>>
          : std::integral_constant<bool,
              !std::is_same<
                std::tuple<typename std::conditional<
                  std::is_base_of<T, Types>::value, Empty, Types>::type...>,
                std::tuple<Types...>
              >::value> { };

      /////////////////////////////////////////////////
      template <typename ToFeatureTuple, typename FromFeatureList>
      struct HasAllFeaturesImpl;

      template <typename FromFeatureList,
                typename ToFeature1, typename... RemainingFeatures>
      struct HasAllFeaturesImpl<
          std::tuple<ToFeature1, RemainingFeatures...>, FromFeatureList>
      {
        static constexpr bool innerValue =
            FromFeatureList::template HasFeature<ToFeature1>();

        static constexpr bool value = innerValue
            && HasAllFeaturesImpl<std::tuple<RemainingFeatures...>,
                                  FromFeatureList>::value;

        static_assert(
            innerValue,
            "YOU CANNOT IMPLICITLY UPCAST TO THIS ENTITY TYPE, BECAUSE IT "
            "CONTAINS A FEATURE THAT IS NOT INCLUDED IN THE ENTITY THAT YOU "
            "ARE CASTING FROM.");
      };

      /////////////////////////////////////////////////
      template <typename FromFeatureList>
      struct HasAllFeaturesImpl<std::tuple<>, FromFeatureList>
      {
        static constexpr bool value = true;
      };

      /////////////////////////////////////////////////
      /// \brief The entity type "From" must contain all the features that are
      /// found within the enttiy type "To" in order for an upcast to be valid.
      template <typename To, typename From>
      struct HasAllFeatures
      {
        using ToFeatures = typename To::Features;
        using FromFeatures = typename From::Features;

        static constexpr bool value =
            HasAllFeaturesImpl<
                typename ToFeatures::Features,
                FromFeatures>::value;
      };

      /////////////////////////////////////////////////
      template <typename To, typename From, bool downcastable>
      struct CheckForDowncastableMessage
      {
        static constexpr bool value =
            TupleContainsBase<typename To::Identifier,
                              typename From::UpcastIdentifiers>::value;

        static_assert(
            value, "YOU CANNOT UPCAST TO AN INCOMPATIBLE ENTITY TYPE.");
      };

      // If an upcast from "To" to "From" is possible, then the user is trying
      // to do an implicit downcast (base type to more derived type) which is
      // not allowed. For that case, we will print a special message to help
      // them out.
      template <typename To, typename From>
      struct CheckForDowncastableMessage<To, From, true>
      {
        static constexpr bool value =
            TupleContainsBase<typename To::Identifier,
                              typename From::UpcastIdentifiers>::value;

        static_assert(
            value,
            "YOU CANNOT UPCAST TO AN ENTITY A MORE DERIVED TYPE. TO DOWNCAST "
            "TO A MORE DERIVED TYPE (e.g. Joint to RevoluteJoint), USE THE "
            "CASTING FEATURE OF THE BASE ENTITY, e.g. "
            "Joint::CastToRevoluteJoint()");
      };

      /////////////////////////////////////////////////
      template <typename To, typename From>
      struct UpcastCompatible
      {
        static_assert(
            std::is_same<typename From::Policy, typename To::Policy>::value,
            "YOU CAN ONLY CAST BETWEEN ENTITIES THAT HAVE THE SAME POLICY ("
            "e.g. FeaturePolicy3d, FeaturePolicy2d, FeaturePolicy3f). THE "
            "REQUESTED CONVERSION IS NOT ADMISSIBLE.");

        static_assert(HasAllFeatures<To, From>::value);

        static_assert(CheckForDowncastableMessage<To, From,
                      TupleContainsBase<
                        typename From::Identifier,
                        typename To::UpcastIdentifiers>::value>::value);

        static_assert(
            ignition::plugin::ConstCompatible<To, From>::value,
            "CANNOT CAST FROM A CONST-QUALIFIED ENTITY TO AN ENTITY WITHOUT A "
            "CONST-QUALIFIER.");
      };
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    EntityPtr<EntityT>::EntityPtr(std::nullptr_t)
      : entity(std::nullopt)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    EntityPtr<EntityT>::EntityPtr(std::nullopt_t)
      : entity(std::nullopt)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    auto EntityPtr<EntityT>::operator=(std::nullptr_t) -> EntityPtr&
    {
      this->entity.reset();
      return *this;
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    auto EntityPtr<EntityT>::operator=(std::nullopt_t) -> EntityPtr&
    {
      this->entity.reset();
      return *this;
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    template <typename Pimpl>
    EntityPtr<EntityT>::EntityPtr(
        const std::shared_ptr<Pimpl> &_pimpl,
        const Identity &_identity)
    {
      // EntityPtr should never accept invalid identities, otherwise users will
      // have to check for validity of their entities in two places instead of
      // one.
      if (_identity)
        this->entity = EntityT(_pimpl, _identity);
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    template <typename OtherEntityT>
    EntityPtr<EntityT>::EntityPtr(const EntityPtr<OtherEntityT> &_other)
    {
      *this = _other;
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    template <typename OtherEntityT>
    auto EntityPtr<EntityT>::operator=(const EntityPtr<OtherEntityT> &_other)
      -> EntityPtr&
    {
      // Verify that an upcast is okay for these types
      detail::UpcastCompatible<EntityT, OtherEntityT>();

      // If _other doesn't contain an entity clear our entity and return
      if (!_other.entity)
      {
        this->entity.reset();
        return *this;
      }

      // If _other.entity doesn't have a valid identity, we don't want to accept
      // it. EntityPtr should never accept invalid identities, otherwise users
      // will have to check for validity of their entities in two places instead
      // of one.
      if (!_other.entity->identity)
      {
        this->entity.reset();
        return *this;
      }

      if (this->entity)
      {
        // Avoid reallocating the pimpl
        *this->entity->pimpl = *_other.entity->pimpl;
        this->entity->identity = _other.entity->identity;
      }
      else
      {
        std::shared_ptr<typename EntityT::Pimpl> newPimpl =
            std::make_shared<typename EntityT::Pimpl>(
              *_other.entity->pimpl);

        this->entity = EntityT(std::move(newPimpl), _other.entity->identity);
      }

      return *this;
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    EntityT * EntityPtr<EntityT>::operator->() const
    {
      return &(this->entity.value());
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    EntityT & EntityPtr<EntityT>::operator*() const
    {
      return this->entity.value();
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    bool EntityPtr<EntityT>::Valid() const
    {
      return this->entity.has_value();
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    EntityPtr<EntityT>::operator bool() const
    {
      return this->entity.has_value();
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    std::size_t EntityPtr<EntityT>::Hash() const
    {
      if (!(*this))
        return std::hash<std::size_t>()(INVALID_ENTITY_ID);

      return std::hash<std::size_t>()(this->entity->EntityID());
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    std::size_t Entity<Policy, Features>::EntityID() const
    {
      return this->identity.id;
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    const std::shared_ptr<const void> &
    Entity<Policy, Features>::EntityReference() const
    {
      return this->identity.ref;
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    Entity<Policy, Features>::Entity(
        const std::shared_ptr<Pimpl> &_pimpl,
        const Identity &_identity)
      : pimpl(_pimpl),
        identity(_identity)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    template <typename FeatureT>
    typename FeatureT::template Implementation<Policy>*
    Entity<Policy, Features>::Interface()
    {
      return (*this->pimpl)->template QueryInterface<
          typename FeatureT::template Implementation<Policy>>();
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    template <typename FeatureT>
    const typename FeatureT::template Implementation<Policy>*
    Entity<Policy, Features>::Interface() const
    {
      return (*this->pimpl)->template QueryInterface<
          typename FeatureT::template Implementation<Policy>>();
    }

    /////////////////////////////////////////////////
    #define DETAIL_IGN_PHYSICS_ENTITY_PTR_IMPLEMENT_OPERATOR(op) \
      template <typename EntityT> \
      template <typename OtherEntityT> \
      bool EntityPtr<EntityT>::operator op (\
        const EntityPtr<OtherEntityT> &_other) const \
      { \
        /* If either ptr is invalid, we always return false */ \
        if (!(*this) || !_other) \
          return false; \
        return (this->entity->EntityID() op _other.entity->EntityID()); \
      }

    DETAIL_IGN_PHYSICS_ENTITY_PTR_IMPLEMENT_OPERATOR( == ) // NOLINT
    DETAIL_IGN_PHYSICS_ENTITY_PTR_IMPLEMENT_OPERATOR( < ) // NOLINT
    DETAIL_IGN_PHYSICS_ENTITY_PTR_IMPLEMENT_OPERATOR( > ) // NOLINT
    DETAIL_IGN_PHYSICS_ENTITY_PTR_IMPLEMENT_OPERATOR( != ) // NOLINT
    DETAIL_IGN_PHYSICS_ENTITY_PTR_IMPLEMENT_OPERATOR( <= ) // NOLINT
    DETAIL_IGN_PHYSICS_ENTITY_PTR_IMPLEMENT_OPERATOR( >= ) // NOLINT

    /////////////////////////////////////////////////
    // Operators to compare with nullptr and nullopt
    template <typename EntityT>
    bool operator==(std::nullptr_t, const EntityPtr<EntityT> &_ptr)
    {
      return !_ptr.Valid();
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    bool operator==(const EntityPtr<EntityT> &_ptr, std::nullptr_t)
    {
      return !_ptr.Valid();
    }

    /////////////////////////////////////////////////
    // Operators to compare with nullptr and nullopt
    template <typename EntityT>
    bool operator!=(std::nullptr_t, const EntityPtr<EntityT> &_ptr)
    {
      return !(_ptr == nullptr);
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    bool operator!=(const EntityPtr<EntityT> &_ptr, std::nullptr_t)
    {
      return !(_ptr == nullptr);
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    bool operator==(std::nullopt_t, const EntityPtr<EntityT> &_ptr)
    {
      return !_ptr.Valid();
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    bool operator==(const EntityPtr<EntityT> &_ptr, std::nullopt_t)
    {
      return !_ptr.Valid();
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    bool operator!=(std::nullopt_t, const EntityPtr<EntityT> &_ptr)
    {
      return !(_ptr == nullptr);
    }

    /////////////////////////////////////////////////
    template <typename EntityT>
    bool operator!=(const EntityPtr<EntityT> &_ptr, std::nullopt_t)
    {
      return !(_ptr == nullptr);
    }
  }
}

// Note that opening up namespace std is legal here because we are specializing
// a templated structure from the STL, which is permitted (and even encouraged).
namespace std
{
  /// \brief Template specialization that provides a hash function for EntityPtr
  /// so that it can easily be used in STL objects like std::unordered_set and
  /// std::unordered_map
  template <typename EntityT>
  struct hash<ignition::physics::EntityPtr<EntityT>>
  {
    size_t operator()(const ignition::physics::EntityPtr<EntityT> &ptr) const
    {
      return ptr.Hash();
    }
  };
}

#endif
