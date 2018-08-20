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

#ifndef IGNITION_PHYSICS_ENTITY_HH_
#define IGNITION_PHYSICS_ENTITY_HH_

#include <limits>
#include <memory>
#include <optional>

#include <ignition/physics/Export.hh>
#include <ignition/physics/detail/Identity.hh>

namespace ignition
{
  namespace physics
  {
    // Forward declaration
    namespace detail { template <typename, typename> struct DeterminePlugin; }

    /// \brief This constant-value should be used to indicate that an Entity ID
    /// is invalid (i.e. does not refer to a real entity).
    const std::size_t INVALID_ENTITY_ID =
        std::numeric_limits<std::size_t>::max();

    template <typename EntityT>
    class EntityPtr
    {
      // All the automatic constructors and assignment operators are okay.
      public: EntityPtr() = default;
      public: EntityPtr(const EntityPtr&) = default;
      public: EntityPtr(EntityPtr&&) = default;
      public: EntityPtr &operator=(const EntityPtr&) = default;
      public: EntityPtr &operator=(EntityPtr&&) = default;
      public: ~EntityPtr() = default;

      /// \brief Create an EntityPtr that points to an invalid Entity
      public: EntityPtr(std::nullptr_t);

      /// \brief Create an EntityPtr that points to an invalid Entity
      public: EntityPtr(std::nullopt_t);

      /// \brief Assign this to point to an invalid Entity.
      public: EntityPtr &operator=(std::nullptr_t);

      /// \brief Assign this to point to an invalid Entity.
      public: EntityPtr &operator=(std::nullopt_t);

      /// \brief Create an EntityPtr that points to the Entity tied to _identity
      /// \param[in] _pimpl
      ///   Pointer to the implementation for this entity
      /// \param[in] _identity
      ///   The identity of the Entity that this should point to
      public: template <typename Pimpl>
      EntityPtr(const std::shared_ptr<Pimpl> &_pimpl,
                const Identity &_identity);

      /// \brief Create a new reference to another entity
      /// \param[in] _other
      ///   Another entity reference with a compatible type and compatible set
      ///   of features
      public: template <typename OtherEntityT>
      EntityPtr(const EntityPtr<OtherEntityT> &_other);

      /// \brief Assign this to point at another compatible Entity
      /// \param[in] _other
      ///   Another entity reference with a compatible type and compatible set
      ///   of features.
      public: template <typename OtherEntityT>
      EntityPtr &operator=(const EntityPtr<OtherEntityT> &_other);

      /// \brief Drill operator. Access members of the Entity being pointed to.
      /// This does NOT check whether the Entity is valid before trying to use
      /// it. If the validity of the Entity is in doubt, check Valid() or simply
      /// operator bool() before attempting to use this operator.
      /// \return The ability to call a member function on the underlying Entity
      public: EntityT * operator->() const;

      /// \brief Dereference operator. Access a reference to the Entity being
      /// pointed to. This does NOT check whether the Entity is valid before
      /// trying to provide it. If the validity of the Entity is in doubt, check
      /// IsEmpty() or simply operator bool() before attempting to use this
      /// operator.
      /// \return A reference to the underlying entity
      public: EntityT & operator*() const;

      /// \brief Check whether this is pointing at a valid Entity.
      /// \return True if this is pointing to a valid Entity, otherwise false.
      public: bool Valid() const;

      /// \brief Implicitly cast this EntityPtr to a boolean.
      /// \return True if this is pointing to a valid Entity, otherwise false.
      public: operator bool() const;

      // TODO(MXG): Write comparison operators and a Hash() implementation.

      /// \brief If we are pointing to a valid entity, it will be stored here.
      /// Otherwise, this is a nullopt.
      ///
      /// We make this mutable so that we get logical const-correctness. We are
      /// not concerned with physical const-correctness here.
      ///
      /// When passing the type into the std::optional, we remove the
      /// const-qualifier because otherwise a ConstEntityPtr object cannot be
      /// modified to point to a different entity, as the assignment operator
      /// of std::optional gets deleted when it is given a const-qualified type.
      /// The dereference operations * and -> will still use the original
      /// const-qualifications of EntityT, so logical constness is still
      /// preserved, because the user cannot access an const-unqualified
      /// reference to EntityT.
      private: mutable std::optional<std::remove_const_t<EntityT>> entity;


      // Declare this friendship so we can cast between different Entity types.
      template <typename> friend class EntityPtr;
    };

    /// \brief This is the base class of all "proxy objects". The "proxy
    /// objects" are essentially interfaces into the actual objects which exist
    /// inside of the various physics engine implementations. The proxy objects
    /// contain the minimal amount of data (e.g. a unique identifier,
    /// a reference-counter for the implementation object, and a reference to
    /// the implementation interface that it needs) necessary to interface with
    /// the object inside of the implementation that it refers to.
    ///
    /// Examples of entities are the Link class, Joint class, and Model
    /// class.
    template <typename PolicyT, typename FeaturesT>
    class Entity
    {
      public: using Policy = PolicyT;
      public: using Features = FeaturesT;
      public: using Pimpl =
          typename detail::DeterminePlugin<Policy, Features>::type;

      /// \brief Get the unique ID value of this Entity.
      public: std::size_t EntityID() const;

      /// \brief Get a reference-counting std::shared_ptr to the object inside
      /// the implementation that this object provides an abstraction for.
      public: const std::shared_ptr<const void> &EntityReference() const;

      /// \brief Constructor for the Entity.
      ///
      /// Notes for developers:
      /// - We provide a default constructor for this class so that the feature
      /// entity classes (which get virtually inherited) do not each need to
      /// call on the constructor of Entity. That would make it difficult
      /// to implement and maintain all the constructors of the different object
      /// feature classes.
      /// - Since all the features are virtually inherited, only the "final"
      /// inheriting class constructor needs to actually call this constructor.
      /// - The default argument for the identity will have an INVALID_ENTITY_ID
      /// value (which is the result of default-constructing Identity). If the
      /// Identity of an Entity is invalid, that implies that there is a bug in
      /// the construction of that Entity.
      protected: Entity(
        const std::shared_ptr<Pimpl> &_pimpl = nullptr,
        const Identity &_identity = Identity());

      /// \brief Get a pointer to the implementation of FeatureT.
      ///
      /// This is a convenience function so that entities don't have to query
      /// for interfaces on their pimpl object.
      ///
      /// \return A pointer to the implementation of FeatureT, or a nullptr if
      /// FeatureT is not available.
      protected: template <typename FeatureT>
      typename FeatureT::template Implementation<Policy> *Interface();

      /// \brief Same as Interface(), but const-qualified so that const entities
      /// can request const-qualified interfaces from the implementation.
      /// \return A pointer to a const-qualified implementation of F, or a
      /// nullptr if F is not available.
      protected: template <typename F>
      const typename F::template Implementation<Policy> *Interface() const;

      /// \brief This is a pointer to the physics engine implementation, and it
      /// can be used by the object features to find the interfaces that they
      /// need in order to function.
      protected: std::shared_ptr<Pimpl> pimpl;

      /// \brief This field contains information to identify the entity.
      protected: Identity identity;

      /// \brief Virtual destructor
      public: virtual ~Entity() = default;

      // Allow EntityPtr to cast between EntityTypes
      template<typename> friend class EntityPtr;
    };
  }
}

#include <ignition/physics/detail/Entity.hh>

#endif
