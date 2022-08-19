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

#ifndef GZ_PHYSICS_DETAIL_IDENTITY_HH_
#define GZ_PHYSICS_DETAIL_IDENTITY_HH_

#include <cstddef>
#include <memory>

#include <gz/physics/Export.hh>
#include <gz/utilities/SuppressWarning.hh>

namespace gz
{
  namespace physics
  {
    // Forward declare
    template <typename, typename> class Entity;
    class Identity;

    namespace detail
    {
      /////////////////////////////////////////////////
      /// \brief This base class is used by plugin implementations to generate
      /// identities for entities.
      class IGNITION_PHYSICS_VISIBLE Implementation
      {
        /// \brief An implementation class should call this function whenever it
        /// wants to generate an identity for an Entity.
        protected: Identity GenerateIdentity(
            std::size_t _id,
            const std::shared_ptr<void> &_ref = nullptr) const;

        protected: Identity GenerateInvalidId() const;

        /// \brief An implementation class can use this function to get the
        /// reference contained in the identity
        protected: const std::shared_ptr<void> &Reference(
            const Identity &_identity) const;

        /// \brief An implementation class can use this function to get the
        /// reference contained in the identity
        /// \tparam T The stored pointer is cast to this type.
        /// \return A raw pointer from the stored shared_ptr but cast to the
        /// provided type T
        protected: template<typename T>
        T *ReferenceInterface(const Identity &_identity) const
        {
          return static_cast<T *>(this->Reference(_identity).get());
        }
      };
    }

    /////////////////////////////////////////////////
    /// \brief This class contains fields that identify an Entity. We use this
    /// separate class in order to have tight control over how Entities can be
    /// instantiated; in particular, an Identity can only be created by a
    /// plugin implementation, so users cannot create invalid Entities.
    class IGNITION_PHYSICS_VISIBLE Identity
    {
      /// \brief Convert to true if this Identity refers to a valid entity (i.e.
      /// its id field is not INVALID_ENTITY_ID).
      public: operator bool() const;

      /// \brief Convert to the id value of this Identity.
      public: operator std::size_t() const;

      /// \brief This integer ID uniquely identifies the object that this
      /// entity is referring to. No two entities may use the same ID unless
      /// they are referring to the same instance of a physics engine object.
      ///
      /// The ID is not allowed to change at any point in the lifetime of the
      /// engine object.
      ///
      /// Note that the ID of 0 is reserved for the "engine" object.
      public: std::size_t id;

      /// \brief This is an optional reference-counting field for the proxy
      /// objects. Not all engines are required to support this field for all
      /// types, so this may be left as a nullptr.
      ///
      /// This reference is not allowed to change at any point in the lifetime
      /// of the engine object.
      IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      public: const std::shared_ptr<void> ref;
      IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING

      /// \brief This is used by Entity so that it can default-construct. This
      /// should never actually be called.
      private: Identity();

      /// \brief This is called by Feature::Implementation
      private: Identity(
          std::size_t _id,
          const std::shared_ptr<void> &_ref);

      // These friends are the only classes allowed to create an identity
      template <typename, typename> friend class ::gz::physics::Entity;
      friend class ::gz::physics::detail::Implementation;
    };
  }
}

#endif
