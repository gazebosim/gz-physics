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

#ifndef IGNITION_PHYSICS_BASICOBJECT_HH_
#define IGNITION_PHYSICS_BASICOBJECT_HH_

#include <memory>
#include <limits>

#include <ignition/physics/Export.hh>
#include <ignition/physics/Feature.hh>

namespace ignition
{
  namespace physics
  {
    // Forward declaration
    class Feature;

    /// \brief This is the base class of all "proxy objects". The "proxy
    /// objects" are essentially interfaces into the actual objects which exist
    /// inside of the various physics engine implementations. The proxy objects
    /// contain the minimal amount of data (e.g. a unique identifier,
    /// a reference-counter for the engine object, and a reference to the engine
    /// interface that it needs) necessary to interface with the object inside
    /// of the engine that it refers to.
    ///
    /// Examples of proxy objects are the Link class, Joint class, and Model
    /// class.
    template <typename FeatureType>
    class IGNITION_PHYSICS_VISIBLE BasicObject
    {
      public: using Engine = typename Feature::Engine<FeatureType>;

      /// \brief Get the unique object ID of this Link
      public: std::size_t ObjectID() const;

      /// \brief Get a reference-counting std::shared_ptr to the object inside
      /// the engine that this object provides an abstraction for.
      protected: std::shared_ptr<const void> ObjectReference() const;

      /// \brief Get a reference to the engine that this object belongs to.
      /// Object features (a.k.a. classes that inherit the BasicObject type)
      /// will use dynamic_cast on this reference to obtain a reference to the
      /// engine feature that it needs in order to function.
      protected: Engine *EngineReference();

      /// \brief Const-qualified version of EngineReference
      protected: const Engine *EngineReference() const;

      /// \brief Constructor for the BasicObject.
      ///
      /// Notes for developers:
      /// - We provide a default constructor for this class so that the object
      /// feature classes (which get virtually inherited) do not each need to
      /// call on the constructor of BasicObject. That would make it difficult
      /// to implement and maintain all the constructors of the different object
      /// feature classes.
      /// - Since all the features are virtually inherited, only the "final"
      /// inheriting class constructor needs to actually call this constructor.
      /// - The default argument for the ID is the highest possible integer.
      /// This should help to make it clear if the construction procedure is not
      /// working as intended. If _features is a nullptr, that would also
      /// indicate that the construction procedure is not working as intended.
      protected: BasicObject(
        Engine *const _engine = nullptr,
        const std::size_t _id = std::numeric_limits<std::size_t>::max(),
        const std::shared_ptr<const void> &_ref = nullptr);

      // Forward declaration of class implementation
      protected: class Implementation;

      /// \brief PIMPL pointer to the implementation
      protected: std::unique_ptr<Implementation> pimpl;

      /// \brief Virtual destructor
      public: virtual ~BasicObject() = default;
    };
  }
}

#include <ignition/physics/detail/BasicObject.hh>

#endif
