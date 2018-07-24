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

#ifndef IGNITION_PHYSICS_FRAMESEMANTICS_HH_
#define IGNITION_PHYSICS_FRAMESEMANTICS_HH_

#include <memory>

#include <ignition/physics/Feature.hh>
#include <ignition/physics/Entity.hh>
#include <ignition/physics/FrameID.hh>
#include <ignition/physics/FrameData.hh>
#include <ignition/physics/FramedQuantity.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief FrameSemantics is an Interface that can be provided by
    /// ignition-physics engines to provide users with easy ways to express
    /// kinematic quantities in terms of frames and compute their values in
    /// terms of arbitrary frames of reference.
    class IGNITION_PHYSICS_VISIBLE FrameSemantics : public virtual Feature
    {
      // Forward declaration
      public: template <typename, typename> class Frame;

      /// \brief This class defines the engine interface that provides the
      /// FrameSemantics feature.
      public: template <typename PolicyT, typename FeaturesT>
      class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
      {
        public: using FrameData =
            ignition::physics::FrameData<
              typename PolicyT::Scalar, PolicyT::Dim>;

        /// \brief Resolve is able to take a FramedQuantity (FQ) and compute its
        /// values in terms of other reference frames. The argument `relativeTo`
        /// indicates a frame that the quantity should be compared against (e.g.
        /// the velocity of Frame A relative to Frame B where both A and B may
        /// be moving). The argument `inCoordinatesOf` indicates the coordinate
        /// frame that the values should be expressed in (this is usually just a
        /// change in rotation).
        public: template <typename FQ>
        typename FQ::Quantity Resolve(
          const FQ &_quantity,
          const FrameID _relativeTo,
          const FrameID _inCoordinatesOf) const;

        /// \brief This overload causes the World Frame to be used as the
        /// default frame when relativeTo is not specified. It also causes the
        /// frame specified for relativeTo to be used as the frame for
        /// inCoordinatesOf.
        ///
        /// In other words:
        ///
        /// -- Get the value of v in terms of the World Frame
        ///      Resolve(v)
        ///
        /// -- Get the value of v relative to frame A, in coordinates of frame A
        ///      Resolve(v, A)
        ///
        /// -- Get the value of v relative to frame A, in coordinates of frame B
        ///      Resolve(v, A, B)
        ///
        public: template <typename FQ>
        typename FQ::Quantity Resolve(
          const FQ &_quantity,
          const FrameID _relativeTo = FrameID::World()) const;

        /// \brief Create a new FramedQuantity which expresses the input
        /// quantity in terms of a new parent frame. Note that the returned
        /// FramedQuantity will behave as though it has a constant value within
        /// its new parent frame.
        public: template <typename FQ>
        FQ Reframe(const FQ &_quantity,
                   const FrameID _withRespectTo = FrameID::World()) const;
      };

      /// \brief Base class for the API of a Frame. This will be inherited by
      /// any objects that are able to express Frame Semantics.
      public: template <typename PolicyT, typename FeaturesT>
      class Frame : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using FrameData =
          ignition::physics::FrameData<typename PolicyT::Scalar, PolicyT::Dim>;

        /// \brief Get a FrameID for this object
        public: FrameID GetFrameID() const;

        /// \brief Get the FrameData of this object with respect to another
        /// frame. The data will also be expressed in the coordinates of the
        /// _relativeTo frame.
        public: FrameData FrameDataRelativeTo(
          const FrameID &_relativeTo) const;

        /// \brief Get the FrameData of this object relative to another frame,
        /// expressed in the coordinates of a third frame.
        public: FrameData FrameDataRelativeTo(
          const FrameID &_relativeTo,
          const FrameID &_inCoordinatesOf) const;

        /// \brief Implicit conversion to a FrameID is provided. This way, a
        /// reference to the Object can be treated as a FrameID.
        public: operator FrameID() const;

        /// \brief Virtual destructor
        public: virtual ~Frame() = default;
      };

      /// \brief This class is inherited by physics plugin classes that want to
      /// provide this feature.
      template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using FrameData =
          ignition::physics::FrameData<typename PolicyT::Scalar, PolicyT::Dim>;

        /// \brief Get the current 3D transformation of the specified frame with
        /// respect to the WorldFrame.
        ///
        /// Engine developers only need to provide an implementation for this
        /// function in order to provide FrameSemantics.
        public: virtual FrameData FrameDataRelativeToWorld(
          const FrameID &_id) const = 0;

        /// \brief Physics engines can use this function to generate a FrameID
        /// using an existing Identity.
        ///
        /// This function is part of a design that ensures that FrameID objects
        /// can only be instantiated by "authoritative" sources, like a physics
        /// engine.
        ///
        /// \param[in] _identity
        ///   The underlying identity of the frame object.
        /// \return A FrameID object that corresponds to _identity.
        protected: virtual FrameID GenerateFrameID(
            const Identity &_identity) const;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature will apply frame semantics to Link objects.
    class IGNITION_PHYSICS_VISIBLE LinkFrameSemantics
        : public virtual FrameSemantics
    {
      public: template <typename Policy, typename Features>
      class Engine : public virtual FrameSemantics::Engine<Policy, Features>{ };

      public: template <typename Policy, typename Features>
      class Link : public virtual FrameSemantics::Frame<Policy, Features> { };
    };

    /////////////////////////////////////////////////
    /// \brief This feature will apply frame semantics to Joint objects.
    class IGNITION_PHYSICS_VISIBLE JointFrameSemantics
        : public virtual FrameSemantics
    {
      public: template <typename Policy, typename Features>
      class Engine : public virtual FrameSemantics::Engine<Policy, Features>{ };

      public: template <typename Policy, typename Features>
      class Joint : public virtual FrameSemantics::Frame<Policy, Features>{ };
    };

    /////////////////////////////////////////////////
    /// \brief This feature will apply frame semantics to Model objects.
    class IGNITION_PHYSICS_VISIBLE ModelFrameSemantics
        : public virtual FrameSemantics
    {
      public: template <typename Policy, typename Features>
      class Engine : public virtual FrameSemantics::Engine<Policy, Features>{ };

      public: template <typename Policy, typename Features>
      class Model : public virtual FrameSemantics::Frame<Policy, Features>{ };
    };

    /////////////////////////////////////////////////
    /// \brief This feature will apply frame semantics to all objects.
    class IGNITION_PHYSICS_VISIBLE CompleteFrameSemantics
        : public virtual LinkFrameSemantics,
          public virtual JointFrameSemantics,
          public virtual ModelFrameSemantics
    {
      public: template <typename Policy, typename Features>
      class Engine : public virtual FrameSemantics::Engine<Policy, Features>{ };
    };
  }
}

#include <ignition/physics/detail/FrameSemantics.hh>

#endif
