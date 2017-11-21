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
#include <ignition/physics/BasicObject.hh>
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
    class IGNITION_PHYSICS_VISIBLE FrameSemanticsBase : public virtual Feature
    {
      // Forward declaration
      public: template <typename> class Object;

      /// \brief This class defines the engine interface that provides the
      /// FrameSemantics feature.
      public: template <typename FeatureType>
      class Engine : public virtual Feature::Engine<FeatureType>
      {
        public: using Scalar = typename FeatureType::Scalar;
        public: enum { Dim = FeatureType::Dim };
        public: using FrameData = ignition::physics::FrameData<Scalar, Dim>;

        /// \brief Get the current 3D transformation of the specified frame with
        /// respect to the WorldFrame.
        ///
        /// Engine developers only need to provide an implementation for this
        /// function in order to provide FrameSemantics.
        public: virtual FrameData FrameDataRelativeToWorld(
          const FrameID &_id) const = 0;

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

        /// \brief Classes that derive from FrameSemantics can use this function
        /// to spawn a FrameID.
        ///
        /// Note that an _id of 0 is always interpreted as the World Frame, so
        /// you should never spawn a FrameID with an _id of 0 unless you intend
        /// for it to represent the world frame.
        protected: FrameID SpawnFrameID(
            const std::size_t _id,
            const std::shared_ptr<const void> &_ref) const;

        template <typename> friend class FrameSemanticsBase::Object;
      };

      public: template <typename FeatureType>
      class Object : public virtual BasicObject<FeatureType>
      {
        public: using Scalar = typename FeatureType::Scalar;
        public: enum { Dim = FeatureType::Dim };
        public: using FrameData = ignition::physics::FrameData<Scalar, Dim>;

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

        /// \brief The constructor will use its base class to find a reference
        /// to the engine feature interface that it needs.
        public: Object();

        /// \brief Virtual destructor
        public: virtual ~Object() = default;

        /// \brief Pointer to the FrameSemantics::Engine interface that allows
        /// this feature to work.
        private: const FrameSemanticsBase::Engine<FeatureType> *const engine;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature will apply frame semantics to Link objects.
    class IGNITION_PHYSICS_VISIBLE LinkFrameSemantics
        : public virtual FrameSemanticsBase
    {
      public: template <typename FeatureType>
      class Engine : public virtual FrameSemanticsBase::Engine<FeatureType> { };

      public: template <typename FeatureType>
      class Link : public virtual FrameSemanticsBase::Object<FeatureType> { };
    };

    /////////////////////////////////////////////////
    /// \brief This feature will apply frame semantics to Joint objects.
    class IGNITION_PHYSICS_VISIBLE JointFrameSemantics
        : public virtual FrameSemanticsBase
    {
      public: template <typename FeatureType>
      class Engine : public virtual FrameSemanticsBase::Engine<FeatureType> { };

      public: template <typename FeatureType>
      class Joint : public virtual FrameSemanticsBase::Object<FeatureType> { };
    };

    /////////////////////////////////////////////////
    /// \brief This feature will apply frame semantics to Model objects.
    class IGNITION_PHYSICS_VISIBLE ModelFrameSemantics
        : public virtual FrameSemanticsBase
    {
      public: template <typename FeatureType>
      class Engine : public virtual FrameSemanticsBase::Engine<FeatureType> { };

      public: template <typename FeatureType>
      class Model : public virtual FrameSemanticsBase::Object<FeatureType> { };
    };

    /////////////////////////////////////////////////
    /// \brief This feature will apply frame semantics to all objects.
    class IGNITION_PHYSICS_VISIBLE FrameSemantics
        : public virtual LinkFrameSemantics,
          public virtual JointFrameSemantics,
          public virtual ModelFrameSemantics
    {
      public: template <typename FeatureType>
      class Engine : public virtual FrameSemanticsBase::Engine<FeatureType> { };
    };
  }
}

#include <ignition/physics/detail/FrameSemantics.hh>

#endif
