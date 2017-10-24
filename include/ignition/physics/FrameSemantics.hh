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

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>

namespace igntiion
{
  namespace physics
  {
    /// \brief Container for specifying Frame IDs. We do not want to use a generic
    /// integer type for this, because it may lead to bugs where a plain integer
    /// is mistaken for a FrameID. This also allows the compiler to always
    /// perform argument deduction successfully.
    struct FrameID
    {
      public: inline explicit FrameID(const std::size_t _id)
        : id(_id)
      {
        // Do nothing
      }

      public: std::size_t id;
    };

    /// \brief The ID of the World Frame is always uniquely 0.
    const FrameID WorldID = FrameID(0);

    /// \brief The FramedQuantity class is a wrapper for the native ign-math
    /// classes. The purpose of this wrapper is to endow the native classes with
    /// frame semantics, so that they can express the frame of reference of
    /// their values.
    template <typename Base, std::size_t Dim, typename ConfigSpace>
    class FramedQuantity : public Base
    {
      /// \brief This constructor will specify the parent frame and then forward
      /// the remaining arguments to the constructor of the underlying quantity.
      public: template <typename... Args>
      FramedQuantity(const FrameID &_parentID, Args&&... args)
        : Base(std::forward<Args>(args)...),
          parentFrame(_parentID)
      {
        // Do nothing
      }

      /// \brief This constructor will forward all of the arguments to the
      /// constructor of the underlying quantity, and then set the parent frame
      /// to be the World Frame.
      public: template <typename... Args>
      FramedQuantity(Args&&... args)
        : Base(std::forward<Args>(args)...),
          parentFrame(WorldID)
      {
        // Do nothing
      }

      /// \brief This variable specifies the parent frame that this
      /// FramedQuantity belongs to.
      public: FrameID parentFrame;

      /// \brief The underlying raw type of the quantity that is being expressed.
      public: using Quantity = Base;

      /// \brief The mathematical space which defines how this quantity is
      /// transformed between reference frames.
      public: using Space = ConfigSpace;
    };

    /// \brief FrameSemantics is an Interface that can be provided by
    /// ignition-physics engines to provide users with easy ways to express
    /// kinematic quantities in terms of frames and compute their values in
    /// terms of arbitrary frames of reference.
    class FrameSemantics
    {
      /// \brief Resolve is able to take a FramedQuantity (FQ) and compute its
      /// values in terms of other reference frames. The argument `relativeTo`
      /// indicates a frame that the quantity should be compared against (e.g.
      /// the velocity of Frame A relative to Frame B where both A and B may be
      /// moving). The argument `inCoordinatesOf` indicates the coordinate frame
      /// that the values should be expressed in (this is usually just a
      /// rotation).
      ///
      /// Since `relativeTo` and `inCoordinatesOf` may be different, this
      /// function only outputs the raw quantity without expressing its frame of
      /// reference.
      public: template <typename FQ>
      typename FQ::Quantity Resolve(
        const FQ &quantity,
        const FrameID relativeTo,
        const FrameID inCoordinatesOf);

      /// \brief This version of Resolve is able to produce a FramedQuantity
      /// which is expressed in the requested reference frame. The parentFrame
      /// of the FramedQuantity will match the frame specified by the argument
      /// `withRespectTo`. The return value can implicitly decay to a raw
      /// quantity if desired.
      public: template <typename FQ>
      FQ Resolve(const FQ &quantity, const FrameID withRespectTo = WorldID);

      /// \brief Get the current 3D transformation of the specified frame.
      public: virtual ignition::math::Pose3d FrameTransform(
          const FrameID& _id) const = 0;
    };
  }
}

#endif // IGNITION_PHYSICS_FRAMESEMANTICS_HH_
