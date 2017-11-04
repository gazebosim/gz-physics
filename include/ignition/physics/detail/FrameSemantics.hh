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

#ifndef IGNITION_PHYSICS_DETAIL_FRAMESEMANTICS_HH_
#define IGNITION_PHYSICS_DETAIL_FRAMESEMANTICS_HH_

#include <ignition/physics/FrameSemantics.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    template <typename FQ>
    typename FQ::Quantity FrameSemantics::Engine<_Scalar, _Dim>::Resolve(
        const FQ &_quantity,
        const FrameID _relativeTo,
        const FrameID _inCoordinatesOf) const
    {
      using Quantity = typename FQ::Quantity;
      using Space = typename FQ::Space;
      using FrameDataType = typename Space::FrameDataType;
      using RotationType = typename Space::RotationType;

      const FrameID parentFrameID = _quantity.ParentFrame();

      Quantity q;
      RotationType currentCoordinates;

      if (parentFrameID == _relativeTo)
      {
        // The quantity is already expressed relative to the _relativeTo frame

        if (_relativeTo.id == _inCoordinatesOf.id)
        {
          // The quantity is already expressed in coordinates of the
          // _inCoordinatesOf frame
          return _quantity.RelativeToParent();
        }

        q = _quantity.RelativeToParent();
        currentCoordinates = this->FrameDataRelativeToWorld(
              _relativeTo).pose.linear();
      }
      else
      {
        // We should only ask for the FrameData if the parent frame is not the
        // world frame.
        const FrameDataType parentFrameData =
            FrameID::World() == parentFrameID ?
              FrameDataType() : this->FrameDataRelativeToWorld(parentFrameID);

        if (FrameID::World() == _relativeTo)
        {
          // Resolving quantities to the world frame requires fewer operations
          // than resolving to an arbitrary frame, so we use a special function
          // for that.
          q = Space::ResolveToWorldFrame(
                _quantity.RelativeToParent(),
                parentFrameData);

          // The World Frame has all zero fields
          currentCoordinates = RotationType::Identity();
        }
        else
        {
          const FrameDataType relativeToData =
              this->FrameDataRelativeToWorld(_relativeTo);

          q = Space::ResolveToTargetFrame(
                _quantity.RelativeToParent(),
                parentFrameData,
                relativeToData);

          currentCoordinates = relativeToData.pose.linear();
        }
      }

      if (_relativeTo != _inCoordinatesOf)
      {
        if (FrameID::World().id == _inCoordinatesOf.id)
        {
          // Resolving quantities to the world coordinates requires fewer
          // operations than resolving to an arbitrary frame.
          return Space::ResolveToWorldCoordinates(q, currentCoordinates);
        }
        else
        {
          const RotationType inCoordinatesOfRotation =
              this->FrameDataRelativeToWorld(
                _inCoordinatesOf).pose.linear();

          return Space::ResolveToTargetCoordinates(
                q, currentCoordinates, inCoordinatesOfRotation);
        }
      }

      return q;
    }

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    template <typename FQ>
    typename FQ::Quantity FrameSemantics::Engine<_Scalar, _Dim>::Resolve(
        const FQ &_quantity, const FrameID _relativeTo) const
    {
      return this->Resolve(_quantity, _relativeTo, _relativeTo);
    }

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    template <typename FQ>
    FQ FrameSemantics::Engine<_Scalar, _Dim>::Reframe(
        const FQ &_quantity, const FrameID _withRespectTo) const
    {
      return FQ(_withRespectTo,
                this->Resolve(_quantity, _withRespectTo, _withRespectTo));
    }

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    FrameID FrameSemantics::Engine<_Scalar, _Dim>::SpawnFrameID(
        const std::size_t _id,
        const std::shared_ptr<const void> &_ref) const
    {
      return FrameID(_id, _ref);
    }

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    FrameID FrameSemantics::Object<_Scalar, _Dim>::GetFrameID() const
    {
      return this->engine->SpawnFrameID(
            this->BasicObject::ObjectID(),
            this->BasicObject::ObjectReference());
    }

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    auto FrameSemantics::Object<_Scalar, _Dim>::FrameDataRelativeTo(
        const FrameID &_relativeTo) const -> FrameData
    {
      return this->FrameDataRelativeTo(_relativeTo, _relativeTo);
    }

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    auto FrameSemantics::Object<_Scalar, _Dim>::FrameDataRelativeTo(
        const FrameID &_relativeTo,
        const FrameID &_inCoordinatesOf) const -> FrameData
    {
      // Create a zeroed-out FrameData which is a child of this frame,
      // effectively making it an equivalent frame. Then, resolve the child in
      // terms of the input frames.
      //
      // The resulting FrameData will be equivalent to resolving this frame in
      // terms of the input frames.
      return this->engine->Resolve(
            RelativeFrameData<_Scalar, _Dim>(this->GetFrameID()),
            _relativeTo, _inCoordinatesOf);
    }

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    FrameSemantics::Object<_Scalar, _Dim>::operator FrameID() const
    {
      return this->GetFrameID();
    }

    /////////////////////////////////////////////////
    template <typename _Scalar, std::size_t _Dim>
    FrameSemantics::Object<_Scalar, _Dim>::Object()
      : engine(dynamic_cast<FrameSemantics::Engine<_Scalar, _Dim> *const>(
                  this->BasicObject::EngineReference()))
    {
      assert(engine && "[FrameSemantics::Object] Could not find a reference "
                       "to the necessary engine feature. This should never "
                       "happen! Please report this bug!\n");
    }
  }
}

#endif
