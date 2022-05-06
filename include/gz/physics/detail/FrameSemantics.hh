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

#include <memory>

#include <ignition/physics/FrameSemantics.hh>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      template <typename PolicyT, typename RQ>
      static typename RQ::Quantity Resolve(
          const FrameSemantics::Implementation<PolicyT> &_impl,
          const RQ &_quantity,
          const FrameID &_relativeTo,
          const FrameID &_inCoordinatesOf)
      {
        using Quantity = typename RQ::Quantity;
        using Space = typename RQ::Space;
        using FrameDataType = typename Space::FrameDataType;
        using RotationType = typename Space::RotationType;

        const FrameID &parentFrameID = _quantity.ParentFrame();

        Quantity q;
        RotationType currentCoordinates;

        if (parentFrameID == _relativeTo)
        {
          // The quantity is already expressed relative to the _relativeTo frame

          if (_relativeTo.ID() == _inCoordinatesOf.ID())
          {
            // The quantity is already expressed in coordinates of the
            // _inCoordinatesOf frame
            return _quantity.RelativeToParent();
          }

          q = _quantity.RelativeToParent();
          if (_relativeTo.IsWorld())
          {
            // The World Frame has all zero fields
            currentCoordinates = RotationType::Identity();
          }
          else
          {
            currentCoordinates = _impl.FrameDataRelativeToWorld(
                  _relativeTo).pose.linear();
          }
        }
        else
        {
          // We should only ask for the FrameData if the parent frame is not the
          // world frame.
          const FrameDataType parentFrameData = parentFrameID.IsWorld() ?
                FrameDataType() : _impl.FrameDataRelativeToWorld(parentFrameID);

          if (_relativeTo.IsWorld())
          {
            // Resolving quantities to the world frame requires fewer operations
            // than resolving to an arbitrary frame, so we use a special
            // function for that.
            q = Space::ResolveToWorldFrame(
                  _quantity.RelativeToParent(),
                  parentFrameData);

            // The World Frame has all zero fields
            currentCoordinates = RotationType::Identity();
          }
          else
          {
            const FrameDataType relativeToData =
                _impl.FrameDataRelativeToWorld(_relativeTo);

            q = Space::ResolveToTargetFrame(
                  _quantity.RelativeToParent(),
                  parentFrameData,
                  relativeToData);

            currentCoordinates = relativeToData.pose.linear();
          }
        }

        if (_relativeTo != _inCoordinatesOf)
        {
          if (_inCoordinatesOf.IsWorld())
          {
            // Resolving quantities to the world coordinates requires fewer
            // operations than resolving to an arbitrary frame.
            return Space::ResolveToWorldCoordinates(q, currentCoordinates);
          }
          else
          {
            const RotationType inCoordinatesOfRotation =
                _impl.FrameDataRelativeToWorld(_inCoordinatesOf).pose.linear();

            return Space::ResolveToTargetCoordinates(
                  q, currentCoordinates, inCoordinatesOfRotation);
          }
        }

        return q;
      }
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    template <typename RQ>
    typename RQ::Quantity FrameSemantics::Engine<PolicyT, FeaturesT>::Resolve(
        const RQ &_quantity,
        const FrameID &_relativeTo,
        const FrameID &_inCoordinatesOf) const
    {
      return detail::Resolve<PolicyT>(
            *this->template Interface<FrameSemantics>(),
            _quantity, _relativeTo, _inCoordinatesOf);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    template <typename RQ>
    typename RQ::Quantity FrameSemantics::Engine<PolicyT, FeaturesT>::Resolve(
        const RQ &_quantity, const FrameID &_relativeTo) const
    {
      return this->Resolve(_quantity, _relativeTo, _relativeTo);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    template <typename RQ>
    RQ FrameSemantics::Engine<PolicyT, FeaturesT>::Reframe(
        const RQ &_quantity, const FrameID &_withRespectTo) const
    {
      return RQ(_withRespectTo,
                this->Resolve(_quantity, _withRespectTo, _withRespectTo));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    FrameID FrameSemantics::Frame<PolicyT, FeaturesT>::GetFrameID() const
    {
      return FrameID(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto FrameSemantics::Frame<PolicyT, FeaturesT>::
    FrameDataRelativeToWorld() const -> FrameData
    {
      return this->template Interface<FrameSemantics>()
                 ->FrameDataRelativeToWorld(FrameID(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto FrameSemantics::Frame<PolicyT, FeaturesT>::FrameDataRelativeTo(
        const FrameID &_relativeTo) const -> FrameData
    {
      return this->FrameDataRelativeTo(_relativeTo, _relativeTo);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto FrameSemantics::Frame<PolicyT, FeaturesT>::FrameDataRelativeTo(
        const FrameID &_relativeTo,
        const FrameID &_inCoordinatesOf) const -> FrameData
    {
      using RelativeFrameData =
          ignition::physics::RelativeFrameData<
            typename PolicyT::Scalar, PolicyT::Dim>;

      // Create a zeroed-out FrameData which is a child of this frame,
      // effectively making it an equivalent frame. Then, resolve the child in
      // terms of the input frames.
      //
      // The resulting FrameData will be equivalent to resolving this frame in
      // terms of the input frames.
      return detail::Resolve<PolicyT>(
            *this->template Interface<FrameSemantics>(),
            RelativeFrameData(this->GetFrameID()),
            _relativeTo, _inCoordinatesOf);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    FrameSemantics::Frame<PolicyT, FeaturesT>::operator FrameID() const
    {
      return this->GetFrameID();
    }

    /////////////////////////////////////////////////
    template <typename PolicyT>
    FrameID FrameSemantics::Implementation<PolicyT>::GenerateFrameID(
        const Identity &_identity) const
    {
      return FrameID(_identity);
    }
  }
}

#endif
