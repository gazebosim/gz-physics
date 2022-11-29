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

#ifndef GZ_PHYSICS_TEST_MOCKCENTEROFMASS_HH_
#define GZ_PHYSICS_TEST_MOCKCENTEROFMASS_HH_

#include <gz/physics/FeatureList.hh>
#include <Eigen/Geometry>

namespace mock
{
  /////////////////////////////////////////////////
  // TODO(MXG): Offer an ignition::physics::Vector class that accepts a
  // FeaturePolicy type.
  template <typename FeaturePolicyT>
  using Vector = Eigen::Matrix<
      typename FeaturePolicyT::Scalar, FeaturePolicyT::Dim, 1>;


  /////////////////////////////////////////////////
  struct MockLinkCenterOfMass : public ignition::physics::Feature
  {
    using Identity = ignition::physics::Identity;

    template <typename PolicyT, typename FeaturesT>
    class Link : public virtual Feature::Link<PolicyT, FeaturesT>
    {
      /// \brief Get the center of mass of this Link
      public: Vector<PolicyT> CenterOfMass() const;
    };

    template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: virtual Vector<PolicyT> GetLinkCenterOfMass(
          const Identity &_id) const = 0;
    };
  };

  /////////////////////////////////////////////////
  struct MockModelCenterOfMass : public ignition::physics::Feature
  {
    using Identity = ignition::physics::Identity;

    template <typename PolicyT, typename FeaturesT>
    class Model : public virtual Feature::Model<PolicyT, FeaturesT>
    {
      /// \brief Get the center of mass of this Model
      public: Vector<PolicyT> CenterOfMass() const;
    };

    template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: virtual Vector<PolicyT> GetModelCenterOfMass(
          const Identity &_id) const = 0;
    };
  };

  using MockCenterOfMass = ignition::physics::FeatureList<
      MockLinkCenterOfMass,
      MockModelCenterOfMass
  >;

  // ---------------------- Implementations ----------------------

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  Vector<PolicyT> MockLinkCenterOfMass::Link<PolicyT, FeaturesT>::
  CenterOfMass() const
  {
    return this->template Interface<MockLinkCenterOfMass>()->
        GetLinkCenterOfMass(this->identity);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  Vector<PolicyT> MockModelCenterOfMass::Model<PolicyT, FeaturesT>::
  CenterOfMass() const
  {
    return this->template Interface<MockModelCenterOfMass>()->
        GetModelCenterOfMass(this->identity);
  }
}

#endif
