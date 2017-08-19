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

#ifndef IGNITION_PHYSICS_SPECIFYDATA_HH_
#define IGNITION_PHYSICS_SPECIFYDATA_HH_

namespace ignition
{
  namespace physics
  {
    /// \brief The SpecifyData class allows you to form combinations of data
    /// specifications. In other words, you can freely mix invokations to
    /// RequireData and ExpectData. Example usage:
    ///
    ///     using namespace ignition::physics;
    ///
    ///     using MyInputSpecifications = SpecifyData<
    ///         RequireData<
    ///             DesiredPositionInput,
    ///             DesiredVelocityInput>,
    ///         ExpectData<
    ///             ExternalForceInput,
    ///             ProximitySensorInput,
    ///             ForceTorqueSensorInput> >;
    ///
    /// This would define a CompositeData which is required to contain a
    /// DesiredPositionInput data structure and a DesiredVelocityInput data
    /// structure. It is also optimized for ExternalForceInput,
    /// ProximitySensorInput, and ForceTorqueSensorInput data structures, but
    /// they are optional. Whether a data structure is specified as expected or
    /// required, you will be able to get extremely low-cost access to it
    /// through an object that has the type of MyInputSpecifications.
    ///
    /// Specifications can also be composed together. For example, if there is
    /// another specification like:
    ///
    ///     using ComplianceInputSpecifications = SpecifyData<
    ///         RequireData<
    ///             ProximitySensorInput,
    ///             ComplianceParameters>,
    ///         ExpectData<
    ///             ForceTorqueSensorInput,
    ///             CameraSensorInput> >;
    ///
    /// then you can combine these specifications:
    ///
    ///     using CombinedInputSpecifications = SpecifyData<
    ///         MyInputSpecifications,
    ///         ComplianceInputSpecifications>;
    ///
    /// Note that RequireData takes precedence over ExpectData, so
    /// ProximitySensorInput will be promoted to Required when the two
    /// specifications are combined.
    template <typename... Specifications>
    class SpecifyData
    {

    };

    template <typename... DataTypes>
    class RequireData
    {

    };

    template <typename... DataTypes>
    class ExpectData
    {

    };

  }
}

#endif
