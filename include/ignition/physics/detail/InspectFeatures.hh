/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DETAIL_INSPECTFEATURES_HH
#define IGNITION_PHYSICS_DETAIL_INSPECTFEATURES_HH

#include <ignition/plugin/PluginPtr.hh>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      /////////////////////////////////////////////////
      /// \private This class is used to inspect what features are provided by
      /// a plugin. It implements the API of RequestEngine.
      template <typename Policy, typename InterfaceTuple>
      struct InspectFeatures;

      /// \private Implementation of InspectFeatures.
      template <typename PolicyT, typename Feature1, typename... Remaining>
      struct InspectFeatures<PolicyT, std::tuple<Feature1, Remaining...>>
      {
        using Interface = typename Feature1::template Implementation<PolicyT>;

        /// \brief Check that each feature is provided by the plugin.
        template <typename PtrT>
        static bool Verify(const PtrT &_pimpl)
        {
          // TODO(MXG): Consider replacing with a fold expression
          return _pimpl && _pimpl->template HasInterface<Interface>()
              && InspectFeatures<PolicyT, std::tuple<Remaining...>>::
                      Verify(_pimpl);
        }

        template <typename LoaderT, typename ContainerT>
        static void EraseIfMissing(
            const LoaderT &_loader,
            ContainerT &_plugins)
        {
          const auto acceptable =
              _loader.template PluginsImplementing<Interface>();

          std::set<std::string> unacceptable;
          for (const std::string &p : _plugins)
          {
            const auto it = std::find(acceptable.begin(), acceptable.end(), p);
            if (it == acceptable.end())
              unacceptable.insert(unacceptable.end(), p);
          }

          for (const std::string &u : unacceptable)
            _plugins.erase(u);

          InspectFeatures<PolicyT, std::tuple<Remaining...>>::EraseIfMissing(
                _loader, _plugins);
        }

        template <typename PtrT>
        static void MissingNames(const PtrT &_pimpl,
                                 std::set<std::string> &_names)
        {
          if (!_pimpl || !_pimpl->template HasInterface<Interface>())
            _names.insert(typeid(Feature1).name());

          InspectFeatures<PolicyT, std::tuple<Remaining...>>::MissingNames(
                _pimpl, _names);
        }
      };

      template <typename PolicyT>
      struct InspectFeatures<PolicyT, std::tuple<>>
      {
        template <typename PtrT>
        static bool Verify(const PtrT&)
        {
          return true;
        }

        template <typename LoaderT, typename ContainerT>
        static void EraseIfMissing(const LoaderT &, ContainerT &)
        {
          // Do nothing
        }

        template <typename PtrT>
        static void MissingNames(const PtrT&, std::set<std::string>&)
        {
          // Do nothing
        }
      };
    }
  }
}

#endif // IGNITION_PHYSICS_DETAIL_INSPECTFEATURES_HH
