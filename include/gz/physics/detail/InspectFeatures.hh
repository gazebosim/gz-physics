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

#include <set>
#include <string>
#include <tuple>

#include <ignition/plugin/PluginPtr.hh>

namespace gz
{
  namespace physics
  {
    namespace detail
    {
      /////////////////////////////////////////////////
      /// \private This class is used to inspect what features are provided by
      /// a plugin. It implements the API of RequestEngine.
      template <typename PolicyT, typename FeatureT, typename = void_t<> >
      struct InspectFeatures
      {
        using Interface = typename FeatureT::template Implementation<PolicyT>;

        /// \brief Check that each feature is provided by the plugin.
        template <typename PtrT>
        static bool Verify(const PtrT &_pimpl)
        {
          return _pimpl && _pimpl->template HasInterface<Interface>();
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
        }

        template <typename PtrT>
        static void MissingNames(const PtrT &_pimpl,
                                 std::set<std::string> &_names)
        {
          if (!_pimpl || !_pimpl->template HasInterface<Interface>())
            _names.insert(typeid(FeatureT).name());
        }
      };

      template <typename PolicyT>
      struct InspectFeatures<PolicyT, void, void_t<> >
      {
        template <typename PtrT>
        static bool Verify(const PtrT &/*_pimpl*/)
        {
          // This is just the terminal leaf of the inspection tree, so it must
          // not falsify the verification.
          return true;
        }

        template <typename LoaderT, typename ContainerT>
        static void EraseIfMissing(
            const LoaderT &/*_loader*/,
            ContainerT &/*_plugins*/)
        {
          // Do nothing, this is a terminal leaf
        }

        template <typename PtrT>
        static void MissingNames(const PtrT &/*_pimpl*/,
                                 std::set<std::string> &/*_names*/)
        {
          // Do nothing, this is a terminal leaf
        }
      };

      /// \private Implementation of InspectFeatures.
      template <typename PolicyT, typename FeatureListT>
      struct InspectFeatures<PolicyT, FeatureListT,
          void_t<typename FeatureListT::CurrentTupleEntry>>
      {
        using Branch1 = InspectFeatures<PolicyT,
            typename FeatureListT::CurrentTupleEntry>;
        using Branch2 = InspectFeatures<PolicyT,
            typename GetNext<FeatureListT>::n>;

        /// \brief Check that each feature is provided by the plugin.
        template <typename PtrT>
        static bool Verify(const PtrT &_pimpl)
        {
          return Branch1::Verify(_pimpl) && Branch2::Verify(_pimpl);
        }

        template <typename LoaderT, typename ContainerT>
        static void EraseIfMissing(
            const LoaderT &_loader,
            ContainerT &_plugins)
        {
          Branch1::EraseIfMissing(_loader, _plugins);
          Branch2::EraseIfMissing(_loader, _plugins);
        }

        template <typename PtrT>
        static void MissingNames(const PtrT &_pimpl,
                                 std::set<std::string> &_names)
        {
          Branch1::MissingNames(_pimpl, _names);
          Branch2::MissingNames(_pimpl, _names);
        }
      };
    }
  }
}

#endif  // IGNITION_PHYSICS_DETAIL_INSPECTFEATURES_HH
