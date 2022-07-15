\page setupphysicsenginetpe "Use custom engine with Gazebo Physics"

## Prerequisites

In the previous tutorial \ref installation "Installation", you have installed
the Gazebo Physics corresponding to the desired Gazebo release.

## How to adapt a physics engine as a plugin in Gazebo Physics

In the last \ref createphysicsplugin "Implement a physics plugin" tutorial, we
know how to implement a dummy physics engine as a plugin and load it using
\ref gz::physics "Gazebo Physics API". In
\ref createcustomfeature "Implement a custom feature" tutorial, we know how to
define and implement a custom feature in an existing
[DART](https://github.com/gazebosim/gz-physics/tree/main/dartsim) physics
plugin. This tutorial will explain step-by-step how to use any physics engine
with Gazebo Physics. We will use [TPE](https://github.com/gazebosim/gz-physics/tree/main/tpe)
as an example for this tutorial.

### General structure of a physics plugin

As described in \ref createcustomfeature "Implement a custom feature" tutorial,
the plugin folder is placed just below the top-level folder of `gz-physics`.
In general, any physics plugin folder will have the following structure, which
is commented in detail:

```
tpe
├── lib                              Main physics engine implementation.
│    ├── src
│    ├── CMakeLists.txt              CMake build script for physics engine implementation.
├── plugin                           Main implementation of the plugin features interfacing the physics engines API
│    ├── src
│    |     ├── plugin.cc             Main file for the plugin declaration and plugin registering.
│    |     ├── <FEATURES>.hh         The FeatureList header file.
│    |     ├── <FEATURES>.cc         Implementation of the FeatureList, which adapts to functions implemented in lib used in the FeatureList.
│    |     ├── <FEATURES_TEST>.cc    Test cases for the FeatureList.
│    ├── worlds                      Example SDF files for testing plugin functionalities.
│    ├── CMakeLists.txt              CMake build script for the plugin features.
└── CMakeLists.txt                   CMake build script for the plugin.
```

Note that the `lib` folder is optionally placed inside the physics plugin folder
depending on the design target, we can link the external physics engine library
in `CMakeLists.txt` of the plugin, assuming the physics engine library is
already installed. In our case, [TPE](https://github.com/gazebosim/gz-physics/tree/main/tpe)
is natively developed inside Gazebo Physics and hence the `lib` folder.

We declare and implement the \ref gz::physics::FeatureList "FeatureList"
interfacing with the physics engine API inside `plugin\src` folder
(please see \ref createcustomfeature "Implement a custom feature"
for the plugin feature requirements). Depending on design target, a \ref gz::physics::FeatureList "FeatureList"
is generally a packing of related \ref gz::physics::Feature "Features".
For example in TPE's [EntityManagementFeatures](https://github.com/gazebosim/gz-physics/blob/main/tpe/plugin/src/EntityManagementFeatures.hh)
, there are \ref gz::physics::GetEngineInfo "GetEngineInfo",
\ref gz::physics::GetWorldFromEngine "GetWorldFromEngine", etc. features
defined in the "FeatureList" structure for entity management purpose.

Conventionally, a \ref gz::physics::FeatureList "FeatureList" can be
implemented as:
- `<FEATURES>.hh` for the "FeatureList" declaration.
- `<FEATURES>.cc` for the "FeatureList" implementation corresponding to each of
the \ref gz::physics::Feature "Features" member functions, using the
physics engine API to realize the feature behavior. For a list of common
pre-defined features in Gazebo Physics, please refer to
\ref physicsplugin "Understanding the Physics Plugin" tutorial.
- `<FEATURES_TEST>.cc` for unit tests of the "FeatureList".

### Main plugin.cc file

In this tutorial, we will show how to construct a simple simulation world using
[TPE](https://github.com/gazebosim/gz-physics/tree/main/tpe) physics
engine. For this purpose, we will implement the pre-defined
\ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
and include this feature into an empty \ref gz::physics::FeatureList "FeatureList"
named `EntityManagementFeatureList` defined in `EntityManagementFeatures.hh`.
We first include the `EntityManagementFeatureList` in `plugin.cc` main file
and register the example TPE physics plugin as follow:

```cpp
#include <gz/physics/Register.hh>

#include "Base.hh"

#include "EntityManagementFeatures.hh"

namespace gz {
namespace physics {
namespace tpeplugin {

struct TpePluginFeatures : FeatureList<
  EntityManagementFeatureList
> { };

class Plugin :
  public virtual Implements3d<TpePluginFeatures>,
  public virtual Base,
  public virtual EntityManagementFeatures { };

GZ_PHYSICS_ADD_PLUGIN(Plugin, FeaturePolicy3d, TpePluginFeatures)

}
}
}
```

In general, there are 3 steps for `plugin.cc`:
- Define the final \ref gz::physics::FeatureList "FeatureList" including
all required "FeatureLists". In TPE case, it is `TpePluginFeatures`.
- Define an empty class inherited all "FeatureLists" class, [Base](https://github.com/gazebosim/gz-physics/blob/main/tpe/plugin/src/Base.hh)
class (optionally depending on software design)
and \ref gz::physics::Implements "Implements" class implementing
\ref gz::physics::FeaturePolicy "FeaturePolicy" 2D or 3D and different
scalar type.
- Register the physics plugin using `GZ_PHYSICS_ADD_PLUGIN` macro (See
  \ref createphysicsplugin "Implement a physics plugin" for more detail).

### Implement a feature using TPE's API

Now we assume that we have not implemented any \ref gz::physics::Feature "Feature"
for `TPE`. In the `plugin` folder, we will create two files `EntityManagementFeatures.hh` and
`EntityManagementFeatures.cc` to implement \ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
in `EntityManagementFeatures` "FeatureList". The unit test for this feature is also
worth implemented in `EntityManagement_TEST.cc`. Please download the example
[CMakeLists.txt](https://github.com/gazebosim/gz-physics/blob/main/tpe/plugin/CMakeLists.txt)
and [Base.hh](https://github.com/gazebosim/gz-physics/blob/main/tpe/plugin/src/Base.hh)
into `plugin` folder by:

```bash
wget https://raw.githubusercontent.com/gazebosim/gz-physics/main/tpe/plugin/CMakeLists.txt -P <path-to-gz-physics>/tpe/plugin/
wget https://raw.githubusercontent.com/gazebosim/gz-physics/main/tpe/plugin/src/Base.hh -P <path-to-gz-physics>/tpe/plugin/src
```

Now the folder structure looks like this:

```
tpe
├── lib
├── plugin
│    ├── src
│    |     ├── plugin.cc
│    |     ├── Base.hh
│    |     ├── EntityManagementFeatures.hh
│    |     ├── EntityManagementFeatures.cc
│    |     ├── EntityManagement_TEST.cc
│    ├── CMakeLists.txt
└── CMakeLists.txt
```

Basically, \ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
has a subclass \ref gz::physics::Feature::Engine "Engine" defining
`ConstructEmptyWorld` member function. The feature implementation is shown as
below:

##### EntityManagementFeatures.hh:

```cpp
#ifndef GZ_PHYSICS_TPE_PLUGIN_SRC_GETENTITIESFEATURE_HH_
#define GZ_PHYSICS_TPE_PLUGIN_SRC_GETENTITIESFEATURE_HH_

#include <string>
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/Implements.hh>

#include "Base.hh" // optionally depending on software design

namespace gz {
namespace physics {
namespace tpeplugin {

struct EntityManagementFeatureList : FeatureList<
  ConstructEmptyWorldFeature
> { };

class EntityManagementFeatures :
  public virtual Base,
  public virtual Implements3d<EntityManagementFeatureList>
{
  // ----- Construct empty entities -----
  public: Identity ConstructEmptyWorld(
    const Identity &_engineID, const std::string &_name) override;
};

}
}
}

#endif
```

Together with other (if existing) \ref gz::physics::Feature "Features",
the \ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
is included in `EntityManagementFeatureList` "FeatureList" to declare the related
features for entity management purpose.

The `EntityManagementFeatures` "FeatureList" here inherits from:
- (optionally) \ref gz::physics::tpelib::Base "Base"
class for foundation metadata definitions of Models, Joints, Links, and Shapes objects
of TPE to provide easy access to [tpelib](https://github.com/gazebosim/gz-physics/tree/main/tpe/lib)
structures in the TPE library. Note that we mention `Base` class here for
completeness, `Base` class is not necessarily needed if there is a straightforward
way to interface external physics engine class objects with `gz-physics` class objects.
- \ref gz::physics::Implements3d "Implements3d" for implementing the
custom feature with \ref gz::physics::FeaturePolicy3d "FeaturePolicy3d"
("FeaturePolicy" of 3 dimensions and scalar type `double`).

##### EntityManagementFeatures.cc:

```cpp
#include <string>
#include "EntityManagementFeatures.hh"

using namespace gz;
using namespace physics;
using namespace tpeplugin;

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
  const Identity &, const std::string &_name)
{
  auto world = std::make_shared<tpelib::World>();
  world->SetName(_name);
  return this->AddWorld(world);
}
```

Here we show the overriding of `ConstructEmptyWorld` member function of
\ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature",
this is where we use the physics engine API to implement this member function.
We simply instantiate \ref gz::physics::tpelib::World "World" object, set
the world name and call \ref gz::physics::tpelib::Base::AddWorld "AddWorld"
function which was defined in [Base.hh](https://github.com/gazebosim/gz-physics/blob/main/tpe/plugin/src/Base.hh).

##### EntityManagement_TEST.cc:
Simple unit tests are good practice for sanity checks.
While we won't go into detail, here is an example to test our new
\ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature":

```cpp
#include <gtest/gtest.h>
#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>
#include "EntityManagementFeatures.hh"

struct TestFeatureList : gz::physics::FeatureList<
  gz::physics::tpeplugin::EntityManagementFeatureList
> { };

TEST(EntityManagement_TEST, ConstructEmptyWorld)
{
  gz::plugin::Loader loader;
  loader.LoadLib(tpe_plugin_LIB);

  gz::plugin::PluginPtr tpe_plugin =
    loader.Instantiate("gz::physics::tpeplugin::Plugin");

  auto engine =
      gz::physics::RequestEngine3d<TestFeatureList>::From(tpe_plugin);
  auto world = engine->ConstructEmptyWorld("empty world");
  ASSERT_NE(nullptr, world);
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

Please take a look at [EntityManagement_TEST.cc](https://github.com/gazebosim/gz-physics/blob/main/tpe/plugin/src/EntityManagement_TEST.cc)
for more comprehensive unit tests.

## Build the custom physics plugin

Please follow the previous tutorial \ref installation "Installation" to build
`gz-physics` from source again for our new feature to be compiled.

Now we can load the new physics plugin named `gz-physics-tpe-plugin`
to test it on Gazebo by following this \ref switchphysicsengines "Switching physics engines" tutorial.
