\page setupphysicsenginetpe "Advanced tutorial: Implement physics engine as a plugin"

## Prerequisites

In the previous tutorial \ref installation "Installation", you have installed
the Ignition Physics corresponding to the desired Ignition release. Note that
the recommended Ignition release is Dome.

## How to adapt a physics engine as a plugin in Ignition Physics

In the last \ref createphysicsplugin "Implement a physics plugin" tutorial, we
know how to implement a dummy physics engine as a plugin and load it using
\ref ignition::physics "Ignition Physics API". In
\ref createcustomfeature "Implement a custom feature" tutorial, we know how to
define and implement a custom feature in an existing
[DART](https://github.com/ignitionrobotics/ign-physics/tree/master/dartsim) physics
plugin. Now we will approach from the start of how to adapt any physics engine
into Ignition Physics as a physics plugin. We will use [TPE](https://github.com/ignitionrobotics/ign-physics/tree/main/tpe)
as an example for this tutorial.

### General structure of a physics plugin

As described in \ref createcustomfeature "Implement a custom feature" tutorial,
the plugin folder is placed just below the top-level folder of `ign-physics`.
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
already installed. In our case, [TPE](https://github.com/ignitionrobotics/ign-physics/tree/main/tpe)
is natively developed inside Ignition Physics and hence the `lib` folder.

We declare and implement the \ref ignition::physics::FeatureList "FeatureList"
interfacing with the physics engine API inside `plugin\src` folder
(please see \ref createcustomfeature "Implement a custom feature"
for the plugin feature requirements). Depending on design target, a \ref ignition::physics::FeatureList "FeatureList"
is generally a packing of related \ref ignition::physics::Feature "Features".
For example in TPE's [EntityManagementFeatures](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/src/EntityManagementFeatures.hh)
, there are \ref ignition::physics::GetEngineInfo "GetEngineInfo",
\ref ignition::physics::GetWorldFromEngine "GetWorldFromEngine", etc. features
defined in the "FeatureList" structure for entity management purpose.

Conventionally, a \ref ignition::physics::FeatureList "FeatureList" can be
implemented as:
- `<FEATURES>.hh` for the "FeatureList" declaration.
- `<FEATURES>.cc` for the "FeatureList" implementation corresponding to each of
the \ref ignition::physics::Feature "Features" member functions, using the
physics engine API to realize the feature behavior. For a list of common
pre-defined features in Ignition Physics, please refer to
\ref physicsplugin "Understanding the Physics Plugin" tutorial.
- `<FEATURES_TEST>.cc` for unit tests of the "FeatureList".

### Implement a feature using TPE's API

It is important to implement `EntityManagementFeatures` \ref ignition::physics::FeatureList "FeatureList"
for any physics plugin, in order to manage simulation objects. Hence, we will
show an example implementing \ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
in `EntityManagementFeatures` for TPE.

Basically, \ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
has a subclass \ref ignition::physics::Feature::Engine "Engine" defining
`ConstructEmptyWorld` member function. This feature will construct an empty
simulation world from using the physics engine API. We will display the feature
implementation here for convenience:
- [EntityManagementFeatures.hh](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/src/EntityManagementFeatures.hh):

```cpp
#include <string>

#include <ignition/physics/ConstructEmpty.hh>
#include <ignition/physics/Shape.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/RemoveEntities.hh>
#include <ignition/physics/Implements.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace tpeplugin {

struct EntityManagementFeatureList : FeatureList<
  GetEngineInfo,
  GetWorldFromEngine,
  GetModelFromWorld,
  GetLinkFromModel,
  GetShapeFromLink,
  RemoveEntities,
  ConstructEmptyWorldFeature,
  ConstructEmptyModelFeature,
  ConstructEmptyLinkFeature,
  CollisionFilterMaskFeature
> { };

class EntityManagementFeatures :
  public virtual Base,
  public virtual Implements3d<EntityManagementFeatureList>
{
  ... // other features' member functions

  // ----- Construct empty entities -----
  public: Identity ConstructEmptyWorld(
    const Identity &_engineID, const std::string &_name) override;

  ... // other features' member functions
};

}
}
}
```

Together with other \ref ignition::physics::Feature "Features", the \ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
is included in `EntityManagementFeatureList` "FeatureList" to declare the related
features for entity management purpose.

The `EntityManagementFeatures` "FeatureList" here inherits from:
- [Base](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/src/Base.hh)
class for foundation metadata definitions of Models, Joints, Links, and Shapes objects
of TPE to provide easy access to [tpelib](https://github.com/ignitionrobotics/ign-physics/tree/main/tpe/lib)
structures in the TPE library.
- \ref ignition::physics::Implements3d "Implements3d" for implementing the
custom feature with \ref ignition::physics::FeaturePolicy3d "FeaturePolicy3d"
("FeaturePolicy" of 3 dimensions and scalar type `double`).

Based on each \ref ignition::physics::Feature "Features" class definition, we
will override the corresponding functions of all the class defined in
`EntityManagementFeatureList` (the feature definitions can be found [here](https://ignitionrobotics.org/api/physics/3.0/classes.html)).
Here we show the overriding of `ConstructEmptyWorld` member function of \ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature".

- [EntityManagementFeatures.cc](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/src/EntityManagementFeatures.cc):

```cpp
#include <string>

#include "EntityManagementFeatures.hh"

using namespace ignition;
using namespace physics;
using namespace tpeplugin;

... // other function implementations

/////////////////////////////////////////////////
Identity EntityManagementFeatures::ConstructEmptyWorld(
  const Identity &, const std::string &_name)
{
  auto world = std::make_shared<tpelib::World>();
  world->SetName(_name);
  return this->AddWorld(world);
}

... // other function implementations
```

To realize the \ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
on the TPE physics engine, we implement `ConstructEmptyWorld` method. We simply
instantiate \ref ignition::physics::tpelib::World "World" object and then
wrap it with `std::share_ptr`. We then set the corresponding input world name
and store the current live world object.

Please take a look at [EntityManagement_TEST.cc](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/src/EntityManagement_TEST.cc)
as an example unit test for this `EntityManagementFeatures` "FeatureList".

### Main plugin.cc file

After the required \ref ignition::physics::FeatureList "FeatureList" are implemented,
we will include all the "FeatureLists" in [plugin.cc](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/src/plugin.cc)
main file and register the TPE physics plugin as follow:

```cpp
#include <ignition/physics/Register.hh>

#include "Base.hh"

#include "CustomFeatures.hh"
#include "EntityManagementFeatures.hh"
#include "FreeGroupFeatures.hh"
#include "KinematicsFeatures.hh"
#include "SDFFeatures.hh"
#include "ShapeFeatures.hh"
#include "SimulationFeatures.hh"

namespace ignition {
namespace physics {
namespace tpeplugin {

struct TpePluginFeatures : FeatureList<
  CustomFeatureList,
  EntityManagementFeatureList,
  FreeGroupFeatureList,
  KinematicsFeatureList,
  SDFFeatureList,
  ShapeFeatureList,
  SimulationFeatureList
> { };

class Plugin :
  public virtual Implements3d<TpePluginFeatures>,
  public virtual Base,
  public virtual CustomFeatures,
  public virtual EntityManagementFeatures,
  public virtual FreeGroupFeatures,
  public virtual KinematicsFeatures,
  public virtual SDFFeatures,
  public virtual ShapeFeatures,
  public virtual SimulationFeatures { };

IGN_PHYSICS_ADD_PLUGIN(Plugin, FeaturePolicy3d, TpePluginFeatures)

}
}
}
```

In general, there are 3 steps for `plugin.cc`:
- Define the final \ref ignition::physics::FeatureList "FeatureList" including
all required "FeatureLists". In TPE case, it is `TpePluginFeatures`.
- Define an empty class inherited all "FeatureLists" class, [Base](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/src/Base.hh)
class and \ref ignition::physics::Implements "Implements" class implementing
\ref ignition::physics::FeaturePolicy "FeaturePolicy" 2D or 3D and different
scalar type.
- Register the physics plugin using `IGN_PHYSICS_ADD_PLUGIN` macro (See
  \ref createphysicsplugin "Implement a physics plugin" for more detail).

An example of `CMakeLists.txt` for building the TPE plugin can be found [here](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/CMakeLists.txt).

Now we can load the new physics plugin to test it on Ignition Gazebo by
following this \ref switchphysicsengines "Switching physics engines" tutorial.
