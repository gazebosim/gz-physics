\page usecustomengine "Use custom engine with Ignition Physics"

## Prerequisites

- \ref installation "Installation"
- \ref physicsplugin "Understand physics plugin"
- \ref physicsengine "Use different physics engines"
- \ref loadplugin "Load physics plugin"
- \ref implementcustomfeature "Implement custom feature"

## How to interface with physics engine

In the previous
\ref implementcustomfeature "Implement custom feature" tutorial, we walked through how to
define and implement a custom feature using an already supported physics
engine. This tutorial will explain step-by-step how to interface with any physics engine
using Ignition Physics. We will use [TPE](https://github.com/ignitionrobotics/ign-physics/tree/main/tpe) as an example in this tutorial.

### Structure of a physics plugin

Depending on what physics engine you would like to use,
the folder structure could be slightly different from what's shown below.
Here's the plugin folder structure of TPE, within the Ignition Physics library.

```
ign-physics
├── tpe
│   ├── plugin                           Implementation of the plugin features interfacing the physics engines API
│   │    ├── src
│   │    |     ├── plugin.cc             Main file for the plugin declaration and plugin registering.
│   │    |     ├── <FEATURES>.hh         The FeatureList header file.
│   │    |     ├── <FEATURES>.cc         Implementation of the FeatureList using physics engine API
│   │    |     └── <FEATURES_TEST>.cc    Tests
│   │    ├── CMakeLists.txt              CMake build script for the plugin features.
│   └── CMakeLists.txt                   CMake build script for the plugin.
└── CMakeList.txt                        CMake build script for Ignition Physics library.
```

We link the external physics engine library
in `CMakeLists.txt` of the plugin, assuming the physics engine library is
already installed. In our case, [TPE](https://github.com/ignitionrobotics/ign-physics/tree/main/tpe)
is placed inside Ignition Physics and hence there is a `lib` folder under `tpe`.

We declare and implement the \ref ignition::physics::FeatureList "FeatureList"
interfacing with the physics engine API inside `plugin\src` folder
(please see \ref implementcustomfeature "Implement custom feature"
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
\ref physicsplugin "Understand physics plugin" tutorial.
- `<FEATURES_TEST>.cc` for unit tests of the "FeatureList".

Next, we will use a simplified TPE plugin example to explain important components needed to interface with any physics engine. All code examples used below can be found in [examples](https://github.com/ignitionrobotics/ign-physics/tree/ign-physics2/examples) under the `simple_tpe_plugin` folder.

### `plugin.cc`

In this tutorial, we will show how to construct a simple simulation world using
[TPE](https://github.com/ignitionrobotics/ign-physics/tree/main/tpe) physics
engine. For this purpose, we will implement the pre-defined
\ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
and include this feature into an empty \ref ignition::physics::FeatureList "FeatureList"
named `EntityManagementFeatureList` defined in `EntityManagementFeatures.hh`.
We first include the `EntityManagementFeatureList` in `plugin.cc` main file
and register the example TPE physics plugin as follow:

\snippet examples/simple_tpe_plugin/plugin.cc

Those are 3 things needed to be specified in `plugin.cc`:
- Define the conclusive \ref ignition::physics::FeatureList "FeatureList" including
all required "FeatureLists" and `Base` class. In TPE case, it is `TpePluginFeatures`.
- Define the dimension of the simulation, ex. \ref ignition::physics::Implements "Implements" class implementing
\ref ignition::physics::FeaturePolicy "FeaturePolicy" 2D or 3D and different
scalar type.
- Register the physics plugin using `IGN_PHYSICS_ADD_PLUGIN` macro (See \ref implementphysicsplugin "Implement physics plugin" for more detail).

### Implement features with physics engine's API

Now we would like to implement the `EntityManagementFeatures`.
In the `plugin` folder, we will create two files `EntityManagementFeatures.hh` and
`EntityManagementFeatures.cc` to implement \ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
in `EntityManagementFeatures` "FeatureList". We will use the provided
[CMakeLists.txt](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/CMakeLists.txt)
and [Base.hh](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/src/Base.hh)
in  `tpe/plugin` folder.

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-physics/main/tpe/plugin/CMakeLists.txt -P <path-to-ign-physics>/tpe/plugin/
wget https://raw.githubusercontent.com/ignitionrobotics/ign-physics/main/tpe/plugin/src/Base.hh -P <path-to-ign-physics>/tpe/plugin/src
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

As an example, we will focus on implementing \ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature" shown below:

\snippet examples/simple_tpe_plugin/EntityManagementFeatures.hh

Together with other (if existing) \ref ignition::physics::Feature "Features",
the \ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
is included in `EntityManagementFeatureList` "FeatureList" to declare the related
features for entity management purpose.

The `EntityManagementFeatures` "FeatureList" here inherits from:
- (optionally) \ref ignition::physics::tpelib::Base "Base"
class for foundation metadata definitions of Models, Joints, Links, and Shapes objects
of TPE to provide easy access to [tpelib](https://github.com/ignitionrobotics/ign-physics/tree/main/tpe/lib)
structures in the TPE library. Note that we mention `Base` class here for
completeness, `Base` class is not necessarily needed if there is a straightforward
way to interface external physics engine class objects with `ign-physics` class objects.
- \ref ignition::physics::Implements3d "Implements3d" for implementing the
custom feature with \ref ignition::physics::FeaturePolicy3d "FeaturePolicy3d"
("FeaturePolicy" of 3 dimensions and scalar type `double`).

Then we can go ahead with the implementation of `ConstructEmptyWorldFeature`:

\snippet examples/simple_tpe_plugin/EntityManagementFeatures.cc

Here we show the overriding of `ConstructEmptyWorld` member function of
\ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature",
this is where we use the physics engine API to implement this member function.
We simply instantiate \ref ignition::physics::tpelib::World "World" object, set
the world name and call \ref ignition::physics::tpelib::Base::AddWorld "AddWorld"
function which was defined in [Base.hh](https://github.com/ignitionrobotics/ign-physics/blob/main/tpe/plugin/src/Base.hh).

Simple unit tests are good practice for sanity checks.
While we won't go into detail, here is an example to test our new
\ref ignition::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature":

\snippet examples/simple_tpe_plugin/EntityManagementFeatures_TEST.cc

To get a more comprehensive view of how `EntityManagementFeatures` are constructed in TPE and Dartsim,
feel free to take a look here:
- Dartsim: [EntityManagementFeatures.hh](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics2/dartsim/src/EntityManagementFeatures.hh) and [EntityManagementFeatures.cc](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics2/dartsim/src/EntityManagementFeatures.cc)
- TPE: [EntityManagementFeatures.hh](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics2/tpe/plugin/src/EntityManagementFeatures.hh) and [EntityManagementFeatures.cc](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics2/tpe/plugin/src/EntityManagementFeatures.cc)

## Load and test

Please follow the previous tutorial \ref installation "Installation" to build
`ign-physics` from source again for our new feature to be compiled.

Now we can load the new physics plugin named `ignition-physics-tpe-plugin`
to test it on Ignition Gazebo by following this \ref physicsengine "Use different physics engines" tutorial.
