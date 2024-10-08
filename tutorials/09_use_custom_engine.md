\page usecustomengine Use a custom engine with Gazebo Physics

## Prerequisites

- \ref installation "Installation"
- \ref physicsplugin "Understand physics plugin"
- \ref physicsengine "Use different physics engines"
- \ref pluginloading "Load physics plugin"
- \ref createcustomfeature "Implement custom feature"

## How to interface with physics engine

In the previous
\ref implementcustomfeature "Implement custom feature" tutorial, we walked through how to
define and implement a custom feature using an already supported physics
engine. This tutorial will explain step-by-step how to interface with any physics engine
using Gazebo Physics. We will use [TPE](https://github.com/gazebosim/gz-physics/tree/main/tpe) as an example in this tutorial.

### Structure of a physics plugin

Depending on what physics engine you would like to use,
the folder structure could be slightly different from what's shown below.
Here's the plugin folder structure of TPE, within the Gazebo Physics library.

```
gz-physics
├── tpe
│   ├── plugin                           Implementation of the plugin features interfacing the physics engines API
│   │    ├── src
│   │    |     ├── plugin.cc             Main file for the plugin declaration and plugin registering.
│   │    |     ├── <FEATURES>.hh         The FeatureList header file.
│   │    |     ├── <FEATURES>.cc         Implementation of the FeatureList using physics engine API
│   │    |     └── <FEATURES_TEST>.cc    Tests
│   │    └── CMakeLists.txt              CMake build script for the plugin features.
│   └── CMakeLists.txt                   CMake build script for the plugin.
└── CMakeList.txt                        CMake build script for Gazebo Physics library.
```

We link the external physics engine library
in `CMakeLists.txt` of the plugin, assuming the physics engine library is
already installed. In our case, [TPE](https://github.com/gazebosim/gz-physics/tree/main/tpe)
is placed inside Gazebo Physics and hence there is a `lib` folder under `tpe`.

We declare and implement the \ref gz::physics::FeatureList "FeatureList"
interfacing with the physics engine API inside `plugin/src` folder
(please see \ref implementcustomfeature "Implement custom feature"
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
\ref physicsplugin "Understand physics plugin" tutorial.
- `<FEATURES_TEST>.cc` for unit tests of the "FeatureList".

Next, we will use a simplified TPE plugin example to explain important components needed to interface with any physics engine. All code examples used below can be downloaded from [examples](https://github.com/gazebosim/gz-physics/tree/ign-physics2/examples) under the `simple_plugin` folder:

```
simple_plugin
├── CMakeLists.txt
├── plugin.cc
├── EntityManagementFeatures.hh
├── EntityManagementFeatures.cc
└── EntityManagementFeatures_TEST.cc
```

### plugin.cc

In this tutorial, we will show how to construct a simple simulation world using
[TPE](https://github.com/gazebosim/gz-physics/tree/main/tpe) physics
engine. For this purpose, we will implement the pre-defined
\ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
and include this feature into an empty \ref gz::physics::FeatureList "FeatureList"
named `EntityManagementFeatureList` defined in `EntityManagementFeatures.hh`.
We first include the `EntityManagementFeatureList` in `plugin.cc` main file
and register the example TPE physics plugin as follow:


\snippet examples/simple_plugin/plugin.cc code


Those are 3 things needed to be specified in `plugin.cc`:
- Define the conclusive \ref gz::physics::FeatureList "FeatureList" including
all required "FeatureLists" and `Base` class. In TPE case, it is `TpePluginFeatures`.
- Define the dimension of the simulation, ex. \ref gz::physics::Implements "Implements" class implementing
\ref gz::physics::FeaturePolicy "FeaturePolicy" 2D or 3D and different
scalar type.
- Register the physics plugin using `GZ_PHYSICS_ADD_PLUGIN` macro (See \ref implementphysicsplugin "Implement physics plugin" for more detail).

### Implement features with physics engine's API

Now we would like to implement the `EntityManagementFeatures`.
In the `simple_plugin` folder, we will create two files `EntityManagementFeatures.hh` and
`EntityManagementFeatures.cc` to implement a single feature \ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
in `EntityManagementFeatures` "FeatureList" using TPE API from `tpe/lib` in Gazebo Physics library.

Before we dive into the feature implementation, we need to understand how the features are defined.

The \ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature"
is declared in a function template file `gz-physics/include/gz/physics/ConstructEmpty.hh`.

Gazebo Physics library uses function templates to specify features that accept generic types.
The use of templates makes it easier to implement features using different physics engine APIs,
without having to repeat the entire code for a function.

The \ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature" example here
is implemented with TPE API, but a similar feature can also be implemented using DART API.

In this case, we are implementing a feature that is already defined by Gazebo Physics,
thus we do not need to write our own template function, and can just include the template in our header file.

But first, let's include the basics:

\snippet examples/simple_plugin/EntityManagementFeatures.hh basic include

Then, we include the specific feature template file and add it to the feature list:

\snippet examples/simple_plugin/EntityManagementFeatures.hh include feature

We also need to declare the feature function in the header file, but since the function is already declared
in the template file we just included, we need to override the generic declaration instead:

\snippet examples/simple_plugin/EntityManagementFeatures.hh override feature

The `EntityManagementFeatures` "FeatureList" here inherits from:
- (optionally) \ref gz::physics::tpelib::Base "Base"
class for foundation metadata definitions of Models, Joints, Links, and Shapes objects
of TPE to provide easy access to [tpelib](https://github.com/gazebosim/gz-physics/tree/main/tpe/lib)
structures in the TPE library.
- \ref gz::physics::Implements3d "Implements3d" for implementing the
custom feature with \ref gz::physics::FeaturePolicy3d "FeaturePolicy3d"
("FeaturePolicy" of 3 dimensions and scalar type `double`).

Then we can go ahead with the implementation of `ConstructEmptyWorldFeature`:

\snippet examples/simple_plugin/EntityManagementFeatures.cc code

Here we show the overriding of `ConstructEmptyWorld` member function of
\ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature",
this is where we use the physics engine API to implement this member function.
We simply instantiate \ref gz::physics::tpelib::World "World" object, set
the world name and call \ref gz::physics::tpelib::Base::AddWorld "AddWorld"
function which was defined in [Base.hh](https://github.com/gazebosim/gz-physics/blob/main/tpe/plugin/src/Base.hh).

Simple unit tests are good practice for sanity checks.
While we won't go into detail, here is an example to test our new
\ref gz::physics::ConstructEmptyWorldFeature "ConstructEmptyWorldFeature":

\snippet examples/simple_plugin/EntityManagementFeatures_TEST.cc code

To get a more comprehensive view of how `EntityManagementFeatures` are constructed in TPE and Dartsim,
feel free to take a look here:
- Dartsim: [EntityManagementFeatures.hh](https://github.com/gazebosim/gz-physics/blob/main/dartsim/src/EntityManagementFeatures.hh) and [EntityManagementFeatures.cc](https://github.com/gazebosim/gz-physics/blob/main/dartsim/src/EntityManagementFeatures.cc)
- TPE: [EntityManagementFeatures.hh](https://github.com/gazebosim/gz-physics/blob/main/tpe/plugin/src/EntityManagementFeatures.hh) and [EntityManagementFeatures.cc](https://github.com/gazebosim/gz-physics/blob/main/tpe/plugin/src/EntityManagementFeatures.cc)

## Compile and run the example

Clone the source code, create a build directory and use `cmake` to compile the code:

```bash
git clone https://github.com/gazebosim/gz-physics
cd gz-physics/examples/simple_plugin
mkdir build
cd build
cmake ..
# Linux
cmake --build . --target PluginTest
# Windows
cmake --build . --target PluginTest --config Release
```

Run the test to verify the new physis plugin:

```bash
# Linux
./PluginTest

# Windows
.\Release\PluginTest.exe
```

You'll see:

```bash
$ ./PluginTest
Created empty world!
```

Once you implement more features, you could try passing `SimplePlugin` as the physics engine
to Gazebo Sim following \ref physicsengine "Use different physics engines" tutorial, e.g. setting

    # Linux
    export GZ_SIM_PHYSICS_ENGINE_PATH=$GZ_SIM_PHYSICS_ENGINE_PATH:$(pwd)/build
    # Windows
    set GZ_SIM_PHYSICS_ENGINE_PATH=<PATH_TO_THE_EXAMPLE_DIR>\build\Release

And run Gazebo sim with

    gz sim -v4 -s --physics-engine SimplePlugin

However, with the poor one feature we have implemented in this tutorial, you will only see an error,
because Gazebo Sim needs much more features:

    [error] [Physics.cc:854] No physics plugins implementing required interface found in library
    [D:\programming\gz9-ws\gz-ws\src\gz-physics\examples\simple_plugin\build\Release\SimplePlugin.dll].
