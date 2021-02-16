\page createphysicsplugin "Implement a physics plugin"

This tutorial guides how to develop a simple plugin that implements a
\ref ignition::physics::Implements3d "3D policy" and includes a sample list of
\ref ignition::physics::Feature "Feature" Features.

## Prerequisites

In the previous tutorial \ref installation "Installation", you have installed
the Ignition Physics corresponding to the desired Ignition release.

## Write a simple physics plugin

Please create a folder for the plugin first:

```bash
cd ~
mkdir -p ~/simple_plugin/build
cd simple_plugin
```

Then download the example loader into your current directory by:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-physics/ign-physics3/examples/hello_world_plugin/HelloWorldPlugin.cc
```

### Examine the code

We first include these:
- \ref ignition::physics::FeatureList "FeatureList" header: to define a list of
\ref ignition::physics::Feature "Feature" that will be implemented for the
plugin, e.g. \ref ignition::physics::GetEngineInfo "GetEngineInfo".
- \ref ignition::physics::FeaturePolicy "FeaturePolicy" header: to specify the
metadata about the coordinate system (e.g. 2 or 3 dimensions) and numeric system
(e.g. float, int) that the `Features` will be implemented accordingly.
- `GetEntities.hh` header: to retrieve the pre-defined list of features in
Ignition Physics. Please refer to this [API Reference](https://ignitionrobotics.org/api/physics/3.0/GetEntities_8hh.html)
for a list of features defined in `GetEntities.hh` and \ref physicsplugin
"Understanding the Physics Plugin" tutorial for an example list of common features
implemented with a plugin.
- `Register.hh` header: to register a plugin to Ignition Physics.

```cpp
#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FeaturePolicy.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Register.hh>
```

Next, let us define a namespace `mock` and implement our plugin in it.
We will add a pre-defined feature \ref ignition::physics::GetEngineInfo "GetEngineInfo"
to our plugin's `HelloWorldFeatureList` as follow:

```cpp
struct HelloWorldFeatureList : ignition::physics::FeatureList<
    ignition::physics::GetEngineInfo
> { };
```

The plugin will be able to return its physics engine metadata.
We will now implement our plugin class named `HelloWorldPlugin`
using the defined \ref ignition::physics::FeatureList `FeatureList` above.
The class is inherited from \ref ignition::physics::Implements3d "Implements3d"
to declare that our plugin's `HelloWorldFeatureList` will be in the 3D
coordinate system.

```cpp
class HelloWorldPlugin
    : public ignition::physics::Implements3d<HelloWorldFeatureList>
{
  using Identity = ignition::physics::Identity;

  public: Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    this->engineName = "HelloWorld";

    return this->GenerateIdentity(0);
  }

  public: std::size_t GetEngineIndex(const Identity &/*_id*/) const override
  {
    return 0;
  }

  public: const std::string &GetEngineName(const Identity &/*_id*/) const override
  {
    return this->engineName;
  }

  std::string engineName;
};
```

For now, because we do not have any real physics engines, we will define a dummy
physics engines inside member function `InitiateEngine` by simply setting
```cpp
this->engineName = "HelloWorld"
```
and returning the engine object using \ref ignition::physics::Identity. Then, we
define the metadata getters `GetEngineIndex` and `GetEngineName` for the
feature \ref ignition::physics::GetEngineInfo "GetEngineInfo" (please look into
corresponding public member functions defined in the subclasses). A list of common
pre-defined features are stated [here](https://ignitionrobotics.org/api/physics/3.0/GetEntities_8hh.html).

Finally, we only have to register our plugin in Ignition Physics as a physics
engine by:

```cpp
IGN_PHYSICS_ADD_PLUGIN(
    HelloWorldPlugin,
    ignition::physics::FeaturePolicy3d,
    HelloWorldFeatureList)
```
Note that:
- The first argument is the name of the class that wraps the physics engine into
a plugin.
- The second argument is the `FeaturePolicy` for this plugin, i.e. `FeaturePolicy3d`
- The third argument is the `FeatureList`, specifying all the features that this
plugin provides, i.e. `HelloWorldFeatureList`

### Setup CMakeLists.txt for building (Version: Dome, ign-physics3)

Now create a file named `CMakeLists.txt` with your favorite editor and add these
lines for finding `ign-plugin` and `ign-physics` dependencies for Dome release:

```cmake
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
set(IGN_PLUGIN_VER 1)
find_package(ignition-plugin${IGN_PLUGIN_VER} 1.1 REQUIRED COMPONENTS all)
set(IGN_PHYSICS_VER 3)
find_package(ignition-physics${IGN_PHYSICS_VER} REQUIRED)
```

After that, add the executable pointing to our plugin file and add linking
library so that `cmake` can compile it:

```cmake
add_library(HelloWorldPlugin SHARED HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin
  PRIVATE
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})
```

## Build and run

### Compile the plugin

Your current plugin folder should look like this:

```bash
$ ls ~/simple_plugin
CMakeLists.txt  HelloWorldPlugin.cc  build
```

Now you can build the plugin by:

```bash
cd ~/simple_plugin/build
cmake ..
make
```

This will generate the `HelloWorldPlugin` library under `build`.
The exact name of the library file depends on the operating system
such as `libHelloWorldPlugin.so` on Linux, `libHelloWorldPlugin.dylib` on MacOS,
and `HelloWorldPlugin.dll` on Windows.

### Test loading the plugin on Linux

Please first follow the \ref pluginloading "Loading a Physics Plugin" tutorial
to create a simple loader. Then we test our plugin using the loader as follow:

```bash
cd ~
./simple_loader/build/hello_world_loader simple_plugin/build/libHelloWorldPlugin.so
```

And you will see the engine info of our plugin:

```bash
Testing plugin: mock::HelloWorldPlugin
  engine name: HelloWorld
```
