\page pluginloading Plugin Loading

This tutorial shows how to create an executable that loads physics engine
plugins on Ubuntu, according to the desired feature list.

## Overview

Physics Plugin integrates external physics engines into the Ignition Physics.
It allows users to select from multiple supported physics engines based on their
simulation needs. This tutorial will describe how to load a compiled physics
plugin using \ref ignition::physics "Ignition Physics" API.

## Prerequisites

In the previous tutorial \ref installation "Installation", you have installed the
Ignition Physics corresponding to the desired Ignition release. Note that the
recommended Ignition release is Dome.

## Write a simple loader

Please create a folder for the loader first:

```bash
cd ~
mkdir -p ~/simple_loader/build
cd simple_loader
```

Then download the example loader into your current directory by:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-physics/main/examples/hello_world_loader/hello_world_loader.cc
```

### Examine the code

At the top of the file `hello_world_loader.cc`, we include the headers that will
be used in our code. After the `std` C++ libraries are the `Loader.hh` and
`PluginPtr.hh`, which provides main functionalities for loading physics plugins
and plugin pointers. Next includes from \ref ignition::physics are the tools for
retrieving \ref ignition::physics::Feature "Feature" and
\ref ignition::physics::Entity "Entity" from physics plugins (please refer to
\ref physicsplugin "Understanding the Physics Plugin" tutorial for their
design concepts).

```cpp
#include <iostream>

#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/PluginPtr.hh>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/RequestEngine.hh>

using Features = ignition::physics::FeatureList<
    ignition::physics::GetEngineInfo
>;
```

Next, in the main function, the loader requires users to provide a path for
desired plugins to be loaded. The plugin names are retrieved by
@ref ignition::plugin::Loader::LoadLib member function.

```cpp
if (argc <= 1)
{
  std::cerr << "Please provide the path to an engine plugin." << std::endl;
  return 1;
}

std::string pluginPath = argv[1];

ignition::plugin::Loader pl;
auto plugins = pl.LoadLib(pluginPath);
```

Assuming the correct path, our loader will instantiate all plugins that are
available in the path using @ref ignition::plugin::Loader::Instantiate member
function. Then for each instantiated plugin, using
@ref ignition::physics::RequestEngine3d<Features>::From, it will request an
engine implementing a \ref ignition::physics::FeaturePolicy "FeaturePolicy" (3D
  in this case).
.

```cpp
for (const std::string &name : pluginNames)
{
  std::cout << "Testing plugin: " << name << std::endl;
  ignition::plugin::PluginPtr plugin = pl.Instantiate(name);

  auto engine = ignition::physics::RequestEngine3d<Features>::From(plugin);

  std::cout << "  engine name: " << engine->GetName() << std::endl;
}
```

### Setup CMakeLists.txt for building (Version: Dome, ign-physics4)

Now create a file named `CMakeLists.txt` with your favorite editor and add these
lines for finding `ign-plugin` and `ign-physics` dependencies in Dome release:

```cmake
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(IGN_PLUGIN_VER 1)
find_package(ignition-plugin${IGN_PLUGIN_VER} 1.1 REQUIRED COMPONENTS all)

set(IGN_PHYSICS_VER 3)
find_package(ignition-physics${IGN_PHYSICS_VER} REQUIRED)
```

After that, add the executable pointing to our file and add linking library so
that `cmake` can compile it:

```cmake
add_executable(hello_world_loader hello_world_loader.cc)
target_link_libraries(hello_world_loader
  ignition-plugin${IGN_PLUGIN_VER}::loader
  ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})
```

For a comprehensive CMake tutorial, please take a look
[here](https://cmake.org/cmake/help/latest/guide/tutorial/index.html).

## Build and run

### Compile the loader

Your current loader folder should look like this:
```bash
$ ls ~/simple_loader
CMakeLists.txt  hello_world_loader.cc  build
```

Now you can build the loader by:

```bash
cd build
cmake ..
make
```

This will generate the `hello_world_loader` executable under `build` folder.
This loader will load any plugin that implements the `GetEngineInfo` feature,
and print the engine name.

### Load existing plugins

For example, if you have the Ignition Physics plugin for
[DART](https://dartsim.github.io/) compiled, find where it is installed with
(you may need administrative rights: `sudo` on Linux platform):

```bash
find / | grep libignition-physics-dartsim-plugin.so
```

You may find more than one file. Choose one of them, and load it with
the loader by:

```bash
./hello_world_loader <path_to>/libignition-physics-dartsim-plugin.so
```

And you'll see the engine info:

```bash
Testing plugin: ignition::physics::dartsim::Plugin
  engine name: dartsim-6.10.0
```

At the time of writing, Ignition Physics is shipped with
[DART](https://dartsim.github.io/) and [TPE](https://community.gazebosim.org/t/announcing-new-physics-engine-tpe-trivial-physics-engine/629)
physics plugins installed. Following the above steps, you can load `TPE` by the
library name `libignition-physics-tpe-plugin.so` or other custom plugins by
their corresponding names.
