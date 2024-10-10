\page pluginloading Plugin Loading

This tutorial shows how to create an executable that loads physics engine
plugins on Ubuntu, according to the desired feature list.

## Overview

Physics Plugin integrates external physics engines into the Gazebo Physics.
It allows users to select from multiple supported physics engines based on their
simulation needs. This tutorial will describe how to load a compiled physics
plugin using \ref gz::physics "Gazebo Physics" API.

## Prerequisites

- \ref installation "Installation"
- \ref physicsplugin "Understanding the physics plugin"

## Write a simple loader

We will use a simplified physics plugin example for this tutorial. Source code can be found at [gz-physics/examples](https://github.com/gazebosim/gz-physics/tree/main/examples/hello_world_loader) folder.

First, create a workspace for the example plugin loader.

```bash
cd ~
mkdir -p ~/hello_world_loader/build
cd hello_world_loader
```

Then download the example loader into your current directory by:

```bash
wget https://raw.githubusercontent.com/gazebosim/gz-physics/main/examples/hello_world_loader/hello_world_loader.cc
```

### Examine the code

At the top of the file `hello_world_loader.cc`, we include the headers that will
be used in our code. After the `std` C++ libraries are the `Loader.hh` and
`PluginPtr.hh`, which provides main functionalities for loading physics plugins
and plugin pointers. Next includes from \ref gz::physics are the tools for
retrieving \ref gz::physics::Feature "Feature" and
\ref gz::physics::Entity "Entity" from physics plugins (please refer to
\ref physicsplugin "Understanding the physics plugin" tutorial for their
design concepts).

\snippet examples/hello_world_loader/hello_world_loader.cc include statements

Next, in the main function, the loader requires users to provide a path for
desired plugins to be loaded. The plugin names are retrieved by
@ref gz::plugin::Loader::LoadLib member function.

Assuming the correct path, our loader will instantiate all plugins that are
available in the path using @ref gz::plugin::Loader::Instantiate member
function. Then for each instantiated plugin, using
@ref gz::physics::RequestEngine3d<Features>::From, it will request an
engine implementing a \ref gz::physics::FeaturePolicy "FeaturePolicy" (3D
 in this case).

\snippet examples/hello_world_loader/hello_world_loader.cc main

### Setup CMakeLists.txt for CMake build

Now create a file named `CMakeLists.txt` with your favorite editor and add these
lines for finding `gz-plugin` and `gz-physics` dependencies in Citadel release.
After that, add the executable pointing to our file and add linking library so
that `cmake` can compile it.

\include examples/hello_world_loader/CMakeLists.txt

If you find CMake syntax difficult to understand, take a look at the official tutorial [here](https://cmake.org/cmake/help/latest/guide/tutorial/index.html).

## Build and run

### Compile the loader

Your current loader folder should look like this

```bash
$ ls ~/hello_world_loader
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

For example, if you have the Gazebo Physics plugin for
[DART](https://dartsim.github.io/) compiled, find where it is installed with
(you may need administrative rights: `sudo` on Linux platform):

```bash
find / | grep libgz-physics-dartsim-plugin.so
```

You may find more than one file. Choose one of them, and load it with
the loader by:

```bash
./hello_world_loader <path_to>/libgz-physics-dartsim-plugin.so
```

And you'll see the engine info:

```bash
Testing plugin: gz::physics::dartsim::Plugin
  engine name: dartsim-6.13.2
```

At the time of writing, Gazebo Physics is shipped with
[DART](https://dartsim.github.io/) and [TPE](https://community.gazebosim.org/t/announcing-new-physics-engine-tpe-trivial-physics-engine/629)
physics plugins installed. Following the above steps, you can load `TPE` by the
library name `libgz-physics-tpe-plugin.so` or other custom plugins by
their corresponding names.

