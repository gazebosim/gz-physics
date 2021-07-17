\page pluginloading Plugin Loading

This tutorial shows how to create an executable that loads physics engine
plugins on Ubuntu, according to the desired feature list.

## Overview

Physics Plugin integrates external physics engines into the Ignition Physics.
It allows users to select from multiple supported physics engines based on their
simulation needs. This tutorial will describe how to load a compiled physics
plugin using \ref ignition::physics "Ignition Physics" API.

## Prerequisites

- \ref installation "Installation"
- \ref physicsplugin "Understand physics plugin"

## Write a simple loader

We will use a simplified physics plugin example for this tutorial. Source code can be found at [ign-physics2/examples](https://github.com/ignitionrobotics/ign-physics/tree/ign-physics3/examples/hello_world_loader) folder.

First, create a workspace for the example plugin loader.

```bash
cd ~
mkdir -p ~/hello_world_loader/build
cd hello_world_loader
```

Then download the example loader into your current directory by:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-physics/ign-physics4/examples/hello_world_loader/hello_world_loader.cc
```

### Examine the code

At the top of the file `hello_world_loader.cc`, we include the headers that will
be used in our code. After the `std` C++ libraries are the `Loader.hh` and
`PluginPtr.hh`, which provides main functionalities for loading physics plugins
and plugin pointers. Next includes from \ref ignition::physics are the tools for
retrieving \ref ignition::physics::Feature "Feature" and
\ref ignition::physics::Entity "Entity" from physics plugins (please refer to
\ref physicsplugin "Understand Physics Plugin" tutorial for their
design concepts).

\snippet examples/hello_world_loader/hello_world_loader.cc include statements

Next, in the main function, the loader requires users to provide a path for
desired plugins to be loaded. The plugin names are retrieved by
@ref ignition::plugin::Loader::LoadLib member function.

Assuming the correct path, our loader will instantiate all plugins that are
available in the path using @ref ignition::plugin::Loader::Instantiate member
function. Then for each instantiated plugin, using
@ref ignition::physics::RequestEngine3d<Features>::From, it will request an
engine implementing a \ref ignition::physics::FeaturePolicy "FeaturePolicy" (3D
  in this case).
.

\snippet examples/hello_world_loader/hello_world_loader.cc main

### Setup CMakeLists.txt for CMake build

Now create a file named `CMakeLists.txt` with your favorite editor and add these
lines for finding `ign-plugin` and `ign-physics` dependencies in Citadel release.
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

