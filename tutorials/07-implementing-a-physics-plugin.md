\page createphysicsplugin Implement physics feature

This tutorial shows how to develop a simple plugin that implements a
\ref ignition::physics::Implements3d "3D policy" and includes a sample list of
\ref ignition::physics::Feature "Feature" Features.

## Prerequisites

- \ref installation "Installation"
- \ref physicsplugin "Understand physics plugin"
- \ref loadplugin "Load physics plugin"

## Write a simple physics plugin

First we need to create a workspace

```bash
cd ~
mkdir -p ~/hello_world_plugin/build
cd hello_world_plugin
```

### Examine the code

To set up all libraries needed to implement physics feature, we need to include the following:
- \ref ignition::physics::FeatureList "FeatureList" header: to define a list of
\ref ignition::physics::Feature "Feature" that will be implemented for the
plugin, e.g. \ref ignition::physics::GetEngineInfo "GetEngineInfo".
- \ref ignition::physics::FeaturePolicy "FeaturePolicy" header: to specify the
metadata about the coordinate system (e.g. 2 or 3 dimensions) and numeric system
(e.g. float, int) that the `Features` will be implemented accordingly.
- `GetEntities.hh` header: to retrieve the pre-defined list of features in
Ignition Physics. Please refer to this [API Reference](https://ignitionrobotics.org/api/physics/2.0/GetEntities_8hh.html)
for a list of features defined in `GetEntities.hh` and \ref physicsplugin
"Understanding the Physics Plugin" tutorial for an example list of common features
implemented with a plugin.
- `Register.hh` header: to register a plugin to Ignition Physics.

\snippet examples/hello_world_plugin/HelloWorldPlugin.cc include statements

Next, we use a dummy namespace `mock` and list all the features we would like to implement.

\snippet examples/hello_world_plugin/HelloWorldPlugin.cc feature list

The plugin will be able to return its physics engine metadata.
We will now implement our plugin class named `HelloWorldPlugin`
using the defined \ref ignition::physics::FeatureList `FeatureList` above.
The class is inherited from \ref ignition::physics::Implements3d "Implements3d"
to declare that the plugin's `HelloWorldFeatureList` will be in the 3D
coordinate system.

\snippet examples/hello_world_plugin/HelloWorldPlugin.cc implementation

Because we are not using a real physics engines, a dummy
physics engines is defined inside member function `InitiateEngine` by simply setting the `engineName` to `HelloWorld`,
and returning the engine object using \ref ignition::physics::Identity. Then, we
define the metadata getters `GetEngineIndex` and `GetEngineName` for the
feature \ref ignition::physics::GetEngineInfo "GetEngineInfo" (please look into
corresponding public member functions defined in the subclasses). A list of other
pre-defined features are can be found in the [`GetEntities` FeatureList](https://ignitionrobotics.org/api/physics/2.0/GetEntities_8hh.html).

Finally, we only have to register our plugin in Ignition Physics as a physics
engine by:

\snippet examples/hello_world_plugin/HelloWorldPlugin.cc register

Note that:
- The first argument is the name of the class that wraps the physics engine into
a plugin.
- The second argument is the `FeaturePolicy` for this plugin, i.e. `FeaturePolicy3d`
- The third argument is the `FeatureList`, specifying all the features that this
plugin provides, i.e. `HelloWorldFeatureList`

### Setup CMakeLists.txt for building (Version: ign-physics5)

Now create a file named `CMakeLists.txt` with your favorite editor and add these
lines for finding `ign-plugin` and `ign-physics` dependencies for the Fortress release:

\snippet examples/hello_world_plugin/CMakeLists.txt

## Build and run

### Compile the plugin

Your current plugin folder should look like this:

```bash
$ ls ~/hello_world_plugin
CMakeLists.txt  HelloWorldPlugin.cc  build
```

Now you can build the plugin by:

```bash
cd ~/hello_world_plugin/build
cmake ..
make
```

This will generate the `HelloWorldPlugin` library under `build`.
The exact name of the library file depends on the operating system
such as `libHelloWorldPlugin.so` on Linux, `libHelloWorldPlugin.dylib` on MacOS,
and `HelloWorldPlugin.dll` on Windows.

### Test loading the plugin on Linux

Please first follow the \ref loadplugin "Load physics plugin" tutorial
to create a simple loader. Then we test our plugin using the loader as follow:

```bash
cd ~
./hello_world_loader/build/hello_world_loader simple_plugin/build/libHelloWorldPlugin.so
```

And you will see the engine info of our plugin:

```bash
Testing plugin: mock::HelloWorldPlugin
  engine name: HelloWorld
```

