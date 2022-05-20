\page switchphysicsengines Switching physics engines

This tutorial describes how to switch between physics engines in Gazebo Physics
when using Gazebo.
Gazebo Physics enables Gazebo to choose flexibly what physics engine
to use at runtime.
By default, Gazebo uses the [DART](https://dartsim.github.io/) physics
engine.

Downstream developers may also integrate other physics engines by creating new
Gazebo Physics engine plugins.
See \ref tutorials "Gazebo Physics'" tutorials to learn how to integrate a
new engine.

## How Gazebo finds physics engines

Gazebo automatically looks for all physics engine plugins that are
installed with Gazebo Physics.
At the time of writing, there are two physics engines available (more detail
in \ref physicsplugin "Physics plugin tutorial"):
- **DART**: `ignition-physics-dartsim-plugin`.
- **TPE**: `ignition-physics-tpe-plugin`.

If you've created a custom engine plugin, you can tell Gazebo where to
find it by setting the `GZ_SIM_PHYSICS_ENGINE_PATH` environment variable to
the directory where the plugin's shared library can be found.

For example, if you've created the following physics engine shared library on Linux:

`/home/physics_engines/libCustomEngine.so`

You should set the variable as follows:

```bash
export GZ_SIM_PHYSICS_ENGINE_PATH=/home/physics_engines
```

If you have several libraries installed in different paths, you can add more
paths separated by a colon, for example:

```bash
export GZ_SIM_PHYSICS_ENGINE_PATH=/home/physics_engines:/home/more_engines
```

For additional environment variables that Gazebo finds other plugins
or resources, you can see them by:

```bash
ign gazebo -h
```

## Pointing Gazebo to physics engines

There are a few different ways of telling Gazebo which engine to load.

In any way, the standard naming for your plugin's shared library is to have a
`lib` prefix and the file extension.
Following this naming convention, the name should be `libCustomEngine.so` but
the `CustomEngine` could also work.

### From SDF

You can specify in Gazebo which engine to load from the SDF world file
by giving the shared library name to the `Physics` plugin tag.
For the example above, you can load it like this:

```{.xml}
<plugin
  filename="ignition-gazebo-physics-system"
  name="gz::gazebo::systems::Physics">
  <engine>
    <filename>CustomEngine</filename>
  </engine>
</plugin>
```

### From the command line

Alternatively, you can choose a plugin from the command line using the
`--physics-engine` option, for example:

```bash
ign gazebo --physics-engine CustomEngine
```

### From C++ API

All features available through the command line are also available through
[gz::gazebo::ServerConfig](https://ignitionrobotics.org/api/gazebo/4.0/classignition_1_1gazebo_1_1ServerConfig.html).
When instantiating a server programmatically, a physics engine can be passed to
the constructor, for example:

```
gz::gazebo::ServerConfig serverConfig;
serverConfig.SetPhysicsEngine("CustomEngine");

gz::gazebo::Server server(serverConfig);
```

## Troubleshooting
These are common error messages that you may encounter:

> Failed to find plugin [libCustomEngine.so]. Have you checked the
GZ_SIM_PHYSICS_ENGINE_PATH environment variable?

Gazebo can't find out where `libCustomEngine.so` is located.

If that's an engine you believe should be installed with Gazebo Physics,
check if the relevant plugin is installed.

If that's a 3rd party engine, find where the `.so` file is installed and add
that path to the environment variable as described above.

> Unable to load the [/home/physics_engines/libCustomEngine.so] library.

There was some problem loading that file. Check that it exists, that you have
permissions to access it, and that it's a physics engine plugin.

> No plugins with all required features found in library
[/home/physics_engines/libCustomEngine.so]

This means that there are plugins on that library, but none of them satisfies
the minimum requirement of features needed to run an Gazebo simulation.
Be sure to implement all the necessary features.

> Failed to load a valid physics engine from [/home/physics_engines/libCustomEngine.so]

Some engines were found in that library, but none of them could be loaded.
Check that the engines implement all the necessary features.
