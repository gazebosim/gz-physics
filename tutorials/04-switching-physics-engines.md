\page switchphysicsengines Physics engines

This tutorial describes how to switch between physics engines in Ignition Physics when using Ignition Gazebo.
Ignition Physics enables Ignition Gazebo to choose flexibly what physics engine to use at runtime.
By default, Ignition Gazebo uses the [DART](https://dartsim.github.io/) physics engine.

Downstream developers may also integrate other physics engines by creating new Ignition Physics engine plugins.
See [Ignition Physics](https://ignitionrobotics.org/api/physics/3.0/tutorials.html)'s tutorials to learn how to integrate a new engine.

## How Ignition Gazebo finds physics engines

Ignition Gazebo automatically looks for all physics engine plugins that are installed with Ignition Physics.
At the time of writing, there are two physics engines available (more detail in \ref physicsplugin "Physics plugin tutorial"):
- **DART**: `ignition-physics-dartsim-plugin`.
- **TPE**: `ignition-physics-tpe-plugin`.

If you've created a custom engine plugin, you can tell Ignition Gazebo where to find it by setting the `IGN_GAZEBO_PHYSICS_ENGINE_PATH` environment variable to the directory where the plugin's shared library can be found.

For example, if you've created the following physics engine shared library on Linux:

`/home/physics_engines/libCustomEngine.so`

You should set the variable as follows:

`export IGN_GAZEBO_PHYSICS_ENGINE_PATH=/home/physics_engines`

If you have several libraries installed in different paths, you can add more paths, for example:

`export IGN_GAZEBO_PHYSICS_ENGINE_PATH=/home/physics_engines:/home/more_engines`

For additional environment variables that Ignition Gazebo finds other plugins or resources, you can see them by:

`ign gazebo -h`

## Pointing Ignition Gazebo to physics engines

There are a few different ways of telling Ignition Gazebo which engine to load.

In any way, the standard naming for your plugin's shared library is to have a `lib` prefix and the file extension.
Following this naming convention, the name should be `libCustomEngine.so` but the `CustomEngine` could also work.

### From SDF

You can specify Ignition Gazebo which engine to load from the SDF world file by giving the shared library name to the `Physics` plugin tag.
For the example above, you can load it like this:

```{.xml}
<plugin
  filename="ignition-gazebo-physics-system"
  name="ignition::gazebo::systems::Physics">
  <engine>
    <filename>CustomEngine</filename>
  </engine>
</plugin>
```

### From the command line

Alternatively, you can choose a plugin from the command line using the
`--physics-engine` option, for example:

`ign gazebo --physics-engine CustomEngine`

### From C++ API

All features available through the command line are also available through [ignition::gazebo::ServerConfig](https://ignitionrobotics.org/api/gazebo/4.0/classignition_1_1gazebo_1_1ServerConfig.html).
When instantiating a server programmatically, a physics engine can be passed to the constructor, for example:

```
ignition::gazebo::ServerConfig serverConfig;
serverConfig.SetPhysicsEngine("CustomEngine");

ignition::gazebo::Server server(serverConfig);
```

## Troubleshooting
These are common error messages that you may encounter:

> Failed to find plugin [libCustomEngine.so]. Have you checked the IGN_GAZEBO_PHYSICS_ENGINE_PATH environment variable?

Ignition Gazebo can't find out where `libCustomEngine.so` is located.

If that's an engine you believe should be installed with Ignition Physics, check if the relevant plugin is installed.

If that's a 3rd party engine, find where the `.so` file is installed and add that path to the environment variable as described above.

> Unable to load the [/home/physics_engines/libCustomEngine.so] library.

There was some problem loading that file. Check that it exists, that you have permissions to access it, and that it's a physics engine plugin.

> No plugins with all required features found in library [/home/physics_engines/libCustomEngine.so]

This means that there are plugins on that library, but none of them satisfies the minimum requirement of features needed to run an Ignition Gazebo simulation.
Be sure to implement all the necessary features.

> Failed to load a valid physics engine from [/home/physics_engines/libCustomEngine.so]

Some engines were found in that library, but none of them could be loaded.
Check that the engines implement all the necessary features.
