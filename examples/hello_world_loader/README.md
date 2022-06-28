# Engine loader

This example shows how to create an executable that loads physics engine
plugins, according to a desired feature list.

## Build

From the root of the `gz-physics` repository, do the following to build the example:

```bash
cd examples/hello_world_loader
mkdir build
cd build
cmake ..
```

### Ubuntu Or MacOS

```bash
make
```

This will generate the `hello_world_loader` executable under `build`.

### Windows

```bash
cmake --build . --config Release
```

This will generate the `hello_world_loader` executable under `build\Release`.

## Run

This loader will load any plugin that implements the `GetEngineInfo` feature,
and print the engine name.

### DART plugin on Linux

For example, if you have the Gazebo Physics plugin for DART compiled, find
where it's installed with:

~~~
find / | grep libgz-physics-dartsim-plugin.so
~~~

You may find more than one file. Choose one of them, and load it with
the example loader like this:

~~~
cd examples/hello_world_loader/build
./hello_world_loader <path_to>/libgz-physics-dartsim-plugin.so
~~~

And you'll see the engine info:

~~~
Testing plugin: gz::physics::dartsim::Plugin
  engine name: dartsim-6.10.0
~~~

### Hello world plugin

This loader should also be able to load the hello world plugin example.
After you've followed the instructions to compile both the plugin and the
loader, you can load the plugin like:

#### On Linux

~~~
cd examples/hello_world_loader/build
./hello_world_loader ../../hello_world_plugin/build/libHelloWorldPlugin.so
~~~

#### On Windows

~~~
cd examples\hello_world_loader\build
.\Release\hello_world_loader.exe ..\..\hello_world_plugin\build\Release\HelloWorldPlugin.dll
~~~

And see the following printed:

~~~
Testing plugin: mock::HelloWorldPlugin
  engine name: HelloWorld
~~~
