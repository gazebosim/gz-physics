# Engine loader

This example shows how to create an executable that loads physics engine
plugins, according to a desired feature list.

## Build

From the root of the `ign-physics` repository, do the following to build the example:

~~~
cd examples/hello_world_loader
mkdir build
cd build
cmake ..
make
~~~

This will generate the `hello_world_loader` executable under `build`.

## Run

This loader will load any plugin that implements the `GetEngineInfo` feature,
and print the engine name.

### DART plugin on Linux

For example, if you have the Ignition Physics plugin for DART compiled, find
where it's installed with:

~~~
find / | grep libignition-physics-dartsim-plugin.so
~~~

You may find more than one file. Choose one of them, and load it with
the example loader like this:

~~~
cd examples/hello_world_loader/build
./hello_world_loader <path_to>/libignition-physics-dartsim-plugin.so
~~~

And you'll see the engine info:

~~~
Testing plugin: ignition::physics::dartsim::Plugin
  engine name: dartsim-6.10.0
~~~

### Hello world plugin on Linux

This loader should also be able to load the hello world plugin example.
After you've followed the instructions to compile both the plugin and the
loader, you can load the plugin like:

~~~
cd examples/hello_world_loader/build
./hello_world_loader ../../hello_world_plugin/build/libHelloWorldPlugin.so
~~~

And see the following printed:

~~~
Testing plugin: mock::HelloWorldPlugin
  engine name: HelloWorld
~~~
