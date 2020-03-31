# Engine plugin

This example shows how to create a plugin that integrates a physics engine with
Ignition Physics.

## Build

From the root of the `ign-physics` repository, do the following to build the example:

~~~
cd ign-physics/examples/hello_world_plugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `libHelloWorldPlugin.so` library under `build`.

## Run

The resulting plugin can be loaded by any plugin loader that requests the
features implemented by the plugin. See an example loader at
`ign-physics/examples/hello_world_loader`.

