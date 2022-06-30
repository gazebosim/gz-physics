# Engine plugin

This example shows how to create a plugin that integrates a physics engine with
Gazebo Physics.

## Build

From the root of the `gz-physics` repository, do the following to build the example:

```bash
cd examples/hello_world_plugin
mkdir build
cd build
cmake ..
```

### Ubuntu Or MacOS

```bash
make
```

### Windows

```bash
cmake --build . --config Release
```

This will generate the `HelloWorldPlugin` library under `build`.
The exact name of the library file depends on the operating system
such as `libHelloWorldPlugin.so` on Linux, `libHelloWorldPlugin.dylib` on macOS,
and `HelloWorldPlugin.dll` on Windows.

## Run

The resulting plugin can be loaded by any plugin loader that requests the
features implemented by the plugin. See an example loader at
`examples/hello_world_loader`.
