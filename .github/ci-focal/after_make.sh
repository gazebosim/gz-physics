#!/bin/sh -l

set -x

# Install
make install

# Compile examples
cd ../examples/hello_world_loader
mkdir build
cd build
cmake ..
make

# Compile examples
cd ../../hello_world_plugin
mkdir build
cd build
cmake ..
make

# return to the compilation place
cd ../../../build
