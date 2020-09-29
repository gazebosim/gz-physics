\page introduction Introduction

Next Tutorial: \ref installation

Ignition Physics is a component in Ignition, a set of libraries
designed to rapidly develop robot and simulation applications. The main
goal of the library is to provide an abstraction layer to various physics
engines, which gives end users the ability to use multiple physics engines
with minimal change to their application code. Ignition Physics uses
a plugin architecture where each physics engine is implemented as a plugin
that can be loaded at runtime. To enable users the ability to choose
between physics engines, Ignition Physics introduces [\"Features\"](@ref ignition::physics::Feature), a design concept used to encode the
capabilities of a physics engine. Users can request for physics engines
that support a set of features and the plugin loading mechanism loads only
the engines that implement the requested features.

The following is a list of currently supported physics engines:

  * [DART](http://dartsim.github.io/)
