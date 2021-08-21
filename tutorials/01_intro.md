\page introduction Introduction

Next Tutorial: \ref installation

## Motivation

Ignition Physics (`ign-physics`) is a library component in Ignition, a set of
libraries designed to rapidly develop robot and simulation applications.
The main goal of the library is to provide an abstraction layer to various
physics engines, which gives end-users the ability to use multiple
physics engines with minimal change to their application code.
Users can request for physics engines that support a set of features and the plugin
loading mechanism loads only the engines that implement the requested features.
Besides, a user-selected physics engine can be integrated into the existing
code-base by writing a compatible plugin interface, which enables
Ignition Physics extensibility and modularity.  

## Overview

For a big picture of the Ignition Physics operation in Ignition ecosystem, see
the abstract diagram below:

@image html img/ign-libraries.png

In general, `ign-gazebo` is the main simulation library, in which its
functionalities are powered by many component libraries.
For example, its graphical drawing is supported by `ign-rendering` or simulated
sensors that are defined and implemented in `ign-sensors`.
In particular, this library `ign-physics` provides an abstract interface to
physics engines, which simulates dynamic transformations and interactions of
objects in `ign-gazebo`. The libraries are connected by C++ code.

Ignition Physics uses a plugin architecture where each physics engine is
implemented as a plugin that can be loaded at runtime.
To enable users the ability to choose between physics engines, Ignition Physics
introduces four keys elements:

- \ref ignition::physics::Entity "Entity": the base class, which abstracts all
proxy objects, which contains minimal descriptive data
(e.g. a unique identifier, a reference-counter, etc.) pointing the corresponding
implemented objects.
- \ref ignition::physics::FeaturePolicy "FeaturePolicy": the policy class,
which provides metadata to features about their simulation engine specifications.
FeaturePolicy supports customizing Ignition Physics' APIs by the number of
dimensions (2D or 3D) and also the floating-point scalar type (float or double).
- \ref ignition::physics::Feature "Feature": defines the concept used to encode
the capabilities of a physics engine.
- \ref ignition::physics::FeatureList "FeatureList": aggregates a list of features.

Depending on which external physics engine is used (e.g. DART, TPE, Bullet, etc.),
the internal interface to the physics engine might be different.
However, the API provided by Ignition Physics will be unchanged between
different physics engines.

## Supported physics engines

For a list of supported physics engines and their descriptions, please refer
to \ref physicsplugin

## Development timeline

### Features logs

**Ignition Physics 1.x**

- Initial release
- Define base concepts: Entity, FeaturePolicy, Feature and FeatureList.
- Add features for `dartsim` physics engines (more detail in \ref physicsplugin).
- Add RequestFeatures API for casting the features of an entity to a new feature set when possible.
- Enforce joint effort limit in `dartsim-plugin`.

**Ignition Physics 2.x**

- Support SDFormat 1.7 frame semantics.
- Support compiling against DART 6.9.
- Trivial Physics Engine (TPE)- partial implementation
- Add features for TPE physics engines (more detail in \ref physicsplugin).
- Extend contact data with force, normal, and penetration depth.
- Add Base and EntityManagement to `tpeplugin`

**Ignition Physics 3.x**

### Future roadmap

Ignition Physics evolves closely with the Ignition ecosystem.
For a broader overview, please visit [Ignition's roadmap](https://ignitionrobotics.org/about).
