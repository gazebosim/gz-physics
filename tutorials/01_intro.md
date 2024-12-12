\page introduction Introduction

Next Tutorial: \ref installation

## Motivation

Gazebo Physics (`gz-physics`) is a library component in Gazebo, a set of
libraries designed to rapidly develop robot and simulation applications.
The main goal of the library is to provide an abstraction layer to various
physics engines, which gives end-users the ability to use multiple
physics engines with minimal change to their application code.
Users can request for physics engines that support a set of features and the plugin
loading mechanism loads only the engines that implement the requested features.
Besides, a user-selected physics engine can be integrated into the existing
code-base by writing a compatible plugin interface, which enables
Gazebo Physics extensibility and modularity.

## Overview

For a big picture of the Gazebo Physics operation in Gazebo ecosystem, see
the abstract diagram below:

@image html img/gz-libraries.png

In general, `gz-sim` is the main simulation library, in which its
functionalities are powered by many component libraries.
For example, its graphical drawing is supported by `gz-rendering` or simulated
sensors that are defined and implemented in `gz-sensors`.
In particular, this library `gz-physics` provides an abstract interface to
physics engines, which simulates dynamic transformations and interactions of
objects in `gz-sim`. The libraries are connected by C++ code.

Gazebo Physics uses a plugin architecture where each physics engine is
implemented as a plugin that can be loaded at runtime.
To enable users the ability to choose between physics engines, Gazebo Physics
introduces four keys elements:

- \ref gz::physics::Entity "Entity": the base class, which abstracts all
proxy objects, which contains minimal descriptive data
(e.g. a unique identifier, a reference-counter, etc.) pointing the corresponding
implemented objects.
- \ref gz::physics::FeaturePolicy "FeaturePolicy": the policy class,
which provides metadata to features about their simulation engine specifications.
FeaturePolicy supports customizing Gazebo Physics' APIs by the number of
dimensions (2D or 3D) and also the floating-point scalar type (float or double).
- \ref gz::physics::Feature "Feature": defines the concept used to encode
the capabilities of a physics engine.
- \ref gz::physics::FeatureList "FeatureList": aggregates a list of features.

Depending on which external physics engine is used (e.g. DART, TPE, Bullet, etc.),
the internal interface to the physics engine might be different.
However, the API provided by Gazebo Physics will be unchanged between
different physics engines.

## Supported physics engines

For a list of supported physics engines and their descriptions, please refer
to \ref physicsplugin
