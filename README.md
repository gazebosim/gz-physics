# Ignition Physics : Physics classes and functions for robot applications

**Maintainer:** scpeters AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/ignitionrobotics/ign-physics.svg)](https://github.com/ignitionrobotics/ign-physics/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/ignitionrobotics/ign-physics.svg)](https://github.com/ignitionrobotics/ign-physics/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/ignitionrobotics/ign-physics/branch/ign-physics6/graph/badge.svg)](https://codecov.io/gh/ignitionrobotics/ign-physics)
Ubuntu Focal | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_physics-ci-ign-physics6-focal-amd64)](https://build.osrfoundation.org/job/ignition_physics-ci-ign-physics6-focal-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_physics-ci-ign-physics6-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_physics-ci-ign-physics6-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ign_physics-ci-win)](https://build.osrfoundation.org/job/ign_physics-ci-win)

Ignition Physics, a component of [Ignition
Robotics](https://ignitionrobotics.org), provides an abstract physics interface
designed to support simulation and rapid development of robot applications.

# Table of Contents

[Motivation](#motivation)

[Features](#features)

[Install](#install)

[Usage](#usage)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#code-of-contributing)

[Versioning](#versioning)

[License](#license)

# Motivation

Many physics simulation software libraries have been designed for different
applications (gaming, robotics, science) and with different features
(rigid or deformable contact, 2d or 3d).
Ignition Physics is designed on the premise that there is not a single physics
engine that is universally best for all simulation contexts.
It should be possible to support a different set of features
for each physics engine according to its capabilities.
A physics engine can then be chosen for each application
based on its context.

# Features

Ignition Physics provides the following functionality:

* Granular definition of physics engine features as optional API's.
* Plugin interface for loading physics engines with requested features
  at runtime.
* Features for common aspects of rigid body dynamic simulation
    - Construct model from [SDFormat](http://sdformat.org/) file.
    - Collision shapes (such as box, sphere, cylinder, capsule, ellipsoid, mesh, heightmap).
    - Joint types (such as revolute, prismatic, fixed, ball, screw, universal).
    - Step simulation, get/set state, apply inputs.
* Reference implementation of physics plugin using
  [dartsim](http://dartsim.github.io/).
* A custom physics engine focused on fast kinematics of large environments, the
  [Trivial Physics Engine](https://community.gazebosim.org/t/announcing-new-physics-engine-tpe-trivial-physics-engine/629).
* `CompositeData` structures for efficiently using native types in API.

# Install

See the [installation tutorial](https://ignitionrobotics.org/api/physics/5.0/installation.html).

# Usage

Please refer to the [examples directory](https://github.com/ignitionrobotics/ign-physics/raw/ign-physics6/examples/).

# Documentation

API and tutorials can be found at [https://ignitionrobotics.org/libs/physics](https://ignitionrobotics.org/libs/physics).

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen. On Ubuntu Doxygen can be installed using

    ```
    sudo apt-get install doxygen
    ```

2. Clone the repository

    ```
    git clone https://github.com/ignitionrobotics/ign-physics -b ign-physics6
    ```

3. Configure and build the documentation.

    ```
    cd ign-physics; mkdir build; cd build; cmake ../; make doc
    ```

4. View the documentation by running the following command from the build directory.

    ```
    firefox doxygen/html/index.html
    ```

# Testing

Follow these steps to run tests and static code analysis in your clone of this repository.

1. Follow the [source install instruction](#source-install).

2. Run tests.

    ```
    make test
    ```

3. Static code checker.

    ```
    make codecheck
    ```

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
ign-physics
├── bullet                    Files for bullet plugin component.
├── dartsim                   Files for dartsim plugin component.
├── example                   Examples about how to use the library
├── heightmap                 Heightmap related header files.
├── include/ignition/physics  Header files.
├── mesh                      Files for mesh component.
├── resources                 Model and mesh resource files used by tests.
├── sdf                       Files for sdf component.
├── src                       Source files and unit tests.
├── test
│    ├── benchmark            Benchmark tests.
│    ├── integration          Integration tests.
│    ├── performance          Performance tests.
│    ├── plugins              Plugins used in tests.
│    ├── regression           Regression tests.
│    └── static_assert        Tests involving compilation failures.
├── tpe
│    ├── lib                  Implementation of TPE engine.
│    └── plugin               Files for TPE plugin component.
├── tutorials                 Tutorials, written in markdown.
├── Changelog.md              Changelog.
└── CMakeLists.txt            CMake build script.
```
# Contributing

Please see the [contribution guide](https://ignitionrobotics.org/docs/all/contributing).

# Code of Conduct

Please see
[CODE\_OF\_CONDUCT.md](https://github.com/ignitionrobotics/ign-gazebo/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/ignitionrobotics/ign-physics/blob/main/LICENSE) file.
