# Gazebo Physics : Physics classes and functions for robot applications

**Maintainer:** scpeters AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/gz-physics.svg)](https://github.com/gazebosim/gz-physics/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/gz-physics.svg)](https://github.com/gazebosim/gz-physics/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/gz-physics/tree/gz-physics9/graph/badge.svg)](https://codecov.io/gh/gazebosim/gz-physics/tree/gz-physics9)
Ubuntu Noble  | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_physics-ci-gz-physics9-noble-amd64)](https://build.osrfoundation.org/job/gz_physics-ci-gz-physics9-noble-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_physics-ci-gz-physics9-homebrew-amd64)](https://build.osrfoundation.org/job/gz_physics-ci-gz-physics9-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_physics-9-cnlwin)](https://build.osrfoundation.org/job/gz_physics-9-cnlwin)

Gazebo Physics, a component of [Gazebo](https://gazebosim.org), provides an abstract physics interface
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
Gazebo Physics is designed on the premise that there is not a single physics
engine that is universally best for all simulation contexts.
It should be possible to support a different set of features
for each physics engine according to its capabilities.
A physics engine can then be chosen for each application
based on its context.

# Features

Gazebo Physics provides the following functionality:

* Granular definition of physics engine features as optional API's.
* Plugin interface for loading physics engines with requested features
  at runtime.
* Features for common aspects of rigid body dynamic simulation
    - Construct model from [SDFormat](http://sdformat.org/) file.
    - Collision shapes (such as box, sphere, cylinder, cone, capsule, ellipsoid, mesh, heightmap).
    - Joint types (such as revolute, prismatic, fixed, ball, screw, universal).
    - Step simulation, get/set state, apply inputs.
* Reference implementation of physics plugin using
  [dartsim](http://dartsim.github.io/).
* A custom physics engine focused on fast kinematics of large environments, the
  [Trivial Physics Engine](https://community.gazebosim.org/t/announcing-new-physics-engine-tpe-trivial-physics-engine/629).
* `CompositeData` structures for efficiently using native types in API.

# Install

See the [installation tutorial](https://gazebosim.org/api/physics/8/installation.html).

# Usage

Please refer to the [examples directory](https://github.com/gazebosim/gz-physics/raw/gz-physics9/examples/).

# Documentation

API and tutorials can be found at [https://gazebosim.org/libs/physics](https://gazebosim.org/libs/physics).

On Ubuntu, you can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen, which can be installed using

    ```
    sudo apt-get install doxygen
    ```

2. Install dependencies
   ```
   sudo apt-add-repository -s "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -c -s) main"
   sudo apt-get build-dep -y libgz-physics9-dev
   ```

3. Clone the repository

    ```
    git clone https://github.com/gazebosim/gz-physics -b gz-physics9
    ```

4. Configure and build the documentation.

    ```
    cd gz-physics; mkdir build; cd build; cmake ..; make doc
    ```

5. View the documentation by running the following command from the build directory.

    ```
    firefox doxygen/html/index.html
    ```

# Testing

Follow these steps to run tests and static code analysis in your clone of this repository.

1. Follow the "Source Installation" instructions in the [installation tutorial](https://gazebosim.org/api/physics/8/installation.html).

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
gz-physics
├── bullet                    Files for bullet plugin component.
├── bullet-featherstone       Files for bullet-featherstone plugin component.
├── dartsim                   Files for dartsim plugin component.
├── examples                  Examples about how to use the library.
├── heightmap                 Heightmap related header files.
├── include/gz/physics        Header files.
├── mesh                      Files for mesh component.
├── sdf                       Files for sdf component.
├── src                       Source files and unit tests.
├── test
│    ├── benchmark            Benchmark tests.
│    ├── common_test          Tests common to multiple physics plugins.
│    ├── include              Header files for tests.
│    ├── integration          Integration tests.
│    ├── performance          Performance tests.
│    ├── plugins              Plugins used in tests.
│    ├── regression           Regression tests.
│    ├── resources            Models and mesh resource files.
│    └── static_assert        Tests involving compilation failures.
├── tpe
│    ├── lib                  Implementation of TPE engine.
│    └── plugin               Files for TPE plugin component.
├── tutorials                 Tutorials, written in markdown.
├── Changelog.md              Changelog.
└── CMakeLists.txt            CMake build script.
```
# Contributing

Please see the [contribution guide](https://gazebosim.org/docs/all/contributing).

# Code of Conduct

Please see
[CODE\_OF\_CONDUCT.md](https://github.com/gazebosim/gz-sim/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Gazebo project](https://gazebosim.org) which periodically releases a versioned set of compatible and complementary libraries. See the [Gazebo website](https://gazebosim.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-physics/blob/gz-physics9/LICENSE) file.
