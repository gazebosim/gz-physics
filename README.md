# Gazebo Physics : Physics classes and functions for robot applications

**Maintainer:** scpeters AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/gz-physics.svg)](https://github.com/gazebosim/gz-physics/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/gz-physics.svg)](https://github.com/gazebosim/gz-physics/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/gz-physics/branch/ign-physics2/graph/badge.svg)](https://codecov.io/gh/gazebosim/gz-physics/branch/ign-physics2)
Ubuntu Focal | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_physics-ci-ign-physics2-focal-amd64)](https://build.osrfoundation.org/job/ignition_physics-ci-ign-physics2-focal-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_physics-ci-ign-physics2-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_physics-ci-ign-physics2-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ign_physics-ci-win)](https://build.osrfoundation.org/job/ign_physics-ci-win)

Gazebo Physics, a component of [Gazebo](https://gazebosim.org), provides an abstract physics interface
designed to support simulation and rapid development of robot applications.

# Table of Contents

[Motivation](#motivation)

[Features](#features)

[Install](#install)

* [Binary Install](#binary-install)

* [Source Install](#source-install)

    * [Prerequisites](#prerequisites)

    * [Building from Source](#building-from-source)

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
    - Construct model from SDFormat file.
    - Collision shapes (such as box, sphere, cylinder, mesh, heightmap).
    - Joint types (such as revolute, prismatic, fixed, ball, screw, universal).
    - Step simulation, get/set state, apply inputs.
* Reference implementation of physics plugin using
  [dartsim](http://dartsim.github.io/).
* CompositeData structures for efficiently using native types in API.

# Install

We recommend following the [Binary Install](#binary-install) instructions to get up and running as quickly and painlessly as possible.

The [Source Install](#source-install) instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Binary Install

On Ubuntu systems, `apt-get` can be used to install `ignition-physics2`:

```
sudo apt install libignition-physics2-dev
```

## Source Install

Source installation can be performed in UNIX systems by first installing the
necessary prerequisites followed by building from source.

### Prerequisites

Install required dependencies:

~~~
sudo apt update
sudo apt-get -y install lsb-release
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
sudo apt-add-repository -s "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -c -s) main"
sudo apt-get build-dep -y ignition-physics2
~~~

Use gcc-8:

~~~
sudo apt update
sudo apt-get -y install g++-8
sudo update-alternatives --install \
  /usr/bin/gcc gcc /usr/bin/gcc-8 800 \
  --slave /usr/bin/g++ g++ /usr/bin/g++-8 \
  --slave /usr/bin/gcov gcov /usr/bin/gcov-8
~~~

### Building from source

1. Clone the repository

    ```
    git clone https://github.com/gazebosim/gz-physics -b ign-physics2
    ```

2. Configure and build

    ```
    cd gz-physics; mkdir build; cd build; cmake ..; make
    ```

3. Optionally, install Gazebo Physics

    ```
    sudo make install
    ```

# Usage

Please refer to the [examples directory](https://github.com/gazebosim/gz-physics/raw/ign-physics2/examples/).

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
gz-physics
├── dartsim                   Files for dartsim plugin component.
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

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Gazebo project](https://gazebosim.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Gazebo website](https://gazebosim.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-physics/blob/main/LICENSE) file.
