\page installation Installation

These instructions are for installing only Ignition Physics.
If you're interested in using all the Ignition libraries, check out this [Ignition installation](https://ignitionrobotics.org/docs/dome/install).

We recommend following the Binary Installation instructions to get up and running as quickly and painlessly as possible.

The Source Installation instructions are generally recommended for developers who want access to the latest features, develop your own feature or make a contribution to our [codebase](https://github.com/ignitionrobotics/ign-physics/).

# Ubuntu

## Prerequisites

Ignition Physics uses several C++17 features which are not available in the
version of gcc supplied with Ubuntu Xenial, so Ubuntu Bionic or later
is required.

If you don't already have the `lsb-release` package installed, please do so now:
```
sudo apt-get update
sudo apt-get install lsb-release
```

Setup your computer to accept software from
[packages.osrfoundation.org](http://packages.osrfoundation.org):
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

## Binary Installation

On Ubuntu systems, `apt-get` can be used to install `ignition-plugin`:
```
sudo apt-get update
sudo apt-get install libignition-physics<#>-dev
```
Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

## Source Installation

1. Install dependencies
  ```
  sudo apt-add-repository -s "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -c -s) main"
  sudo apt-get build-dep -y ignition-physics<#>-dev
  ```
  Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

2. Use gcc-8
  ```
  sudo apt update
  sudo apt-get -y install g++-8
  sudo update-alternatives --install \
    /usr/bin/gcc gcc /usr/bin/gcc-8 800 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-8 \
    --slave /usr/bin/gcov gcov /usr/bin/gcov-8
  ```

3. Clone the repository
  ```
  git clone https://github.com/ignitionrobotics/ign-physics -b ign-physics<#>
  ```
  Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

4. Configure and build
  ```
  cd ign-physics
  mkdir build
  cd build
  cmake ..
  make
  ```

5. Optionally, install
  ```
  sudo make install
  ```

# macOS

## Prerequisites

Ignition Physics and several of its dependencies can be installed on macOS
with [Homebrew](http://brew.sh/) using the [osrf/simulation
tap](https://github.com/osrf/homebrew-simulation). Ignition Physics uses
several C++17 features which are not available in macOS High Sierra (10.13)
and earlier, so macOS Mojave (10.14) with XCode 10.1 are the minimum
system requirements.

## Binary Installation

1. Install Homebrew, which should also prompt you to install the XCode
command-line tools:
  ```
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  ```

2. Run the following commands
  ```
  brew tap osrf/simulation
  brew install ignition-physics<#>
  ```
Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

## Source Installation

1. Install dependencies
  ```
  brew tap osrf/simulation
  brew install ignition-physics<#> --only-dependencies
  ```

2. Clone the repository
  ```
  git clone https://github.com/ignitionrobotics/ign-physics -b ign-physics<#>
  ```
Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

3. Configure and build
  ```
  cd ign-physics
  mkdir build
  cd build
  cmake ..
  make
  ```

4. Optionally, install
  ```
  sudo make install
  ```

# Windows

Currently, on Windows, DART is not supported, pending resolution of [dartsim/dart#1522](https://github.com/dartsim/dart/issues/1522) and [conda-forge/dartsim-feedstock#2](https://github.com/conda-forge/dartsim-feedstock/issues/2).
Only [Trivial Physics Engine (TPE)](https://community.gazebosim.org/t/announcing-new-physics-engine-tpe-trivial-physics-engine/629) is supported at the moment.

## Prerequisites

First, follow the [ign-cmake](https://github.com/ignitionrobotics/ign-cmake) tutorial for installing Conda, Visual Studio, CMake, and other prerequisites, and also for creating a Conda environment.

Navigate to ``condabin`` if necessary to use the ``conda`` command (i.e., if Conda is not in your `PATH` environment variable. You can find the location of ``condabin`` in Anaconda Prompt, ``where conda``).

Create if necessary, and activate a Conda environment:
```
conda create -n ign-ws
conda activate ign-ws
```

## Binary Installation

```
conda install libignition-physics<#> --channel conda-forge
```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

## Source Installation

This assumes you have created and activated a Conda environment while installing the Prerequisites.

1. Install Ignition dependencies:

  You can view available versions and their dependencies:
  ```
  conda search libignition-physics* --channel conda-forge --info
  ```

  Install dependencies, replacing `<#>` with the desired versions:
  ```
  conda install libignition-cmake<#> libignition-common<#> libignition-math<#> libignition-plugin<#> libsdformat<#> --channel conda-forge
  ```

2. Navigate to where you would like to build the library, and clone the repository.
  ```
  # Optionally, append `-b ign-physics#` (replace # with a number) to check out a specific version
  git clone https://github.com/ignitionrobotics/ign-physics.git
  ```

3. Configure and build
  ```
  cd ign-physics
  mkdir build
  cd build
  cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
  cmake --build . --config Release
  ```

4. Optionally, install
  ```
  cmake --install . --config Release
  ```

# Documentation

API and tutorials can be found at [https://ignitionrobotics.org/libs/physics](https://ignitionrobotics.org/libs/physics).

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen. On Ubuntu Doxygen can be installed using
  ```
  sudo apt-get install doxygen
  ```

2. Clone the repository
  ```
  git clone https://github.com/ignitionrobotics/ign-physics -b ign-physics<#>
  ```

3. Configure and build the documentation.
  ```
  cd ign-physics
  mkdir build
  cd build
  cmake ..
  make doc
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
