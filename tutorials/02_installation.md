\page installation Installation

These instructions are for installing only Gazebo Physics.
If you're interested in using all the Gazebo libraries, check out this [Gazebo installation](https://gazebosim.org/docs/dome/install).

We recommend following the Binary Installation instructions to get up and running as quickly and painlessly as possible.

The Source Installation instructions are generally recommended for developers who want access to the latest features, develop your own feature or make a contribution to our [codebase](https://github.com/gazebosim/gz-physics/).

# Ubuntu

## Prerequisites

Ubuntu Focal or later.

If you don't already have the packages `gnupg`, `lsb-release`, or `wget` installed, please do so now:
```
sudo apt-get update
sudo apt-get install gnupg lsb-release wget
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

On Ubuntu systems, `apt-get` can be used to install `gz-plugin`:
```
sudo apt-get update
sudo apt-get install libgz-physics<#>-dev
```
Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

## Source Installation

1. Install dependencies
  ```
  sudo apt-add-repository -s "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -c -s) main"
  sudo apt-get build-dep -y libgz-physics<#>-dev
  ```
  Be sure to replace `<#>` with a number value, such as `5` or `6`, depending on which version you need.
  From version `6` onward, you should use `libgz-physics<#>-dev`; for lower versions, `libignition-physics<#>-dev`.

2. Clone the repository
  ```
  git clone https://github.com/gazebosim/gz-physics -b gz-physics<#>
  ```
  Be sure to replace `<#>` with a number value, such as `5` or `6`, depending on which version you need.
  From version `6` onward, you should use `gz-physics<#>`; for lower versions, `ign-physics<#>`.

3. Configure and build
  ```
  cd gz-physics
  mkdir build
  cd build
  cmake ..
  make
  ```

4. Optionally, install
  ```
  sudo make install
  ```

# macOS

## Prerequisites

Gazebo Physics and several of its dependencies can be installed on macOS
with [Homebrew](http://brew.sh/) using the [osrf/simulation
tap](https://github.com/osrf/homebrew-simulation). Gazebo Physics uses
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
  brew install gz-physics<#>
  ```
Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

## Source Installation

1. Install dependencies
  ```
  brew tap osrf/simulation
  brew install gz-physics<#> --only-dependencies
  ```

2. Clone the repository
  ```
  git clone https://github.com/gazebosim/gz-physics -b ign-physics<#>
  ```
Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

3. Configure and build
  ```
  cd gz-physics
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

First, follow the [gz-cmake](https://github.com/gazebosim/gz-cmake) tutorial for installing Conda, Visual Studio, CMake, and other prerequisites, and also for creating a Conda environment.

Navigate to ``condabin`` if necessary to use the ``conda`` command (i.e., if Conda is not in your `PATH` environment variable. You can find the location of ``condabin`` in Anaconda Prompt, ``where conda``).

Create if necessary, and activate a Conda environment:
```
conda create -n gz-ws
conda activate gz-ws
```

## Binary Installation

```
conda install libgz-physics<#> --channel conda-forge
```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

## Source Installation

This assumes you have created and activated a Conda environment while installing the Prerequisites.

1. Install Gazebo dependencies:

  You can view available versions and their dependencies:
  ```
  conda search libgz-physics* --channel conda-forge --info
  ```

  Install dependencies, replacing `<#>` with the desired versions:
  ```
  conda install libgz-cmake<#> libgz-common<#> libgz-math<#> libgz-plugin<#> libsdformat<#> --channel conda-forge
  ```

2. Navigate to where you would like to build the library, and clone the repository.
  ```
  # Optionally, append `-b ign-physics#` (replace # with a number) to check out a specific version
  git clone https://github.com/gazebosim/gz-physics.git
  ```

3. Configure and build
  ```
  cd gz-physics
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

API and tutorials can be found at [https://gazebosim.org/libs/physics](https://gazebosim.org/libs/physics).

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen. On Ubuntu Doxygen can be installed using
  ```
  sudo apt-get install doxygen
  ```

2. Clone the repository
  ```
  git clone https://github.com/gazebosim/gz-physics -b ign-physics<#>
  ```

3. Configure and build the documentation.
  ```
  cd gz-physics
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

  Each test is registered with `ctest`. These can then be filtered with the `ctest` command line.

  ```
  # See a list of all available tests
  ctest -N

  # Run all tests in dartsim (verbose)
  ctest -R dartsim -V

  # Run all INTEGRATION tests (verbose)
  ctest -R INTEGRATION -V

  # Run all COMMON tests (verbose)
  ctest -R COMMON -V
  ```

3. You will need Cppcheck in order to run static code checks. On Ubuntu Cppcheck can be installed using
  ```
  sudo apt-get install cppcheck
  ```

4. Configure and run the static code checker.
  ```
  cd gz-physics
  mkdir build
  cd build
  cmake ..
  make codecheck
  ```
