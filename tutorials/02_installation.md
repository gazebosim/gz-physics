\page installation Installation

This tutorial describes how to install Ignition Physics via either a binary distribution or from source.

## Ubuntu

### Prerequisites

Ignition Physics uses several c++17 features which are not available in the
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

### Binary Install

```
sudo apt-get update
sudo apt-get install libignition-physics<#>-dev
```

Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

### Source Install

1. Run the following to install dependencies
    ```
    sudo apt-add-repository -s "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -c -s) main"
    sudo apt-get build-dep -y ignition-physics<#>-dev
    ```
    Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.


2. Use gcc-8:
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
    cd ign-physics; mkdir build; cd build; cmake ..; make
    ```

5. Optionally, install Ignition Physics
    ```
    sudo make install
    ```

## macOS

### Prerequisites

Ignition Physics and several of its dependencies can be installed on macOS
with [Homebrew](http://brew.sh/) using the [osrf/simulation
tap](https://github.com/osrf/homebrew-simulation). Ignition Physics uses
several c++17 features which are not available in macOS High Sierra (10.13)
and earlier, so macOS Mojave (10.14) with XCode 10.1 are the minimum
system requirements.

### Install Binaries using Homebrew

Install homebrew, which should also prompt you to install the XCode
command-line tools:

```
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

Run the following commands:

```
brew tap osrf/simulation
brew install ignition-physics<#>
```
Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.


### Build from source using Homebrew

Run the following to install dependencies
```
brew tap osrf/simulation
brew install ignition-physics<#> --only-dependencies
```

Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

Clone the ign-physics repository from GitHub
```
git clone https://github.com/ignitionrobotics/ign-physics -b ign-physics<#>
```
Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

Then build using CMake
```
cd ign-physics
mkdir build
cd build
cmake ..
make
sudo make install
```

## Windows

### Prerequisites

First, follow the [ign-cmake](https://github.com/ignitionrobotics/ign-cmake) tutorial for installing Conda, Visual Studio, CMake, etc., prerequisites, and creating a Conda environment.

Navigate to ``condabin`` if necessary to use the ``conda`` command (i.e., if Conda is not in your `PATH` environment variable. You can find the location of ``condabin`` in Anaconda Prompt, ``where conda``).

Create if necessary, and activate a Conda environment:

```
conda create -n ign-ws
conda activate ign-ws
```

### Binary Install

```
conda install libignition-physics<#> --channel conda-forge
```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

### Source Install

1. Install Ignition dependencies:

You can view available versions and their dependencies:

```
conda search libignition-physics* --channel conda-forge --info
```

Install dependencies, replacing `<#>` with the desired versions:

```
conda install libignition-cmake<#> libignition-common<#> libignition-math<#> libignition-plugin<#> libsdformat<#> --channel conda-forge
```

1. Navigate to where you would like to build the library, and clone the repository.

    ```
    # Optionally, append `-b ign-physics#` (replace # with a number) to check out a specific version
    git clone https://github.com/ignitionrobotics/ign-physics.git
    ```

1. Configure and build

    ```
    cd ign-physics
    mkdir build
    cd build
    cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
    cmake --build . --config Release
    ```

1. Optionally, install

    ```
    cmake --install . --config Release
    ```
