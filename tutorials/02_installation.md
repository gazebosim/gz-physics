\page installation Installation

This tutorial describes how to install Ignition Physics on Ubuntu Linux and macOS via either a binary distribution or from source. Support for Windows is coming soon.

## Ubuntu

Ignition Physics uses several c++17 features which are not available in the
version of gcc supplied with Ubuntu Xenial, so Ubuntu Bionic or later
is required.

If you don't already have the `lsb-release` package installed, please do so now:
```{.sh}
sudo apt-get update
sudo apt-get install lsb-release
```

Setup your computer to accept software from
[packages.osrfoundation.org](http://packages.osrfoundation.org):

```{.sh}
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:

```{.sh}
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

**Install Ignition Physics binaries**

```{.sh}
sudo apt-get update
sudo apt-get install libignition-physics-dev
```

**Build from source**

Run the following to install dependencies
```{.sh}
sudo apt-get install libignition-cmake2-dev \
                     libignition-common3-dev \
                     libignition-common3-graphics-dev \
                     libignition-math6-dev \
                     libignition-math6-eigen3-dev \
                     libignition-plugin-dev \
                     libdart6-dev \
                     libdart6-collision-ode-dev \
                     libdart6-utils-urdf-dev \
                     libsdformat9-dev
```

Clone the ign-physics repository from bitbucket
```{.sh}
hg clone https://bitbucket.org/ignitionrobotics/ign-physics
```

Then build using CMake
```{.sh}
cd ign-physics
mkdir build
cd build
cmake ..
make
sudo make install
```

## macOS

Ignition Physics and several of its dependencies can be installed on macOS
with [Homebrew](http://brew.sh/) using the [osrf/simulation
tap](https://github.com/osrf/homebrew-simulation). Ignition Physics uses
several c++17 features which are not available in macOS High Sierra (10.13)
and earlier, so macOS Mojave (10.14) with XCode 10.1 are the minimum
system requirements.

**Install Binaries using Homebrew**

Install homebrew, which should also prompt you to install the XCode
command-line tools:

```{.sh}
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

Run the following commands:

```{.sh}
brew tap osrf/simulation
brew install ignition-physics1
```

**Build from source using Homebrew**

Run the following to install dependencies
```{.sh}
brew tap osrf/simulation
brew install ignition-physics1 --only-dependencies
```

Clone the ign-physics repository from bitbucket
```{.sh}
hg clone https://bitbucket.org/ignitionrobotics/ign-physics
```

Then build using CMake
```{.sh}
cd ign-physics
mkdir build
cd build
cmake ..
make
sudo make install
```
