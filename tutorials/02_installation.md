\page installation Installation

This tutorial describes how to install Ignition Physics on [Ubuntu Linux](#ubuntu_install) and [macOS](#macos_install) via either a binary distribution or from source. Support for Windows is coming soon.


## Ubuntu ## {#ubuntu_install}


Setup your computer to accept software from
*packages.osrfoundation.org*:

```{.sh}
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:

```{.sh}
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install Ignition Physics

```
sudo apt-get update
sudo apt-get install libignition-physics-dev
```

**Build from source**

Run the following to install dependencies
```
sudo apt-get install libignition-cmake2-dev, libignition-common3-dev, libignition-common3-graphics-dev,  libignition-math6-dev, libignition-math6-eigen3-dev, libignition-plugin-dev, libdart6-dev, libdart6-collision-ode-dev, libdart6-utils-urdf-dev, libsdformat8-dev
```

Clone the ign-physics repository from bitbucket
```
hg clone https://bitbucket.org/ignitionrobotics/ign-physics
```

Then build using CMake
```
cd ign-physics
mkdir build
cd build
cmake ..
make
sudo make install
```

## macOS ## {#macos_install}

Ignition Physics and several of its dependencies can be installed on macOS
with [Homebrew](http://brew.sh/) using the [osrf/simulation
tap](https://github.com/osrf/homebrew-simulation). Ignition Physics has
been tested on macOS Mojave (10.14) with AppleClang 10.0.

**Install Binaries using Homebrew**

Install homebrew, which should also prompt you to install the XCode
command-line tools:

```
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

Run the following commands:

```
brew tap osrf/simulation
brew install ignition-physics1
```

**Build from source using Homebrew**

Run the following to install dependencies
```
brew tap osrf/simulation
brew install ignition-physics1 --only-dependences
```

Clone the ign-physics repository from bitbucket
```
hg clone https://bitbucket.org/ignitionrobotics/ign-physics
```

Then build using CMake
```
cd ign-physics
mkdir build
cd build
cmake ..
make
sudo make install
```
