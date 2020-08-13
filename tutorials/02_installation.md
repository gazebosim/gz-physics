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
sudo apt-get install libignition-physics3-dev
```

**Build from source**

1. Run the following to install dependencies

    ```{.sh}
    sudo apt-add-repository -s "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -c -s) main"
    sudo apt-get build-dep -y ignition-physics3
    ```

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
    git clone https://github.com/ignitionrobotics/ign-physics -b master
    ```

4. Configure and build

    ```
    cd ign-physics; mkdir build; cd build; cmake ..; make
    ```

5. Optionally, install Ignition Physics

    ```
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
brew install ignition-physics3
```

**Build from source using Homebrew**

Run the following to install dependencies
```{.sh}
brew tap osrf/simulation
brew install ignition-physics3 --only-dependencies
```

Clone the ign-physics repository from GitHub
```{.sh}
git clone https://github.com/ignitionrobotics/ign-physics -b master
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
