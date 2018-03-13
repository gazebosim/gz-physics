# Ignition Physics 

** Ignition Physics classes and functions for robot applications.**

Ignition Physics is a component in the ignition framework, a set
of libraries designed to rapidly develop robot applications.
  
  [http://ignitionrobotics.org](http://ignitionrobotics.org)

## Installation

Standard installation can be performed in UNIX systems using the following 
steps:

 - mkdir build/
 - cd build/
 - cmake ..
 - sudo make install

## Uninstallation 

To uninstall the software installed with the previous steps:

 - cd build/
 - sudo make uninstall

## Create Documentation & Release

1. Build documentation

```
cd build
make doc
```

1. Upload documentation to ignitionrobotics.org.

```
cd build
sh upload.sh
```

1. If you're creating a new release, then tell ignitionrobotics.org about
   the new version. For example:

```
curl -k -X POST -d '{"libName":"common", "version":"1.0.0", "releaseDate":"2017-10-09T12:10:13+02:00","password":"secret"}' https://api.ignitionrobotics.org/1.0/versions
```

