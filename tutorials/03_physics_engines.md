# Understanding Physics Plugin

This is an introduction to different physics engines and how they are integrated into the Ignition Physics library.

## Ignition Physics

``ign-physics`` library integrates external physics engines into the Ignition Simulation eco-system.
It allows users to select from multiple supported physics engines based on their simulation needs.
Its plugin interface loads physics engines with requested features at runtime.
It is also possible to integrate your own selected physics engine by writing a compatible plugin interface.

To get a more in-depth understanding of how `physics` works in Ignition, we will start with some high level concepts and definitions.
<!-- TODO: add tutorial on how to write your own physics plugin -->

### High Level Concept

Conceptually, the physics plugin can be viewed as two sides of interface: user vs. implementation.

Each physics engine provides access to different features implemented by the ignition physics engine. 
The interface is made possible through ``ignition-plugin`` library, which instantiates ``Feature``s in ``ign-physics`` engine’s ``FeatureList``s and supplies pointer to the selected engine. 
This "user side interface" makes ``ign-physics`` "callable" from other Ignition libraries.

The implementation side interface handles specific implementation of each ``Feature``.
Depending on what external physics engine we are using (DART, TPE, Bullet etc.), the interface might be different.
This interface is more internal facing as in used mostly inside ``ign-physics`` library. 

The implementation of physics plugin revolves around four key elements.

1. Entity

    This is the base class of all "proxy objects".
    The "proxy objects" are essentially interfaces into the actual objects which exist inside of the various physics engine implementations.
    The proxy objects contain the minimal amount of data (e.g. a unique identifier, a reference-counter for the implementation object, and a reference to the implementation interface that it needs) necessary to interface with the object inside of the implementation that it refers to.

2. FeaturePolicy

    FeaturePolicy is a "policy class" used to provide metadata to features about what kind of simulation engine they are going to be used in.
    Many physics simulations software libraries model 3-dimensional systems, though some (like Box2d) only consider 2-dimensional systems.
    A FeaturePolicy is used to customize ignition-physics API’s by the number of dimensions (2 or 3) and also the floating point scalar type (float or double).
    Dartsim and TPE reference implementations both use FeaturePolicy3d (3 dimensions, double).

3. Feature

    This class defines the concept of a ``Feature``, examples like ``GetEngineFromWorld``, ``GetEngineInfo`` etc.
    There is a pre-defined list of features in Ignition Physics. 
    They are implemented by using external physcis engine API to fulfill simulation needs requested by Ignition. 

4. FeatureList

    This is the class that aggregates a list of features.
    FeatureLists can be constructed in hierarchies, e.g. a ``FeatureList`` can be passed into another ``FeatureList``, and the set of all features in the new list will be the sum.

### Dart vs. Trivial Physics Engine (TPE)

<!-- TODO: add Bullet once it's supported -->
<!-- ### Bullet -->

Dart ([Dynamic Animation and Robotics Toolkit](https://dartsim.github.io/)) is an open source library that provides data structures and algorithms for kinematic and dynamic applications in robotics and computer animation.
It is the default physics engine used in Ignition Simulation.
The plugin interface for DART is called ``Dartsim`` in Ignition Physics.

TPE ([Trivial Physics Engine](<!-- TODO: add repo link -->)) is an open source library createtd by Open Robotics that enables fast, inexpensive kinematics simulation for entities at large scale.
It supports higher-order fleet dynamics without real physics (eg. gravity, force, contraint etc.) and multi-machine sychronization.
The plugin interface for TPE is called ``TPE-Plugin`` in Ignition Physics.

The following is a list of features supported by each physics engine to help user select one that fits their needs.

#### Entity Comparison

The following is a table of ``Entity`` names used in ``ign-physcis`` plugin interface, Dart and TPE. Entities are arranged in top-down hierarchical order.

| Physics Plugin | Dart  | TPE  |
|:-:|:-:|:-:|
| Engine  | Engine  | Engine  |
| World  | World  | World  |
| Frame  | Frame  | N/A  |
| Model  | Skeleton  | Model |
| Joint  | Joint  | N/A |
| Link  | Node  | Link  |
| Shape  | Shape  | Collision |
| Box/Sphere/Cylinder etc. | Box/Sphere/Cylinder etc.  | Shape |
|   |   |   |

#### Feature Comparison

The following is a table of implemented ``FeatureList``s of Dartsim and TPE-Plugin.

| Dartsim  | TPE-Plugin  |
|:-:|:-:|
| Base  | Base  |
| CustomFeatures  | CustomFeatures  |
| EntityManagementFeatures  | EntityManagementFeatures  |
| FreeGroupFeatures  | FreeGroupFeatures  |
| JointFeatures  | N/A  |
| KinematicsFeatures  | KinematicsFeatures  |
| LinkFeatures  | N/A  |
| SDFFeatures  | SDFFeatures  |
| ShapeFeatures  | ShapeFeatures   |
| SimulationFeatures  | SimulationFeatures  |

#### Feature Definition

| Name  | Definition  |
|---|---|
| Base  | contains data structures and functions that define and use "proxy objects"   |
| CustomFeatures  | retrieves ``World`` entity from physics engine|
| EntityManagementFeatures  | provides features to get, remove and contruct entities  |
| FreeGroupFeatures  | finds free group entities and sets world pose, linear and angular velocities  |
| JointFeatures  | defines types of joints used and sets joint properties  |
| KinematicsFeatures  | computes frame relative to world  |
| LinkFeatures  | applies external force and torque to link  |
| SDFFeatures  | contructs entities from sdf file  |
| ShapeFeatures  | retrieves ``Shape`` related properties like ``BoundingBox``, ``ShapeSize`` etc. |
| SimulationFeatures  | updates ``World`` and everything within by defined stepsize |
|   |   |
