\page physicsplugin Understanding the Physics Plugin

The \ref gz::physics "Gazebo Physics" library integrates physics engines into the Gazebo Simulation eco-system.
Each physics engine is wrapped into a Gazebo Physics Plugin that can be loaded in a Gazebo simulation.
The Physics Plugin interface loads physics engines with the requested features at runtime.
It is also possible to integrate your own selected physics engine by writing a compatible plugin interface.

To get a more in-depth understanding of how the physics plugin works in Gazebo, we will start with some high level concepts and definitions.

<!-- TODO: add tutorial on how to write your own physics plugin -->

## High Level Concept

Conceptually, the physics plugin can be viewed from two sides of its interface: user vs. implementation.

Each physics engine provides access to different features implemented by the Gazebo Physics engine.
The interface is made possible through the \ref gz::plugin "Gazebo Plugin" library, which instantiates \ref gz::physics::Feature "Features" in \ref gz::physics::FeatureList "FeatureLists" and supplies pointers to the selected engine.
This "user side interface" makes the Gazebo Physics library "callable" from other Gazebo libraries.

The implementation side interface handles specific implementations of each `Feature`.
Depending on what physics engine we are using in the plugin (DART, TPE etc.), the interface might be different.
This interface is more internal facing, i.e. used mostly inside the Gazebo Physics library.

The implementation of the physics plugin revolves around four key elements.

1. \ref gz::physics::Entity "Entity"

    This is the base class of all "proxy objects".
    The "proxy objects" are essentially interfaces into the actual objects which exist inside of the various physics engine implementations.
    The proxy objects contain the minimal amount of data (e.g. a unique identifier, a reference-counter for the implementation object, and a reference to the implementation interface that it needs) necessary to interface with the object inside of the implementation that it refers to.

2. \ref gz::physics::FeaturePolicy "FeaturePolicy"

    FeaturePolicy is a "policy class" used to provide metadata to features about what kind of simulation engine they are going to be used in.
    Many physics simulations software libraries model 3-dimensional systems, though some (like Box2d) only consider 2-dimensional systems.
    A FeaturePolicy is used to customize Gazebo Physics' APIs by the number of dimensions (2 or 3) and also the floating point scalar type (float or double).
    (All of the currently supported physics engines use FeaturePolicy3d i.e. 3 dimensions and double.)

3. \ref gz::physics::Feature "Feature"

    This class defines the concept of a `Feature`, examples like `GetWorldFromEngine`, \ref gz::physics::GetEngineInfo "GetEngineInfo" etc.
    There is a pre-defined list of features in Gazebo Physics.
    They are implemented by using external physics engines' APIs to fulfill simulation needs requested by Gazebo.

4. \ref gz::physics::FeatureList "FeatureList"

    This is the class that aggregates a list of features.
    FeatureLists can be constructed in hierarchies, e.g. a `FeatureList` can be passed into another `FeatureList`, and the set of all features in the new list will be the sum.


## FeatureList Organization

For example, here are the `FeatureLists` used in the implementation of the `Dartsim` physics engine.
Plugin implementations for other engines may choose to organize features into `FeatureLists` differently.

| Name  | Definition  |
|---|---|
| Base  | contains data structures and functions that define and use "proxy objects"   |
| CustomFeatures  | retrieves `World` entity from physics engine|
| EntityManagementFeatures  | provides features to get, remove and construct entities  |
| FreeGroupFeatures  | finds free group entities and sets world pose, linear and angular velocities  |
| JointFeatures  | defines types of joints used and sets joint properties  |
| KinematicsFeatures  | computes frame relative to world  |
| LinkFeatures  | applies external force and torque to link  |
| SDFFeatures  | constructs entities from SDF file  |
| ShapeFeatures  | retrieves `Shape` related properties like `BoundingBox`, `ShapeSize` etc. |
| SimulationFeatures  | updates `World` and everything within by defined stepsize |
| WorldFeatures  | sets options like solver and collision detector  |

## Available Physics Plugins

Dart ([Dynamic Animation and Robotics Toolkit](https://dartsim.github.io/)) is an open source library that provides data structures and algorithms for kinematic and dynamic applications in robotics and computer animation.
It is the default physics engine used in Gazebo Simulation.
The source code for Dartsim plugin can be found in [Gazebo Physics repository](https://github.com/gazebosim/gz-physics/tree/main) under `dartsim` directory.

TPE ([Trivial Physics Engine](https://github.com/gazebosim/gz-physics/tree/main/tpe)) is an open source library created by Open Robotics that enables fast, inexpensive kinematics simulation for entities at large scale.
It supports higher-order fleet dynamics without real physics (eg. gravity, force, constraint etc.) and multi-machine synchronization.
The source code for TPE plugin can be found in [Gazebo Physics repository](https://github.com/gazebosim/gz-physics/tree/main) under the `tpe/plugin` directory.

Bullet ([Bullet3 Physics Engine](https://github.com/bulletphysics/bullet3)) is an open source library that supports real-time collision detection and multi-physics simulation for robotics and other application domains.
Since Bullet supports two different APIs - a rigid-body API and a multibody API - with different physics implementations, there are two available plugin implementations:
- The `bullet` plugin implements the rigid-body API, and the source code can be found in [Gazebo Physics repository](https://github.com/gazebosim/gz-physics/tree/main) under the `bullet` directory.
- The `bullet-featherstone` plugin implements the multibody API (based on Featherstone's algorithms), and the source code can be found in [Gazebo Physics repository](https://github.com/gazebosim/gz-physics/tree/main) under the `bullet-featherstone` directory.

### Entity Comparison

The following is a table of `Entity` names used in the Gazebo Physics plugin interface, mapped to corresonding types in each supported physics engine.
Entities are arranged in top-down hierarchical order.

| Physics Plugin | Dart  | TPE | Bullet | Bullet Featherstone |
|:-:|:-:|:-:|:-:|:-:|
| Engine  | Engine  | Engine  | Engine  | Engine  |
| World  | World  | World  | btDiscreteDynamicsWorld | btMultiBodyDynamicsWorld |
| Frame  | Frame  | N/A  | N/A<sup>1</sup> | N/A<sup>1</sup> |
| Model  | Skeleton  | Model | N/A<sup>2</sup> | btMultiBody |
| Joint  | Joint  | N/A | btTypedConstraint | btMultiBodyJoint<sup>3</sup> |
| Link  | BodyNode  | Link | btRigidBody | btMultiBodyLink |
| Shape  | Shape  | Collision | btCollisionShape | btCollisionShape |
| Box/Sphere/Cylinder etc. | Box/Sphere/Cylinder etc. | Box/Sphere/Cylinder etc. | Box/Sphere/Cylinder etc. | Box/Sphere/Cylinder etc. |

<sup>1</sup> Frames are implicitly attached to joints, links, collisions or models in the Bullet physics engine.

<sup>2</sup> The Bullet rigid-body API does not have a concept of a Model, but the plugin maintains a collection of Links and Joints in the engine associated with a Model.

<sup>3</sup> There are multiple types in the Bullet Featherstone API to interact with joints, such as btMultiBodyJointLimitConstraint, btMultiBodyJointMotor and btMultiBodyJointFeedback.

### Feature Comparison

For a list of all available `Features` in the Gazebo Physics library, check the classes inherited from `Feature` in the [Gazebo Physics API](https://gazebosim.org/api/physics/8/hierarchy.html).
To check if a physics plugin implements a particular `Feature`, check the `FeatureLists` supported by that plugin as specified in the plugin.cc file, for example, [dartsim/src/plugin.cc](https://github.com/gazebosim/gz-physics/blob/main/dartsim/src/plugin.cc).

Next, check out the tutorial on \ref pluginloading "Loading physics plugins" on how to load a plugin and access a specific `Feature` interface of the plugin programatically.
e