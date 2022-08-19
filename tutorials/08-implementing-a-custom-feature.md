\page createcustomfeature Implement a custom feature

## Prerequisites

- \ref installation "Installation"
- \ref physicsplugin "Understanding the physics plugin"
- \ref pluginloading "Loading physics plugins"
- \ref createphysicsplugin "Implement a physics feature"

## Implement a custom feature in DART plugin

In the last \ref createphysicsplugin "Implement a physics feature" tutorial, we
know how to implement a dummy physics engine as a plugin and load it using
\ref gz::physics "Ignition Physics API". In this tutorial, we will look
deeper into the structure of a physics engine plugin, for example, the available
[DART](https://github.com/ignitionrobotics/ign-physics/tree/ign-physics2/dartsim)
physics engine in `ign-physics` repository and how to define a custom
\ref gz::physics::Feature "Feature" for the plugin.

### Folder structure of the plugins

Below is the general structure of the `ign-physics` repository:

```
ign-physics
├── dartsim                   Files for dartsim plugin component.
├── tpe                       Files for tpe plugin component.
├── include/ignition/physics  Header files.
├── mesh                      Files for mesh component.
├── resources                 Model and mesh resource files used by tests.
├── sdf                       Files for sdf component.
├── src                       Source files and unit tests.
├── test
├── tutorials                 Tutorials, written in markdown.
├── Changelog.md              Changelog.
└── CMakeLists.txt            CMake build script.
```

As shown above, there are two physics engines available:
- **DART**: `ignition-physics-dartsim-plugin`.
- **TPE**: `ignition-physics-tpe-plugin`.

and their plugin folders are placed just below the top level of `ign-physics`.

Looking closer to a plugin folder, for example, the `dartsim` (DART) plugin:

```
dartsim
├── worlds                            Example SDF files for testing dartsim plugin functionalities.
├── src                               Main implementation files of the plugin features interfacing the physics engines API
├── include/ignition/physics/dartsim  Header files for the plugin features.
└── CMakeLists.txt                    CMake plugin build script.
```

Basically, new implementation of \ref gz::physics::Feature "Feature" or
\ref gz::physics::FeatureList "FeatureList", which is corresponded to a
functionality of the external physics engine can be defined as a header in
`include/ignition/physics/<plugin_name>` folder. The custom feature could
be added in a \ref gz::physics::FeatureList "FeatureList"
and implemented its functionalities in `src` folder.

See the \ref physicsplugin "Understanding the physics plugin" tutorial for details on physics engines.

### Plugin and feature requirements

In general, the minimum set of features that any physics engine plugin must
implement to be supported by Ignition Gazebo is as below:
- \ref gz::physics::FindFreeGroupFeature "FindFreeGroupFeature"
- \ref gz::physics::SetFreeGroupWorldPose "SetFreeGroupWorldPose"
- \ref gz::physics::FreeGroupFrameSemantics "FreeGroupFrameSemantics"
- \ref gz::physics::LinkFrameSemantics "LinkFrameSemantics"
- \ref gz::physics::ForwardStep "ForwardStep"
- \ref gz::physics::RemoveEntities "RemoveEntities"
- \ref gz::physics::sdf::ConstructSdfLink "ConstructSdfLink"
- \ref gz::physics::sdf::ConstructSdfModel "ConstructSdfModel"
- \ref gz::physics::sdf::ConstructSdfWorld "ConstructSdfWorld"

This list defines the minimum requirements for the simulation capability of a
physics engine plugin and also maintains backward compatibility with
downstream physics plugins.

For custom feature requirements, there are two main component classes
in the general structure of a custom feature:
- \ref gz::physics::Entity "Entity" corresponds to the "proxy object" that
the \ref gz::physics::Feature "Feature" is implemented. These are the most
common "proxy objects" that are inherited from `Entity` class:
  - \ref gz::physics::Feature::Engine "Engine": Placeholder class for the
    Engine API. This class serves metadata for the physics engine (for example
    the \ref gz::physics::GetEngineInfo "GetEngineInfo" feature).
    Every Engine feature **must** inherit this class.
  - \ref gz::physics::Feature::Joint "Joint": defines physics concept
    `Joint` behaviors (for example the
    \ref gz::physics::GetBasicJointState "GetBasicJointState" feature).
  - \ref gz::physics::Feature::Link "Link": defines physics concept `Link`
    structure.
  - \ref gz::physics::Feature::Model "Model": defines physics concept
    `Model` structure (for example the
    \ref gz::physics::GetLinkFromModel "GetLinkFromModel" feature
    including both `Link` and `Model` objects).
  - \ref gz::physics::Feature::Shape "Shape": defines physics concept
    `Shape` structure (for example the
    \ref gz::physics::GetShapeKinematicProperties "GetShapeKinematicProperties"
    feature).
  - \ref gz::physics::Feature::World "World": defines physics concept
    `Shape` structure (for example
    the \ref gz::physics::dartsim::RetrieveWorld "RetrieveWorld" feature
    in `dartsim` plugin).

  Note that these object classes are not mutually exclusive and could be defined
  in conjunction together to describe the `Feature`. There are also other
  uncommon objects defined depending on feature functionality, for example, the
  \ref gz::physics::SetFreeGroupWorldPose::FreeGroup "FreeGroup"
  object in `SetFreeGroupWorldPose` feature. For more information about the
  physics concepts, please refer to
  \ref physicsconcepts "Ignition Physics simulation concepts" tutorial.
- \ref gz::physics::Feature::Implementation "Implementation" interfaces
the actual physics engines API for the custom feature. It has
\ref gz::physics::Feature::Implementation::InitiateEngine "InitiateEngine"
to trigger physics engine initiation to provide the required functionalities.

Moreover, we can define dependencies between custom `Features`:
- By default, a blank feature will not require any other features.
If the custom feature does require some other set of features,
then it should be inherited from
\ref gz::physics::FeatureWithRequirements "FeatureWithRequirements" class,
and provided a list of the `Features` required.
- By default, a blank feature will not conflict with any other features. If
the custom feature does conflict with some other set of features, then it should
be inherited from
\ref gz::physics::FeatureWithConflicts "FeatureWithConflicts" class,
and provided a list of the conflicting `Features`. The conflicting `Features`
will not run at the same time when requested.

### Define custom feature template

With the requirements and restrictions above, we first need to define a feature template for the custom feature. In this case, this feature will be responsible for retrieving world pointer from the physics engine. The template is placed in [World.hh](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics2/dartsim/include/ignition/physics/dartsim/World.hh):

\snippet dartsim/include/ignition/physics/dartsim/World.hh feature template

The `RetrieveWorld` feature retrieves
world pointer from physics engine, so we will use the `World` entity inherited
from \ref gz::physics::Feature::World "Feature::World" and declare the
member function `GetDartsimWorld`. Then we substantiate the virtual `Implementation`
member function by overriding in the actual implementation of
the custom feature `RetrieveWorld` later.

Finally, we implement the `World`
entity's member function `GetDartsimWorld` to call the `Implementation`
class's member function `GetDartsimWorld` via
\ref gz::physics::Feature::Entity::Interface "Entity::Interface"
convenience function for querying the feature `Implementation` object.

The newly defined feature template is placed in an `/include` folder shown in the following structure:

```
dartsim
├── worlds
├── src
│    ├── CustomFeatures.hh
│    ├── CustomFeatures.cc
│    └── ...
├── include/ignition/physics/dartsim
│    └──  World.hh
└── CMakeLists.txt
```

We put this custom feature template in `dartsim`, and the next step is to
implement the `RetrieveWorld` feature function using Dartsim API.

### Implement the custom feature

After defining the feature template, we can add it to a custom
\ref gz::physics::FeatureList "FeatureList":

\snippet dartsim/src/CustomFeatures.hh add to list

The custom feature `RetrieveWorld` is added to `CustomFeatureList`, other custom
features could also be added here.
The `CustomFeatures` "FeatureList" here uses data structures and classes from:
- [Base.hh](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics2/dartsim/src/Base.hh), which defines structures that contain information to create `Model`, `Joint`, `Link`, and `Shape` objects in Dartsim API.
They act as an interface between Ignition Physics Library and the actual physics engine.
- \ref gz::physics::Implements3d "Implements3d" for implementing the
custom feature with \ref gz::physics::FeaturePolicy3d "FeaturePolicy3d"
("FeaturePolicy" of 3 dimensions and scalar type `double`).

We will then implement the actual function with Dartsim API in [CustomFeatures.cc](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics2/dartsim/src/CustomFeatures.cc) to override the member function
declared in the custom feature header file:

\snippet dartsim/src/CustomFeatures.cc implementation

Here, we implement the behavior of `GetDartsimWorld` with Dartsim API to return the
world pointer from `EntityStorage` object storing world pointers of `dartsim` in
[Base](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics2/dartsim/src/Base.hh) class.

In the end, we add the implemented `CustomFeatures` "FeatureList" together with
other \ref gz::physics::FeatureList "FeatureList" to final `DartsimFeatures`
"FeatureList" as in [dartsim/src/plugin.cc](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics2/dartsim/src/plugin.cc)
(please see the \ref createphysicsplugin "Implement a physics feature" tutorial
for registering the plugin to Ignition Physics).

The folder structure is shown below:

```
dartsim
├── worlds
├── src
│    ├── CustomFeatures.hh
│    ├── CustomFeatures.cc
│    ├── ...
├── include/ignition/physics/dartsim
└── CMakeLists.txt
```
