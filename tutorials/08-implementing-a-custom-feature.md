\page createcustomfeature "Implement a custom feature"

## Prerequisites

In the previous tutorial \ref installation "Installation", you have installed
the Ignition Physics corresponding to the desired Ignition release. Note that
the recommended Ignition release is Dome.

## Physics engine as a plugin in Ignition Physics

In the last \ref createphysicsplugin "Implement a physics plugin" tutorial, we
know how to implement a dummy physics engine as a plugin and load it using
\ref ignition::physics "Ignition Physics API". In this tutorial, we will look
deeper into the structure of a physics engine plugin, for example, the available
[DART](https://github.com/ignitionrobotics/ign-physics/tree/master/dartsim)
physics engine in `ign-physics` repository and how to define a custom
\ref ignition::physics::Feature "Feature" for the plugin.

### Folder structure

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

As can be seen, there are two physics engines available (more detail
in \ref physicsplugin "Physics plugin tutorial"):
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

Basically, new implementation of \ref ignition::physics::Feature "Feature" or
\ref ignition::physics::FeatureList "FeatureList", which is corresponded to a
functionality of the external physics engine can be defined as a header in
`include/ignition/physics/<plugin_name>` folder. The custom feature could
be added in a \ref ignition::physics::FeatureList "FeatureList"
and implemented its functionalities in `src` folder.

The `dartsim` plugin's \ref ignition::physics::FeatureList "FeatureList" could be
found in \ref physicsplugin "Understanding the Physics Plugin" tutorial.

### Plugin and feature requirements

In general, the minimum set of features that any physics engine plugin must
implement to be supported by Ignition Gazebo is as below:
- \ref ignition::physics::FindFreeGroupFeature "FindFreeGroupFeature"
- \ref ignition::physics::SetFreeGroupWorldPose "SetFreeGroupWorldPose"
- \ref ignition::physics::FreeGroupFrameSemantics "FreeGroupFrameSemantics"
- \ref ignition::physics::LinkFrameSemantics "LinkFrameSemantics"
- \ref ignition::physics::ForwardStep "ForwardStep"
- \ref ignition::physics::RemoveEntities "RemoveEntities"
- \ref ignition::physics::sdf::ConstructSdfLink "ConstructSdfLink"
- \ref ignition::physics::sdf::ConstructSdfModel "ConstructSdfModel"
- \ref ignition::physics::sdf::ConstructSdfWorld "ConstructSdfWorld"

This list defines the minimum requirements for the simulation capability of a
physics engine plugin and also maintains backward compatibility with
downstream physics plugins.

For custom feature requirements, there are two main component classes
in the general structure of a custom feature:
- \ref ignition::physics::Entity "Entity" corresponds to the "proxy object" that
the \ref ignition::physics::Feature "Feature" is implemented. These are the most
common "proxy objects" that are inherited from `Entity` class:
  - \ref ignition::physics::Feature::Engine "Engine": Placeholder class for the
    Engine API. This class serves metadata for the physics engine (for example
    the \ref ignition::physics::GetEngineInfo "GetEngineInfo" feature).
    Every Engine feature **must** inherit this class.
  - \ref ignition::physics::Feature::Joint "Joint": defines physics concept `Joint` behaviors (for example
    the \ref ignition::physics::GetBasicJointState "GetBasicJointState"
    feature).
  - \ref ignition::physics::Feature::Link "Link": defines physics concept `Link` structure.
  - \ref ignition::physics::Feature::Model "Model": defines physics concept `Model` structure (for example
    the \ref ignition::physics::GetLinkFromModel "GetLinkFromModel"
    feature including both `Link` and `Model` objects).
  - \ref ignition::physics::Feature::Shape "Shape": defines physics concept `Shape` structure (for example
    the \ref ignition::physics::GetShapeKinematicProperties "GetShapeKinematicProperties"
    feature).
  - \ref ignition::physics::Feature::World "World": defines physics concept `Shape` structure (for example
    the \ref ignition::physics::dartsim::RetrieveWorld "RetrieveWorld" feature
    in `dartsim` plugin).

  Note that these object classes are not mutually exclusive and could be defined in conjunction together to describe the `Feature`. There are also other uncommon
  objects defined depending on feature functionality, for example, the \ref ignition::physics::SetFreeGroupWorldPose::FreeGroup "FreeGroup"
  object in `SetFreeGroupWorldPose` feature. For more information about the
  physics concepts, please refer to
  \ref physicsconcepts "Ignition Physics simulation concepts" tutorial.
- \ref ignition::physics::Feature::Implementation "Implementation" interfaces
the actual physics engines API for the custom feature. It has
\ref ignition::physics::Feature::Implementation::InitiateEngine "InitiateEngine"
 to trigger physics engine initiation to provide the required functionalities.

Moreover, we can define dependencies between custom `Features`:
- By default, a blank feature will not require any other features.
If the custom feature does require some other set of features,
then it should be inherited from
\ref ignition::physics::FeatureWithRequirements "FeatureWithRequirements" class,
and provided a list of the `Features` required.
- By default, a blank feature will not conflict with any other features. If
the custom feature does conflict with some other set of features, then it should
be inherited from
\ref ignition::physics::FeatureWithConflicts "FeatureWithConflicts" class,
and provided a list of the conflicting `Features`. The conflicting `Features`
will not run at the same time when requested.

### Define a custom feature

With the requirements and restrictions above, we will implement an example
custom `Feature` that retrieves a simulation world from `dartsim` physics engine.
From [dartsim/include/ignition/physics/dartsim/World.hh](https://github.com/ignitionrobotics/ign-physics/blob/main/dartsim/include/ignition/physics/dartsim/World.hh)
, we show it here for convenience:

```cpp
#include <dart/simulation/World.hpp>
#include <ignition/physics/FeatureList.hh>

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
class RetrieveWorld : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    /// \brief Get the underlying dartsim world for this World object.
    public: dart::simulation::WorldPtr GetDartsimWorld();
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual dart::simulation::WorldPtr GetDartsimWorld(
        const Identity &_worldID) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
dart::simulation::WorldPtr RetrieveWorld::World<PolicyT, FeaturesT>
::GetDartsimWorld()
{
  return this->template Interface<RetrieveWorld>()
      ->GetDartsimWorld(this->identity);
}

}
}
}
```

As can be seen, after including the necessary library of `dartsim` and `ign-physics`,
we define the `RetrieveWorld` custom feature inherited from the base
\ref ignition::physics::Feature "Feature".

As defined, the `RetrieveWorld` feature retrieves
world pointer from physics engine, so it is natural to define `World` entity inherited
from \ref ignition::physics::Feature::World "Feature::World" and declare the
necessary member function `GetDartsimWorld`. Then we define the `Implementation`
class having virtual member function for overriding in the actual implementation of
the custom feature `RetrieveWorld` later.

Finally, we implement the `World`
entity's member function `GetDartsimWorld` to call the `Implementation`
class's member function `GetDartsimWorld` via
\ref ignition::physics::Feature::Entity::Interface "Entity::Interface"
convenience function for querying the feature `Implementation` object.

### Implement the custom plugin

After defining the custom feature, please look into where it is added to a
\ref ignition::physics::FeatureList "FeatureList" in
[dartsim/src/CustomFeatures.hh](https://github.com/ignitionrobotics/ign-physics/blob/main/dartsim/src/CustomFeatures.hh)
and implemented in [dartsim/src/CustomFeatures.cc](https://github.com/ignitionrobotics/ign-physics/blob/main/dartsim/src/CustomFeatures.cc).
We display them here for convenience:

- `CustomFeatures.hh`:

```cpp
#include <ignition/physics/Implements.hh>
#include <ignition/physics/dartsim/World.hh>
#include "Base.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using CustomFeatureList = FeatureList<
  RetrieveWorld
>;

class CustomFeatures :
    public virtual Base,
    public virtual Implements3d<CustomFeatureList>
{
  public: dart::simulation::WorldPtr GetDartsimWorld(
      const Identity &_worldID) override;
};

}
}
}

```

The custom feature `RetrieveWorld` is added to `CustomFeatureList`, other custom
features could also be added here.
The `CustomFeatures` "FeatureList" here inherits from:
- [Base](https://github.com/ignitionrobotics/ign-physics/blob/main/dartsim/src/Base.hh)
class for foundation definitions of Models, Joints, Links, and Shapes objects
of `dartsim` interfacing to Ignition Physics API.
- \ref ignition::physics::Implements3d "Implements3d" for implementing the
custom feature with \ref ignition::physics::FeaturePolicy3d "FeaturePolicy3d"
("FeaturePolicy" of 3 dimensions and scalar type `double`).

Based on the `CustomFeatureList`, we will override the corresponding member functions
declared in each of the custom features. In this case, we have only the custom
feature `RetrieveWorld` and its corresponding `Implementation::GetDartsimWorld`
member function.

- `CustomFeatures.cc`:

```cpp
#include "CustomFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
dart::simulation::WorldPtr CustomFeatures::GetDartsimWorld(
    const Identity &_worldID)
{
  return this->worlds.at(_worldID);
}

}
}
}
```

Here we simply implement the actual behavior of `GetDartsimWorld` to return the world pointer
from `EntityStorage` object storing world pointers of `dartsim` in
[Base](https://github.com/ignitionrobotics/ign-physics/blob/main/dartsim/src/Base.hh) class.

Finally, we add the implemented `CustomFeatures` "FeatureList" together with
other \ref ignition::physics::FeatureList "FeatureList" to final `DartsimFeatures`
"FeatureList" as in [dartsim/src/plugin.cc](https://github.com/ignitionrobotics/ign-physics/blob/main/dartsim/src/plugin.cc)
(please see \ref createphysicsplugin "Implement a physics plugin" for
registering the plugin to Ignition Physics).
