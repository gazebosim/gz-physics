# MuJoCo Physics Plugin

## Physics features list

These are the specific physics API features implemented.

### EntityManagementFeatureList

```c++

struct EntityManagementFeatureList : FeatureList<
  GetEngineInfo,
  GetWorldFromEngine,
  GetLinkFromModel,
  GetModelFromWorld,
  GetShapeFromLink,
  RemoveEntities,
  ConstructEmptyWorldFeature
  // ConstructEmptyModelFeature,
  // ConstructEmptyLinkFeature
  // CollisionFilterMaskFeature,
  // WorldModelFeature
> { };

```

TODO:

- [ ] (S) Get entities by name
- [ ] (S) Get an entities index
- [ ] (M) Remove models

### FreeGroupFeatureList

```c++
struct FreeGroupFeatureList : gz::physics::FeatureList<
  FindFreeGroupFeature,
  SetFreeGroupWorldPose,
  SetFreeGroupWorldVelocity
> { };
```

This feature is mostly stubbed out, so all the actual `Set` functions
need to be implemented.

TODO:

- [ ] (S) Implement checks to see if a given model has a freegroup in `FindFreeGroupFeature`.
- [ ] (S) Set linear and angular velocities on a freegroup.
- [ ] (S) Set pose on a freegroup

### KinematicsFeatureList

```c++
struct KinematicsFeatureList : FeatureList<
  LinkFrameSemantics,
  ShapeFrameSemantics,
  JointFrameSemantics,
  FreeGroupFrameSemantics
> { };
```

TODO:

- [ ] (S) Compute frame data for shapes
- [ ] (S) Compute frame data for joints
  - I'm not sure if this is actually needed by gz-sim.

### SDFFeatureList

```c++
struct SDFFeatureList : FeatureList<
  sdf::ConstructSdfWorld,
  sdf::ConstructSdfModel
> { };
```

TODO:

- [ ] (S) Resolve SDFormat frames before instead of using raw poses
- [ ] (S) Apply poses of collisions
- [ ] (M) Support nested models
- [ ] (M) Support joints
- [ ] (S) ConstructSdfCollisions

### SimulationFeatureList

```c++
struct SimulationFeatureList : gz::physics::FeatureList<
  ForwardStep
> { };
```

TODO:

- [ ] Report only links with changed poses instead of all the links.

## Features to Implement for a fully viable engine, but with some missing features

```c++

struct JointFeatureList : FeatureList<
  GetBasicJointState,
  SetBasicJointState,
  GetBasicJointProperties,

  SetJointVelocityCommandFeature,
  SetJointPositionLimitsFeature,
  SetJointVelocityLimitsFeature,
  SetJointEffortLimitsFeature,

  SetJointTransformFromParentFeature,
  AttachFixedJointFeature,
  DetachJointFeature,

  GetRevoluteJointProperties,
  GetPrismaticJointProperties,
  FixedJointCast,
  GetJointTransmittedWrench,
> { };
```

```c++
struct KinematicLinkFeatureList : FeatureList<
  KinematicLink
> { };
```

```c++
struct LinkFeatureList : FeatureList<
  AddLinkExternalForceTorque
> { };
```


```c++

struct SimulationFeatureList : gz::physics::FeatureList<
  ForwardStep,
  GetContactsFromLastStepFeature
> { };
```

- The  GetContactsFromLastStepFeature feature is missing

```

struct WorldFeatureList : FeatureList<
  Gravity,
> { };
```



## Notes

The Physics system in Gazebo requires the following features at a minimum:

```c++
  /// \brief This is the minimum set of features that any physics engine must
  /// implement to be supported by this system.
  /// New features can't be added to this list in minor / patch releases, in
  /// order to maintain backwards compatibility with downstream physics plugins.
  public: struct MinimumFeatureList : physics::FeatureList<
          physics::FindFreeGroupFeature,
          physics::SetFreeGroupWorldPose,
          physics::FreeGroupFrameSemantics,
          physics::LinkFrameSemantics,
          physics::ForwardStep,
          physics::RemoveModelFromWorld,
          physics::sdf::ConstructSdfModel,
          physics::sdf::ConstructSdfWorld,
          physics::GetLinkFromModel,
          physics::GetShapeFromLink
          >{};
```

In addition to these, the following is a minimal list of features should be
implemented to evaluate MuJoCo in gz-sim.

```c++
  /// Feature list to handle joints.
  struct JointFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::GetJointFromModel,
            physics::GetBasicJointProperties,
            physics::GetBasicJointState,
            physics::SetBasicJointState>{};
```

```c++

  // Joint transmitted wrench
  /// \brief Feature list for getting joint transmitted wrenches.
  public: struct JointGetTransmittedWrenchFeatureList : physics::FeatureList<
            physics::GetJointTransmittedWrench>{};
```

```c++
  // Joint velocity command
  /// \brief Feature list for set joint velocity command.
  public: struct JointVelocityCommandFeatureList : physics::FeatureList<
            physics::SetJointVelocityCommandFeature>{};
```

Not sure if this is needed for meshes.

```c++
  /// \brief Feature list to handle collisions.
  public: struct CollisionFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::sdf::ConstructSdfCollision>{};
  // Meshes
  /// \brief Feature list for meshes.
  /// Include MinimumFeatureList so created collision can be automatically
  /// up-cast.
  public: struct MeshFeatureList : physics::FeatureList<
            CollisionFeatureList,
            physics::mesh::AttachMeshShapeFeature>{};
```

```c++
  // Nested Models
  /// \brief Feature list to construct nested models
  public: struct NestedModelFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::sdf::ConstructSdfNestedModel>{};
```

1. Critical gz-sim Requirements (Highest Priority)
    - Set Model Pose: The `SetFreeGroupWorldPose` function is not implemented.
    - Remove Models: The `RemoveModelFromWorld` function is not implemented.

2. Major Missing Features
    - Joints: The entire `JointFeatureList` is missing, which prevents all joint functionality (creation, state access, commanding, feedback).
    - Contact Reporting: `GetContactsFromLastStepFeature` is not implemented, so contact sensor data is unavailable.
    - External Forces: `AddLinkExternalForceTorque` is not implemented.
    - World Properties: The `WorldFeatureList` (including Gravity) is not implemented.

3. Incomplete Features & Specific Runtime Construction Limitations
    - Whole Model Runtime Construction: This is supported. New, complete models can be added to the world after startup.
    - Piecemeal Runtime Construction: The ability to add individual components to existing models at runtime is missing. This includes:
        - `AttachMeshShapeFeature` (adding a mesh to an existing link).
        - `ConstructSdfCollision` (adding a collision to an existing link).
    - SDF & Model Loading Details:
        - SDF frames are not fully resolved (uses `RawPose`).
        - Support for nested models is incomplete.
        - Heightmap and Polyline geometries are not supported.
    - Entity Management: Functions to find entities by name or get their index are stubbed.
    - Kinematics: Calculating frame data for shapes and joints is a TODO.
    - Simulation Step: Pose updates are inefficient (all links are reported as changed).
