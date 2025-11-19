# MuJoCo Physics Plugin

# Features

## Physics features list

These are the specific physics API features implemented.

- GetEngineInfo
- GetWorldFromEngine
- ConstructEmptyWorldFeature

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

