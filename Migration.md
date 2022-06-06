# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Gazebo Physics 5.X to 6.X

### Deprecation

1. The `ignition` namespace is deprecated and will be removed in future versions.  Use `gz` instead.

1. Header files under `ignition/...` are deprecated and will be removed in future versions.
   Use `gz/...` instead.

1. The following `IGN_` / `IGNITION_` prefixed macros are deprecated and will be removed in future versions.
   Use the `GZ_` prefix instead.
   1. `IGN_PHYSICS_DECLARE_JOINT_TYPE`, `IGN_PHYSICS_DECLARE_SHAPE_TYPE`

   1. `IGN_PHYSICS_OPERATEONSPECIFIEDDATA_TEMPLATES`, `IGN_PHYSICS_OPERATEONSPECIFIEDDATA_PREFIX`
   1. `IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS`
   1. `IGN_PHYSICS_ADD_PLUGIN`
   1. `IGN_PLUGIN_REGISTER_MORE_TRANS_UNITS` -> `GZ_**PHYSICS**_REGISTER_MORE_TRANS_UNITS`
   1. `IGN_PHYSICS_REQUEST_FEATURES_MACRO`
   1. `IGN_PHYSICS_CREATE_SELECTOR`
   1. `IGNITION_PHYSICS_CONST_GET_ERROR` (hard-tocked, inside detail header)
   1. `IGNITION_UNITTEST_EXPECTDATA_ACCESS` (hard-tocked, inside test and detail headers)
   1. `IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE` (hard-tocked, inside detail header)
   1. `IGN_PHYSICS_MOCK_MACRO_HELPER`
   1. `IGN_PHYSICS_MOCK_MACRO`


## Gazebo Physics 4.X to 5.X

### Modifications

1. Depends on sdformat12.

## Gazebo Physics 4.1 to 4.2

### Additions

1. Bullet: new plugin implementation.

1. Heightmap: Feature defined and implemented in dartsim.

1. TPE: Capsule and Ellipsoid shapes added.

## Gazebo Physics 3.X to 4.X

### Modifications

1. Depends on ignition-utils1.

1. Depends on sdformat11.

1. `ignition::physics::FindFreeGroupFeature::Implementation::GetFreeGroupCanonicalLink`
   has been replaced by `ignition::physics::FindFreeGroupFeature::Implementation::GetFreeGroupRootLink`.

### Deprecations

1. **physics/FreeGroup.hh**
    + **Deprecation:** `Identity ignition::physics::FreeGroup::CanonicalLink(const Identity &_groupID)`
    + **Replacement:** `Identity ignition::physics::FreeGroup::RootLink(const Identity &_groupID)`

## Gazebo Physics 2.X to 3.X

### Modifications

1. Depends on sdformat10.

## Gazebo Physics 1.X to 2.X

### Modifications

1. Depends on sdformat9.
