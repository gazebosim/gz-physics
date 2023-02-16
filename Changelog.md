## Ignition Physics 5.x

### Ignition Physics 5.3.1 (2023-02-16)

1. Fix memory corruption due to faulty refcount tracking
    * [Pull request #480](https://github.com/gazebosim/gz-physics/pull/480)

1. Port: 2 to 5
    * [Pull request #476](https://github.com/gazebosim/gz-physics/pull/476)

1. Prepare for 2.6.1 release
    * [Pull request #468](https://github.com/gazebosim/gz-physics/pull/468)


### Ignition Physics 5.3.0 (2023-01-09)

1. Fix windows warnings related to bullet 
    * [Pull request #473](https://github.com/gazebosim/gz-physics/pull/473)

1. Apply ign-gz after forward merge from ign-physics2 
    * [Pull request #472](https://github.com/gazebosim/gz-physics/pull/472)

1. Port: 2 to 5
    * [Pull request #471](https://github.com/gazebosim/gz-physics/pull/471)

1. Fix build errors and warnings for DART 6.13.0
    * [Pull request #465](https://github.com/gazebosim/gz-physics/pull/465)

1. Backport windows fix
    * [Pull request #437](https://github.com/gazebosim/gz-physics/pull/437)

1. dartsim: fix handling inertia matrix pose rotation
    * [Pull request #351](https://github.com/gazebosim/gz-physics/pull/351)

1. Add code coverage ignore file
    * [Pull request #388](https://github.com/gazebosim/gz-physics/pull/388)

1. Change IGN\_DESIGNATION to GZ\_DESIGNATION
    * [Pull request #390](https://github.com/gazebosim/gz-physics/pull/390)

1. README: Ignition -> Gazebo
    * [Pull request #386](https://github.com/gazebosim/gz-physics/pull/386)

### Ignition Physics 5.2.0 (2022-06-29)

1. dartsim: support non-tree kinematics in AttachFixedJoint
    * [Pull request #352](https://github.com/gazebosim/gz-physics/pull/352)

1. Use SuppressWarning from ign-utils
    * [Pull request #343](https://github.com/gazebosim/gz-physics/pull/343)

1. Make CONFIG a CMake pass-through option for DART
    * [Pull request #339](https://github.com/gazebosim/gz-physics/pull/339)

1. Remove explicitly-defined copy constructor/operator
    * [Pull request #328](https://github.com/gazebosim/gz-physics/pull/328)

1. Fix ExpectData compiler warnings
    * [Pull request #335](https://github.com/gazebosim/gz-physics/pull/335)

1. Fix copying of ExpectData objects
    * [Pull request #337](https://github.com/gazebosim/gz-physics/pull/337)

1. Add Ubuntu Jammy CI
    * [Pull request #329](https://github.com/gazebosim/gz-physics/pull/329)

1. Fix Apache license version
    * [Pull request #325](https://github.com/gazebosim/gz-physics/pull/325)

1. Add project() to examples
    * [Pull request #322](https://github.com/gazebosim/gz-physics/pull/322)

### Ignition Physics 5.1.0 (2021-11-12)

1. Remove unused ign_auto_headers.hh.in
    * [Pull request #305](https://github.com/ignitionrobotics/ign-physics/pull/305)

1. Added DART feature for setting joint limits dynamically.
    * [Pull request #260](https://github.com/ignitionrobotics/ign-physics/pull/260)

1. Allow customization of contact surface properties
    * [Pull request #267](https://github.com/ignitionrobotics/ign-physics/pull/267)

1. Avoid `auto` usage in Eigen expressions.
    * [Pull request #301](https://github.com/ignitionrobotics/ign-physics/pull/301)

1. Fix cmake script for bullet
    * [Pull request #297](https://github.com/ignitionrobotics/ign-physics/pull/297)

1. Improved README.md folder structure
    * [Pull request #295](https://github.com/ignitionrobotics/ign-physics/pull/295)

### Ignition Physics 5.0.0 (2021-09-30)

1. Add GetJointTransmittedWrench feature
    * [Pull request #283](https://github.com/ignitionrobotics/ign-physics/pull/283)

1. [dartsim] Add support for joint frame semantics
    * [Pull request #288](https://github.com/ignitionrobotics/ign-physics/pull/288)

1. Fix TPE Link velocity not being updated and Model velocity not having any effect.
    * [Pull request #289](https://github.com/ignitionrobotics/ign-physics/pull/289)

1. Make ignition-physics CMake config files relocatable
    * [Pull request #282](https://github.com/ignitionrobotics/ign-physics/pull/282)

1. Added DART feature for setting joint limits dynamically.
    * [Pull request #260](https://github.com/ignitionrobotics/ign-physics/pull/260)

1. Remove use of deprecated function sdf::JointAxis::InitialPosition
    * [Pull request #276](https://github.com/ignitionrobotics/ign-physics/pull/276)
    * [Pull request #280](https://github.com/ignitionrobotics/ign-physics/pull/280)

1. Clean up functions that trigger GCC9 warnings
    * [Pull request #261](https://github.com/ignitionrobotics/ign-physics/pull/261)

1. Forward merges
    * [Pull request #292](https://github.com/ignitionrobotics/ign-physics/pull/292)
    * [Pull request #291](https://github.com/ignitionrobotics/ign-physics/pull/291)
    * [Pull request #290](https://github.com/ignitionrobotics/ign-physics/pull/290)
    * [Pull request #272](https://github.com/ignitionrobotics/ign-physics/pull/272)
    * [Pull request #254](https://github.com/ignitionrobotics/ign-physics/pull/254)

1. Infrastructure
    * [Pull request #287](https://github.com/ignitionrobotics/ign-physics/pull/287)
    * [Pull request #246](https://github.com/ignitionrobotics/ign-physics/pull/246)
    * [Pull request #245](https://github.com/ignitionrobotics/ign-physics/pull/245)

## Ignition Physics 4.x

### Ignition Physics 4.x.x (20XX-XX-XX)

### Ignition Physics 4.3.0 (2021-11-11)

1. Remove unused ign_auto_headers.hh.in
    * [Pull request #305](https://github.com/ignitionrobotics/ign-physics/pull/305)

1. Added DART feature for setting joint limits dynamically.
    * [Pull request #260](https://github.com/ignitionrobotics/ign-physics/pull/260)

1. Allow customization of contact surface properties
    * [Pull request #267](https://github.com/ignitionrobotics/ign-physics/pull/267)

1. Avoid `auto` usage in Eigen expressions.
    * [Pull request #301](https://github.com/ignitionrobotics/ign-physics/pull/301)

1. [dartsim] Add support for joint frame semantics
    * [Pull request #288](https://github.com/ignitionrobotics/ign-physics/pull/288)

1. Fix TPE Link velocity not being updated and Model velocity not having any effect.
    * [Pull request #289](https://github.com/ignitionrobotics/ign-physics/pull/289)

1. Infrastructure
    * [Pull request #287](https://github.com/ignitionrobotics/ign-physics/pull/287)
    * [Pull request #281](https://github.com/ignitionrobotics/ign-physics/pull/281)

1. Make ignition-physics CMake config files relocatable
    * [Pull request #282](https://github.com/ignitionrobotics/ign-physics/pull/282)

### Ignition Physics 4.2.0 (2021-07-16)

1. Support FreeGroup features for nested models
    * [Pull request #231](https://github.com/ignitionrobotics/ign-physics/pull/231)

1. Add Gravity Feature, fix LinkFeatures_TEST
    * [Pull request #275](https://github.com/ignitionrobotics/ign-physics/pull/275)

1. Heightmaps 🏔️
    * [Pull request #218](https://github.com/ignitionrobotics/ign-physics/pull/218)

1. Added Ellipsoid and Capsule shapes to TPE
    * [Pull request #203](https://github.com/ignitionrobotics/ign-physics/pull/203)

1. Fix collision detection issue when using joints across nested models
    * [Pull request #268](https://github.com/ignitionrobotics/ign-physics/pull/268)

1. Use slip compliance APIs available in upstream dartsim release
    * [Pull request #265](https://github.com/ignitionrobotics/ign-physics/pull/265)

1. Fix DART deprecation warning
    * [Pull request #263](https://github.com/ignitionrobotics/ign-physics/pull/263)

1. Integrate Bullet engine
    * [Pull request #208](https://github.com/ignitionrobotics/ign-physics/pull/208)

1. Remove `tools/code_check` and update codecov
    * [Pull request #257](https://github.com/ignitionrobotics/ign-physics/pull/257)

1. [Citadel] Update tutorials
    * [Pull request #204](https://github.com/ignitionrobotics/ign-physics/pull/204)
    * [Pull request #259](https://github.com/ignitionrobotics/ign-physics/pull/259)

### Ignition Physics 4.1.0 (2021-05-01)

1. [TPE] Update link pose and velocity
    * [Pull request #179](https://github.com/ignitionrobotics/ign-physics/pull/179)

1. Infrastructure
    * [Pull request #221](https://github.com/ignitionrobotics/ign-physics/pull/221)
    * [Pull request #215](https://github.com/ignitionrobotics/ign-physics/pull/215)

### Ignition Physics 4.0.0 (2021-03-30)

1. Remove nested models
    * [Pull request #230](https://github.com/ignitionrobotics/ign-physics/pull/230)

1. Add dart-bullet dependency to CI
    * [Pull request #239](https://github.com/ignitionrobotics/ign-physics/pull/239)

1. [tpe] Add empty nested model construction and nested model entity management
    * [Pull request #229](https://github.com/ignitionrobotics/ign-physics/pull/229)

1. dartsim: fdir1 expressed_in frame from SDFormat
    * [Pull request #224](https://github.com/ignitionrobotics/ign-physics/pull/224)

1. Set collision detector and solver for DART
    * [Pull request #225](https://github.com/ignitionrobotics/ign-physics/pull/225)

1. Added ellipsoid and capsule to dartsim
    * [Pull request #202](https://github.com/ignitionrobotics/ign-physics/pull/202)

1. Add "ChangedWorldPose" to ForwardStep::Output's expected data
    * [Pull request #238](https://github.com/ignitionrobotics/ign-physics/pull/238)

1. [dartsim] Add empty nested model construction and nested model entity management
    * [Pull request #228](https://github.com/ignitionrobotics/ign-physics/pull/228)

1. Rename CanonicalLink to RootLink
    * [Pull request #234](https://github.com/ignitionrobotics/ign-physics/pull/234)

1. write modified link data to output in physics step
    * [Pull request #223](https://github.com/ignitionrobotics/ign-physics/pull/223)

1. [dartsim] Ensure Link and Model APIs continue to work after joint creation in DART
    * [Pull request #227](https://github.com/ignitionrobotics/ign-physics/pull/227)

1. [dartsim] Fix joint construction errors due to link name duplication or BodyNodes moving to other skeletons
    * [Pull request #220](https://github.com/ignitionrobotics/ign-physics/pull/220)

1. Infrastructure
    * [Pull request #221](https://github.com/ignitionrobotics/ign-physics/pull/221)
    * [Pull request #215](https://github.com/ignitionrobotics/ign-physics/pull/215)
    * [Pull request #133](https://github.com/ignitionrobotics/ign-physics/pull/133)
    * [Pull request #116](https://github.com/ignitionrobotics/ign-physics/pull/116)

1. Bump in edifice: ign-common4
    * [Pull request #205](https://github.com/ignitionrobotics/ign-physics/pull/205)

1. Constructing nested models for dartsim
    * [Pull request #189](https://github.com/ignitionrobotics/ign-physics/pull/189)

1. Depend on ignition-utils1 cli component
    * [Pull request #213](https://github.com/ignitionrobotics/ign-physics/pull/213)
    * [Pull request #217](https://github.com/ignitionrobotics/ign-physics/pull/217)

1. Bump in edifice: sdformat11
    * [Pull request #191](https://github.com/ignitionrobotics/ign-physics/pull/191)

1. Documentation
    * [Pull request #164](https://github.com/ignitionrobotics/ign-physics/pull/164)
    * [Pull request #163](https://github.com/ignitionrobotics/ign-physics/pull/163)
    * [Pull request #162](https://github.com/ignitionrobotics/ign-physics/pull/162)
    * [Pull request #165](https://github.com/ignitionrobotics/ign-physics/pull/165)
    * [Pull request #120](https://github.com/ignitionrobotics/ign-physics/pull/120)
    * [Pull request #144](https://github.com/ignitionrobotics/ign-physics/pull/144)
    * [Pull request #131](https://github.com/ignitionrobotics/ign-physics/pull/131)
    * [Pull request #128](https://github.com/ignitionrobotics/ign-physics/pull/128)
    * [Pull request #115](https://github.com/ignitionrobotics/ign-physics/pull/115)
    * [Pull request #104](https://github.com/ignitionrobotics/ign-physics/pull/104)
    * [Pull request #105](https://github.com/ignitionrobotics/ign-physics/pull/105)
    * [Pull request #106](https://github.com/ignitionrobotics/ign-physics/pull/106)
    * [Pull request #243](https://github.com/ignitionrobotics/ign-physics/pull/243)
    * [Pull request #242](https://github.com/ignitionrobotics/ign-physics/pull/242)

## Ignition Physics 3.x

### Ignition Physics 3.x.x (20XX-XX-XX)

### Ignition Physics 3.3.0 (2021-07-12)

1. Use slip compliance API's available in upstream dartsim release
    * [Pull request #265](https://github.com/ignitionrobotics/ign-physics/pull/265)

1. Fix DART deprecation warning
    * [Pull request #263](https://github.com/ignitionrobotics/ign-physics/pull/263)

1. Integrate Bullet engine
    * [Pull request #208](https://github.com/ignitionrobotics/ign-physics/pull/208)

1. Remove `tools/code_check` and update codecov
    * [Pull request #257](https://github.com/ignitionrobotics/ign-physics/pull/257)

1. [Citadel] Update tutorials
    * [Pull request #204](https://github.com/ignitionrobotics/ign-physics/pull/204)

### Ignition Physics 3.2.0 (2021-04-28)

1. Infrastructure
    * [Pull request #221](https://github.com/ignitionrobotics/ign-physics/pull/221)
    * [Pull request #215](https://github.com/ignitionrobotics/ign-physics/pull/215)
    * [Pull request #211](https://github.com/ignitionrobotics/ign-physics/pull/211)
    * [Pull request #210](https://github.com/ignitionrobotics/ign-physics/pull/210)

1. Linters
    * [Pull request #201](https://github.com/ignitionrobotics/ign-physics/pull/201)
    * [Pull request #154](https://github.com/ignitionrobotics/ign-physics/pull/154)

1. Documentation
    * [Pull request #164](https://github.com/ignitionrobotics/ign-physics/pull/164)
    * [Pull request #163](https://github.com/ignitionrobotics/ign-physics/pull/163)
    * [Pull request #187](https://github.com/ignitionrobotics/ign-physics/pull/187)
    * [Pull request #162](https://github.com/ignitionrobotics/ign-physics/pull/162)
    * [Pull request #120](https://github.com/ignitionrobotics/ign-physics/pull/120)
    * [Pull request #144](https://github.com/ignitionrobotics/ign-physics/pull/144)
    * [Pull request #131](https://github.com/ignitionrobotics/ign-physics/pull/131)
    * [Pull request #128](https://github.com/ignitionrobotics/ign-physics/pull/128)
    * [Pull request #115](https://github.com/ignitionrobotics/ign-physics/pull/115)
    * [Pull request #104](https://github.com/ignitionrobotics/ign-physics/pull/104)
    * [Pull request #105](https://github.com/ignitionrobotics/ign-physics/pull/105)
    * [Pull request #106](https://github.com/ignitionrobotics/ign-physics/pull/106)

1. Add Windows installation
    * [Pull request #194](https://github.com/ignitionrobotics/ign-physics/pull/194)

1. dartsim-plugin windows build fixes
    * [Pull request #148](https://github.com/ignitionrobotics/ign-physics/pull/148)

1. [TPE] Skip computing collisions for static objects
    * [Pull request #181](https://github.com/ignitionrobotics/ign-physics/pull/181)

1. [TPE] Update link pose and velocity
    * [Pull request #179](https://github.com/ignitionrobotics/ign-physics/pull/179)

1. [TPE] Fix poseDirty getter
    * [Pull request #182](https://github.com/ignitionrobotics/ign-physics/pull/182)

1. Add restitution coefficient support for bouncing
    * [Pull request #139](https://github.com/ignitionrobotics/ign-physics/pull/139)

1. Fix compilation with gcc 10.2.0
    * [Pull request #185](https://github.com/ignitionrobotics/ign-physics/pull/185)

1. Support setting canonical link
    * [Pull request #142](https://github.com/ignitionrobotics/ign-physics/pull/142)

1. Ignore invalid joint commands
    * [Pull request #137](https://github.com/ignitionrobotics/ign-physics/pull/137)

1. Fix CONFIG arg in ign_find_package(DART) call
    * [Pull request #119](https://github.com/ignitionrobotics/ign-physics/pull/119)

### Ignition Physics 3.1.0 (2020-10-18)

1. Support for slip compliance in the dartsim-plugin.
    * [Pull request 56](https://github.com/ignitionrobotics/ign-physics/pull/56)

1. Enforce joint effort limit in dartsim-plugin
    * [Pull request 74](https://github.com/ignitionrobotics/ign-physics/pull/74)

1. Support nested models in TPE
    * [Pull request 86](https://github.com/ignitionrobotics/ign-physics/pull/86)

1. Fix CONFIG arg in ign_find_package(DART) call
    * [Pull request 119](https://github.com/ignitionrobotics/ign-physics/pull/119)

1. Fix getting model bounding box in world frame in TPE
    * [Pull request 127](https://github.com/ignitionrobotics/ign-physics/pull/127)

1. Improve fork experience
    * [Pull request 130](https://github.com/ignitionrobotics/ign-physics/pull/130)

### Ignition Physics 3.0.0 (2020-09-30)

1. Upgrade to libsdformat10
    * [Pull request 85](https://github.com/ignitionrobotics/ign-physics/pull/85)

1. Update examples and tutorials
    * [Pull request 107](https://github.com/ignitionrobotics/ign-physics/pull/107)
    * [Pull request 110](https://github.com/ignitionrobotics/ign-physics/pull/110)
    * [Pull request 111](https://github.com/ignitionrobotics/ign-physics/pull/111)
    * [Pull request 112](https://github.com/ignitionrobotics/ign-physics/pull/112)

## Gazebo Physics 2.x

### Gazebo Physics 2.6.1 (2023-01-09)

1. Fix build errors and warnings for DART 6.13.0
    * [Pull request #465](https://github.com/gazebosim/gz-physics/pull/465)

1. Don't install CMakeLists.txt files
    * [Pull request #456](https://github.com/gazebosim/gz-physics/pull/456)

### Gazebo Physics 2.6.0 (2022-11-30)

1. Migrate Ignition headers
    * [Pull request #402](https://github.com/gazebosim/gz-physics/pull/402)

### Gazebo Physics 2.5.1 (2022-08-16)

1. Remove redundant namespace references
    * [Pull request #400](https://github.com/gazebosim/gz-physics/pull/400)

1. Add code coverage ignore file
    * [Pull request #388](https://github.com/gazebosim/gz-physics/pull/388)

1. Change `IGN_DESIGNATION` to `GZ_DESIGNATION`
    * [Pull request #390](https://github.com/gazebosim/gz-physics/pull/390)

1. Ignition -> Gazebo
    * [Pull request #386](https://github.com/gazebosim/gz-physics/pull/386)

1. Make `CONFIG` a CMake pass-through option for DART
    * [Pull request #339](https://github.com/gazebosim/gz-physics/pull/339)

1. Remove explicitly-defined copy constructor/operator for `Shape`
    * [Pull request #328](https://github.com/gazebosim/gz-physics/pull/328)

1. Fix `ExpectData` compiler warnings
    * [Pull request #335](https://github.com/gazebosim/gz-physics/pull/335)

1. Fix copying of `ExpectData` objects
    * [Pull request #337](https://github.com/gazebosim/gz-physics/pull/337)

1. Fix Apache license version
    * [Pull request #326](https://github.com/gazebosim/gz-physics/pull/326)

1. Tutorial fixes
    * [Pull request #318](https://github.com/gazebosim/gz-physics/pull/318)

1. Add `project()` to examples
    * [Pull request #322](https://github.com/gazebosim/gz-physics/pull/322)

### Ignition Physics 2.5.0 (2021-11-09)

1. Remove unused ign_auto_headers.hh.in
    * [Pull request #305](https://github.com/ignitionrobotics/ign-physics/pull/305)

1. Added DART feature for setting joint limits dynamically.
    * [Pull request #260](https://github.com/ignitionrobotics/ign-physics/pull/260)

1. Allow customization of contact surface properties
    * [Pull request #267](https://github.com/ignitionrobotics/ign-physics/pull/267)

1. [dartsim] Add support for joint frame semantics
    * [Pull request #288](https://github.com/ignitionrobotics/ign-physics/pull/288)

1. Use slip compliance API's available in upstream dartsim release
    * [Pull request #265](https://github.com/ignitionrobotics/ign-physics/pull/265)

1. Fix dart deprecation warning
    * [Pull request #263](https://github.com/ignitionrobotics/ign-physics/pull/263)

1. [Citadel] Update tutorials
    * [Pull request #204](https://github.com/ignitionrobotics/ign-physics/pull/204)

1. Infrastructure
    * [Pull request #257](https://github.com/ignitionrobotics/ign-physics/pull/257)
    * [Pull request #281](https://github.com/ignitionrobotics/ign-physics/pull/281)
    * [Pull request #287](https://github.com/ignitionrobotics/ign-physics/pull/287)

### Ignition Physics 2.4.0 (2021-04-14)

1. [TPE] Update link pose and velocity
    * [Pull request #179](https://github.com/ignitionrobotics/ign-physics/pull/179)

1. Infrastructure
    * [Pull request #221](https://github.com/ignitionrobotics/ign-physics/pull/221)
    * [Pull request #211](https://github.com/ignitionrobotics/ign-physics/pull/211)
    * [Pull request #130](https://github.com/ignitionrobotics/ign-physics/pull/130)
    * [Pull request #118](https://github.com/ignitionrobotics/ign-physics/pull/118)

1. Documentation
    * [Pull request #187](https://github.com/ignitionrobotics/ign-physics/pull/187)
    * [Pull request #194](https://github.com/ignitionrobotics/ign-physics/pull/194)

1. TPE: Skip computing collisions for static objects
    * [Pull request #181](https://github.com/ignitionrobotics/ign-physics/pull/181)

1. Add restitution coefficient support for bouncing
    * [Pull request #139](https://github.com/ignitionrobotics/ign-physics/pull/139)

1. Fix compilation with gcc 10.2.0
    * [Pull request #185](https://github.com/ignitionrobotics/ign-physics/pull/185)

1. Fix TPE poseDirty getter
    * [Pull request #182](https://github.com/ignitionrobotics/ign-physics/pull/182)

1. Support setting canonical link
    * [Pull request #142](https://github.com/ignitionrobotics/ign-physics/pull/142)

1. Resolved codecheck issues
    * [Pull request #154](https://github.com/ignitionrobotics/ign-physics/pull/154)

1. Ignore invalid joint commands
    * [Pull request #137](https://github.com/ignitionrobotics/ign-physics/pull/137)

1. Support getting shape AABB in world frame
    * [Pull request #127](https://github.com/ignitionrobotics/ign-physics/pull/127)

1. Fix CONFIG arg in `ign_find_package(DART)` call
    * [Pull request #119](https://github.com/ignitionrobotics/ign-physics/pull/119)

### Ignition Physics 2.3.0 (2020-09-29)

1. Support for slip compliance in the dartsim-plugin.
    * [Pull request 56](https://github.com/ignitionrobotics/ign-physics/pull/56)

1. Enforce joint effort limit in dartsim-plugin
    * [Pull request 74](https://github.com/ignitionrobotics/ign-physics/pull/74)

1. Support nested models in TPE
    * [Pull request 86](https://github.com/ignitionrobotics/ign-physics/pull/86)

### Ignition Physics 2.2.0 (2020-07-30)

1. Add Base and EntityManagement to tpeplugin
    * [Pull Request 32](https://github.com/ignitionrobotics/ign-physics/pull/32)

1. Add all features to tpeplugin
    * [Pull Request 46](https://github.com/ignitionrobotics/ign-physics/pull/46)

1. Fix TPE codecheck errors
    * [Pull Request 62](https://github.com/ignitionrobotics/ign-physics/pull/62)

1. [TPE] Add function to get an Entity's bounding box
    * [Pull Request 59](https://github.com/ignitionrobotics/ign-physics/pull/59)

1. [TPE] Add collision detector
    * [Pull Request 60](https://github.com/ignitionrobotics/ign-physics/pull/60)

1. [TPE] Add GetContactsFromLastStepFeature
    * [Pull Request 61](https://github.com/ignitionrobotics/ign-physics/pull/61)

1. [TPE] Implement collision filtering using collide bitmasks and add CollisionFilterMaskFeature
    * [Pull Request 69](https://github.com/ignitionrobotics/ign-physics/pull/69)

1. Fix copying SpecifyData objects
    * [Pull Request 77](https://github.com/ignitionrobotics/ign-physics/pull/77)

1. Extend contact data with force, normal, and penetration depth
    * [Pull Request 40](https://github.com/ignitionrobotics/ign-physics/pull/40)

1. Add offset to link and collision pose
    * [Pull Request 79](https://github.com/ignitionrobotics/ign-physics/pull/79)

1. Add link offset to simulation
    * [Pull Request 83](https://github.com/ignitionrobotics/ign-physics/pull/83)

### Ignition Physics 2.1.0 (2020-05-07)

1. Add RequestFeatures API for casting the features of an entity to a new feature set when possible.
    * [BitBucket pull request 130](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/130)

1. Add Get Bounding Box features
    * [BitBucket pull request 122](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/122)

1. Install plugins to unversioned files
    * [BitBucket pull request 121](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/121)

1. Add DetachJointFeature feature.
    * [BitBucket pull request 102](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/102)

1. Added support for collision bitmasks for collision filtering
    * [BitBucket pull request 116](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/116)

1. Clean up internal resources when a model gets removed
    * [BitBucket pull request 115](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/115)

1. Trivial Physics Engine - partial implementation
    * [BitBucket pull request 125](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/125)
    * [BitBucket pull request 126](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/126)
    * [Pull request 30](https://github.com/ignitionrobotics/ign-physics/pull/30)
    * [Pull request 45](https://github.com/ignitionrobotics/ign-physics/pull/45)

1. Add simple example of physics plugin and loader
    * [BitBucket pull request 115](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/115)

1. Update BitBucket links, add .gitignore, CODEOWNERS, workflow
    * [Pull request 34](https://github.com/ignitionrobotics/ign-physics/pull/34)
    * [Pull request 39](https://github.com/ignitionrobotics/ign-physics/pull/39)
    * [Pull request 47](https://github.com/ignitionrobotics/ign-physics/pull/47)
    * [Pull request 52](https://github.com/ignitionrobotics/ign-physics/pull/52)

1. Physics Plugin Documentation
    * [Pull request 36](https://github.com/ignitionrobotics/ign-physics/pull/36)

1. Reduce the symbol load caused by feature templates
    * [Pull request 41](https://github.com/ignitionrobotics/ign-physics/pull/41)

1. Fix collision issue with detachable joints
    * [Pull request 31](https://github.com/ignitionrobotics/ign-physics/pull/31)

1. Add PlaneShape feature and implement in dartsim with test.
    * [BitBucket pull request 66](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/66)

1. Restore detached BodyNodes to original skeleton
    * [Pull request 42](https://github.com/ignitionrobotics/ign-physics/pull/42)

### Ignition Physics 2.0.0 (2019-12-10)

1. Support compiling against dart 6.9.
    * [BitBucket pull request 110](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/110)

1. Support sdformat 1.7 frame semantics.
    * [BitBucket pull request 106](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/106)

1. Upgrade to libsdformat9
    * [BitBucket pull request 108](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/108)

1. Improve compile time by reducing length of symbol names
    * [BitBucket pull request 93](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/93)
    * [BitBucket pull request 88](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/88)

## Ignition Physics 1.x

### Ignition Physics 1.10.0 (2020-11-04)

1. Resolved codecheck issues
    * [Pull request 154](https://github.com/ignitionrobotics/ign-physics/pull/154)

1. Ignore invalid joint commands in the dartsim-plugin
    * [Pull request 137](https://github.com/ignitionrobotics/ign-physics/pull/137)

1.  Fix CONFIG arg in ign_find_package(DART) call
    * [Pull request 119](https://github.com/ignitionrobotics/ign-physics/pull/119)

### Ignition Physics 1.9.0 (2020-09-17)

1. Support for slip compliance in the dartsim-plugin.
    * [Pull request 56](https://github.com/ignitionrobotics/ign-physics/pull/56)

1. Enforce joint effort limit in dartsim-plugin
    * [Pull request 74](https://github.com/ignitionrobotics/ign-physics/pull/74)

### Ignition Physics 1.8.0 (2020-05-08)

1. Restore detached BodyNodes to original skeleton
    * [Pull request 42](https://github.com/ignitionrobotics/ign-physics/pull/42)

1. Fix collision issue with detachable joints
    * [Pull request 31](https://github.com/ignitionrobotics/ign-physics/pull/31)

1. Add PlaneShape feature and implement in dartsim with test.
    * [BitBucket pull request 66](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/66)

### Ignition Physics 1.7.0 (2020-04-13)

1. Add RequestFeatures API for casting the features of an entity to a new feature set when possible.
    * [BitBucket pull request 130](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/130)

### Ignition Physics 1.6.0 (2020-03-18)

1. Add Get Bounding Box features
    * [BitBucket pull request 122](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/122)

1. Install plugins to unversioned files
    * [BitBucket pull request 121](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/121)

### Ignition Physics 1.5.0 (2020-02-10)

1. Clean up internal resources when a model gets removed
    * [BitBucket pull request 115](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/115)

1. Add DetachJointFeature feature.
    * [BitBucket pull request 102](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/102)

### Ignition Physics 1.4.0 (2019-08-27)

1. Add SetJointVelocityCommand feature.
    * [BitBucket pull request 100](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/100)

1. Add `IGN_PROFILER_ENABLE` cmake option for enabling the ign-common profiler.
    * [BitBucket pull request 96](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/96)

### Ignition Physics 1.3.1 (2019-07-19)

1. Set the time step from ForwardStep::Input in dartsim.
    * [BitBucket pull request 95](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/95)

### Ignition Physics 1.3.0 (2019-07-18)

1. Support for more friction pyramid parameters in dartsim.
    * [BitBucket pull request 94](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/94)

1. Skip compilation of test plugins if `BUILD_TESTING` is false
    * [BitBucket pull request 92](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/92)

### Ignition Physics 1.2.0 (2019-05-29)

1. Change DART dependency to look for 6.9.0 instead of 6.7.2

### Ignition Physics 1.1.0 (2019-05-20)

1. Simple port of existing PERFORMANCE test as BENCHMARK
    * [BitBucket pull request 84](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/84)

1. Add prototype of FreeGroup features
    * [BitBucket pull request 85](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/85)

1. Feature for adding external forces and torques to a link
    * [BitBucket pull request 79](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/79)

1. Assign friction coefficients from collision elements
    * [BitBucket pull request 80](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/80)

1. Added axis-aligned bounding box feature
    * [BitBucket pull request 68](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/68)
    * [BitBucket pull request 69](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/69)
    * [BitBucket pull request 71](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/71)
    * [BitBucket pull request 78](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/78)

1. Add introduction and installation tutorials
    * [BitBucket pull request 76](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/76)
    * [BitBucket pull request 77](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/77)

1. Remove workaround for console\_bridge linking on macOS
    * [BitBucket pull request 75](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/75)

### Ignition Physics 1.0.1 (2019-03-05)

1. Don't link core to ignition-common3, just the test plugin that uses it
    * [BitBucket pull request 74](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/74)

### Ignition Physics 1.0.0 (2019-03-01)

1. Initial release
