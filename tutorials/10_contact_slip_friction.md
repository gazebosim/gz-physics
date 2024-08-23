\page contactoverview Overview of contact, slip, and friction

This tutorial provides an overview of simplifying assumptions, approximations,
and numerical models used in simulation of contact, slip, and friction in
Gazebo.

## Prerequisites

- \ref physicsconcepts "Physics concepts"

## Conceptual definitions

- **Contact** occurs when two objects are touching.

    - Illustration of shapes touching: a long box touching a smaller box and a circle, and two circles touching each other.

    - ![Illustration of shapes touching: a long box touching a smaller box and a circle, and two circles touching each other.](img/contact_boxes_circles.svg)

- **Slip** is a measure of relative motion along the surface of contacting objects.
  It can have units of meters/second for translational slip or radians/second
  for rotational slip.

    - Illustration of shapes touching without slip: two stationary boxes, a circle rolling on top of a stationary box, and two circles rolling against each other.

    - ![Illustration of shapes touching without slip: two stationary boxes, a circle rolling on top of a stationary box, and two circles rolling against each other.](img/noslip_boxes_circles.svg)

    - Illustration of shapes touching with slip: a box sliding along the surface of a longer box, a circle sliding and counter-rotating on top of a stationary box, and a circle spinning in place on top of a stationary circle.

    - ![Illustration of shapes touching with slip: a box sliding along the surface of a longer box, a circle sliding and counter-rotating on top of a stationary box, and a circle spinning in place on top of a stationary circle.](img/slip_boxes_circles.svg)

<!-- illustrate torsional slip? -->

- **Friction** is a physical effect that restricts slip. Friction causes a
  force to restrict translational slip and a torque to restrict rotational
  slip. Friction can both act to reduce existing slip and to prevent slip from
  occurring.

    - Illustration of a box at rest on an inclined surface with downward
      gravity and a friction force between the box and inclined surface that
      holds the box in place.

    - Illustration of a sliding box and a circle rolling with slip and
      friction forces opposing the slip.

## Contact simulation

### Approximations for a mathematical model

- Object shapes are non-deformable
- Volumetric overlap / interpenetration of shapes is allowed

    - Illustration of overlapping boxes and spheres.

- Contact between a pair of objects is represented by a set of contact points,
  which must be inside or on the surface of both objects.

    - Illustration of contact points for overlapping boxes and spheres.
    <!--
      Sphere-sphere: one point
      Sphere-box face: one point
      Box face-box face: two points?
    -->

- A normal direction `n` is defined for each contact point. The plane orthogonal
  to the normal direction that passes through the contact point is defined as
  the contact tangent plane.

    - Illustration of contact normals / tangent planes for overlapping boxes and spheres.
    - Illustration of contact coordinate frame from Open Dynamics Engine manual
    - ![Illustration of contact coordinate frame from Open Dynamics Engine manual](https://ode.org/wiki/images/b/b9/Contact.jpg)

- Contact depth is defined as the distance from a contact point to the surface,
  with the following sign convention:

    - depth > 0 :: inside the object

    - depth = 0 :: on surface of the object

    - depth < 0 :: outside the object

- The process of computing contact points for a pair of objects is often called
  **collision checking**.

#### Commentary on contact point approximation / collision checking

- Computational cost
    - Checking for collisions between `N` objects requires checking `O(N^2)` pairs
      of shapes, which can be computationally expensive.
    - A common trick to reduce this computational burden is to approximate each
      shape with an axis-aligned bounding box (AABB) and check pairs of the AABB
      shapes for overlap. This is a **broadphase collision check**, which can
      quickly rule out collisions between pairs of objects that are far apart.
      For objects whose AABB approximations overlap, a more accurate **narrowphase
      collision check** is performed, which computes contact points.
- Uniqueness, quality, ability of constraint solver to converge

### Numerical representation of contact

A numerical constraint for each contact point is enforced at each timestep to
limit contact depth.

- The constraint force `N` acts along the normal direction `n`.
- The constraint is unilateral:

  depth < 0 ==> `N` = 0

  `N` > 0 ==> depth >= 0

- Constraint relaxation parameters allow tuning the relationship between normal
  force and depth (see the
  [Soft Constraints presentation at GDC2011](https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf)
  for a good discussion of the ERP and CFM constraint relaxation parameters
  and how they map to linear stiffness `kp` and damping `kd` parameters).
  Some parameters used by the Open Dynamics Engine in Gazebo-Classic include
  the `kp`, `kd`, `min_depth`, and `max_vel` parameters in
  [//surface/contact](http://sdformat.org/spec?ver=1.11&elem=collision#surface_contact)
  (see also the Physics parameters tutorial for Gazebo Classic).
- The contact depth is analogous to sinkage in deformable terrain but is not
  computed from a terramechanics model.

## Slip calculation

- Translational slip `v_t` is computed at each contact point as follows:
    - Compute the linear velocity of each shape at the contact point.
    - Compute the difference between these velocities to yield the
      relative linear velocity at the contact point.
    - Project the relative linear velocity at the contact point into the
      contact tangent plane. This is the translational slip `v_t`
      with units of `m/s`.
- Torsional slip `ω_n` is computed at each contact point as follows:
    - Compute the angular velocity of each shape.
    - Compute the diference between these velocities to yield the
      relative angular velocity.
    - The component of the relative angular velocity parallel to the normal
      direction is the torsional slip `ω_n` with units of `rad/s`.
- When simulating wheels, it is common to use a nondimensional measure of slip.
  This will be detailed in a later section focusing specifically on simulation
  of wheels in contact.

## Friction simulation

### Principled assumptions for mathematical model of friction

- Friction causes a force `T` to act in the contact tangent plane that
  opposes the slip velocity `v_t`.

  `T \dot v_t <= 0`

- Friction is dissipative, removes energy from a system.

- Friction force magnitude is limited and proportional to the contact normal
  force.
    - The friction coefficient µ is used to represent the maximum ratio of
      friction forces to normal forces `T / N` for a given contact.
    - For a geometric interpretation, the arc-tangent of µ represents the
      maximum angle of a sloped surface for which a box can rest on the surface
      without sliding down due to gravity.
    - A generic mathematical expression of this limit in the two dimensions of
      the contact tangent plane requires a nonlinear constraint, for example
      with `T_x` and `T_y` representing the friction force x and y
      components:
      `-µ N <= sqrt{T_x^2 + T_y^2} <= µ N`.
    - A geometric interpretation of this relationship is the "friction
      cone" concept.
    - Illustration of friction cone from
      [Open Dynamics Engine manual](http://ode.org/wiki/index.php?title=Manual#Friction_Approximation).
    - ![Illustration of friction cone from Open Dynamics Engine manual](https://ode.org/wiki/images/4/49/Cone_frottement.jpg)

### Approximations for mathematical model of friction

- To reduce the complexity of the friction constraint, the nonlinear
  constraint representing behavior in the two dimensions of the contact
  tangent plane is decoupled into two independent linear constraints along two
  orthogonal "friction directions."
  This is known as the "friction pyramid" approximation of the more general
  "friction cone" model.

  `-µ N <= T_x <= µ N`.

  `-µ N <= T_y <= µ N`.

    - Illustration of contact coordinate frame with friction directions from Open Dynamics Engine manual
    - ![Illustration of contact coordinate frame with friction directions from Open Dynamics Engine manual](https://ode.org/wiki/images/b/b9/Contact.jpg)

- To specify friction directions, only one direction needs to be specified,
  though it must not be parallel to the normal direction `n`.
    - Given `n` and the user-provided "first friction direction" `fdir1`,
      a vector `t_1` orthogonal to `n` is computed by subtracting the component
      of `fdir1` orthogonal to `n` and normalizing to unit length.
    - Given `n` and `t_1`, the second direction `t_2` can be computed directly:
      `t_2 = n \cross t_1`

- There are several options when choosing friction directions:
    - Aligned with the slip velocity. This has the advantage of reproducing
      the "friction cone" behavior for sliding contact but is undefined for
      objects at rest.
      Support for this depends on the underlying physics engine. For the fork
      of Open Dynamics Engine (ODE) used in Gazebo Classic, the friction cone
      model can be enabled by setting the
      [//physics/ode/solver/friction_model](http://sdformat.org/spec?ver=1.6&elem=physics#solver_friction_model)
      element to `cone_model`. See further documentation in the
      [Gazebo Classic Physics Parameters tutorial](https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics#Frictionparameters)
      and an example world
      [friction\_cone.world](https://github.com/gazebosim/gazebo-classic/blob/e4b4d0fb752c7e43e34ab97d0e01a2a3eaca1ed4/test/worlds/friction_cone.world#L16)
      that is used in the
      [MaximumDissipation physics friction test](https://github.com/gazebosim/gazebo-classic/blob/e4b4d0fb752c7e43e34ab97d0e01a2a3eaca1ed4/test/integration/physics_friction.cc#L300).
    - Fixed to a rigid body frame. This is useful for bodies with distinct
      anisotropic friction behavior (such as the longitudinal and lateral
      friction behavior of pneumatic tires or for
      [simulation of omni-directional Mecanum wheels](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/mecanum_drive.sdf)),
      but more logic is required to
      determine which body-fixed frame to use if two objects come into
      contact that each prefer to use a friction direction defined in their
      own body-fixed frames.
      To specify a friction direction in the body-fixed frame in Gazebo, use
      the [`//collision/surface/friction/ode/fdir1` SDFormat parameter](http://sdformat.org/spec?ver=1.11&elem=collision#ode_fdir1)
      as detailed in [this update to the Gazebo friction tutorial](https://github.com/osrf/gazebo_tutorials/pull/174).
    - The corner cases of the previous two approaches can be avoided by
      aligning the friction directions with a fixed frame. This is logically
      simpler and is the default behavior in gazebo-classic for this reason,
      though it has known limitations (see the
      [comparison of boxes sliding with pyramid vs cone friction](https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics#Frictionparameters)).

### Numerical representation of friction

A numerical constraint for each friction direction is enforced at each timestep
to limit slip.

- The constraint force magnitude in each friction direction is limited by the
  normal force and a friction coefficient.

  `-µ N <= T <= µ N`

- The constraint force attempts to drive the slip velocity to zero.
  Constraint relaxation parameters allow tuning the friction response.

- Stiff friction model example

    - For a very "stiff" friction model, the maximum available friction force
      could be applied when any slip is detected at all, with friction force
      as a step function of slip velocity `T = -sgn(v_t)`, as illustrated
      in the plot below.

~~~
                ^
                | friction force T (units: N)
                |
                |
o o o o o o o o o maximum friction force µN
                o
                o
                o
                o
                o
                o
----------------o--------------------------> slip v_t (units: m/s)
                o
                o
                o
                o
                o
                o
                o o o o o o o o o o o o  minimum friction force -µN
                |
                |
~~~

- Compliant friction model example

    - A more compliant friction model is generated by applying a constraint
      relaxation parameter that changes the relationship between slip and
      friction force to represent a saturated linear function, as shown in the
      plot below. For the step function in the previous plot, the slope at
      the origin is infinite, whereas a finite slope allows friction forces
      less than the maximum value to be applied for small slip values.

~~~
                ^
                | friction force T (units: N)
                |
                |
 o o o o o      | maximum friction force µN
          o     |
           o    |
            o   |
             o  |
              o |
               o|
----------------o--------------------------> slip v_t (units: m/s)
                |o
                | o---┐
                |  o  | slope is "slip stiffness"
                |   o | inverse slope is "slip compliance"
                |    o|
                |     o
                |      o o o o o o o o o minimum friction force -µN
                |
                |
~~~

- In Open Dynamics Engine, the "Force-dependent-slip (FDS)" parameter
  (implemented as the CFM parameter to relax the friction constraint) can be
  used to set the inverse slope of the friction vs slip curve with units of
  `m/s/N`.
  In Gazebo these parameters are referred to as "slip compliance."

- The slip compliance parameters have been characterized for ODE in
  Gazebo Classic in the
  [physics friction SphereSlip test](https://github.com/gazebosim/gazebo-classic/blob/e4b4d0fb752c7e43e34ab97d0e01a2a3eaca1ed4/test/integration/physics_friction.cc#L549)
  using the [friction_spheres test world](https://github.com/gazebosim/gazebo-classic/blob/e4b4d0fb752c7e43e34ab97d0e01a2a3eaca1ed4/test/worlds/friction_spheres.world).
  The test involves a world with an inclined gravity vector (simulating a
  sloped surface of angle `slope_angle`) and multiple models with
  spherical collision shapes with nonzero slip compliance parameters.
  Each collision shape has only one contact point, but some models have
  multiple shapes (see animation below).
  The simulated slope is small enough that the friction forces are not
  saturated, and models reach constant slip velocity that is:

  - proportional to `sin(slope_angle)` (though only one slope was tested)
  - proportional to the slip compliance parameter
  - proportional to the weight of the model
  - inversely proportional to the number of contact points

- Animation of models with spherical collisions sliding with constant slip to characterize slip compliance behavior.
  For each model with the same color, the product of slip compliance and model weight is equal.
  The additional contact points for the multi-sphere models cause a reduction in steady slip velocity.
- ![Animation of models with spherical collisions sliding with constant slip to characterize slip compliance behavior](https://osrf-migration.github.io/gazebo-gh-pages/data/bitbucket.org/repo/jgXqbo/images/42019069-friction_spheres.gif)

- The reason for the dependence on the number of contact points is that each
  additional contact point with non-zero slip compliance acts like an
  additional viscous damper in acting on the model in parallel.
  An attempt was made to normalize the effect of multiple contact points for a
  single collision shape, such as a box on a plane
  (but not a model with multiple spheres), in
  [bitbucket osrf/gazebo#2965](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2965/page/1)
  (merged in [gazebo-classic@d6358dbe1cc1](https://github.com/gazebosim/gazebo-classic/commit/d6358dbe1cc1e3c227338cb7e00eab24aa46f4e7))
  by scaling the slip compliance for each collision's contact constraints
  by the number of contact points acting at that time step.
  This allows a box with the same weight and slip compliance to reach the same
  slip velocity as the single-sphere models in the SphereSlip test (see
  screen capture below).
- Animation of box sliding with same velocity as single-sphere models in SphereSlip test.
- ![Animation of box sliding with same velocity as single-sphere models in SphereSlip test](https://osrf-migration.github.io/gazebo-gh-pages/data/bitbucket.org/repo/jgXqbo/images/959316863-slip_after.gif)
