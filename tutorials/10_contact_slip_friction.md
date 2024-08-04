\page contactoverview Overview of contact, slip, and friction

This tutorial provides an overview of simplifying assumptions, approximations,
and numerical models used in simulation of contact, slip, and friction in
Gazebo.

## Prerequisites

- \ref physicsconcepts "Physics concepts"

## Conceptual definitions

- **Contact** occurs when two objects are touching.

![Illustration of shapes touching: a long box touching a smaller box and a circle, and two circles touching each other.](img/contact_boxes_circles.svg)

- **Slip** is a measure of relative motion along the surface of contacting objects.
  It can have units of meters/second for translational slip or radians/second
  for rotational slip.

    - Illustration of objects in contact without slip

![Illustration of shapes touching without slip: two stationary boxes, a circle rolling on top of a stationary box, and two circles rolling against each other.](img/noslip_boxes_circles.svg)

    - Illustration of objects in contact with slip

![Illustration of shapes touching with slip: a box sliding along the surface of a longer box, a circle sliding and counter-rotating on top of a stationary box, and a circle sninning in place on top of a stationary circle.](img/noslip_boxes_circles.svg)

- **Friction** is a physical effect that restricts relative tangential motion
  between contacting surfaces.

<!-- image of boxes touching, slipping -->

## Contact simulation

### Approximations for a mathematical model

- Object shapes are non-deformable
- Volumetric overlap / interpenetration of shapes is allowed
- Contact between a pair of objects is represented by a set of contact points,
  which must be inside or on the surface of both objects. A normal direction is
  defined for each contact point, with an implied tangent plane. The contact
  depth is the distance from a contact point to the surface.

  depth > 0 :: inside the object

  depth < 0 :: outside the object

- The process of computing contact points for a pair of objects is often called
  **collision checking**.

#### Commentary on collision checking

- Checking for collisions between `N` objects requires checking `O(N^2)` pairs
  of shapes, which can be computationally expensive.
- A common trick to reduce this computational burden is to approximate each
  shape with an axis-aligned bounding box (AABB) and check pairs of the AABB
  shapes for overlap. This is a **broadphase collision check**, which can
  quickly rule out collisions between pairs of objects that are far apart.
  For objects whose AABB approximations overlap, a more accurate **narrowphase
  collision check** is performed, which computes contact points.

### Numerical representation of contact

A numerical constraint for each contact point is enforced at each timestep to
limit contact depth.

- The constraint force `F_N` acts along the normal direction.
- The constraint is unilateral:

  depth < 0 ==> `F_N` = 0

  `F_N` > 0 ==> depth >= 0

- Constraint relaxation parameters allow tuning the relationship between normal
  force and depth.
- The contact depth is analogous to sinkage in deformable terrain, but this is
  not a terramechanics model.
- Some parameters used by the Open Dynamics Engine in Gazebo-Classic include
  the `kp`, `kd`, `min_depth`, and `max_vel` parameters in
  [//surface/contact](http://sdformat.org/spec?ver=1.11&elem=collision#surface_contact).

## Slip calculation

- Slip is computed at each contact point as the relative linear velocity
  between the contacting objects projected into the contact tangent plane
  with units of `m/s`.
- Torsional slip is computed at each contact point from the relative angular
  velocity between the contacting objects projected into the direction of the
  contact normal with units of `rad/s`.
- When simulating wheels, it is common to use a nondimensional measure of slip.
  This will be detailed in a later section focusing specifically on simulation
  of wheels in contact.

## Friction simulation

### Assumptions for mathematical model of friction

- Friction causes a force `F_f` to act in the contact tangent plane that
  opposes the slip velocity `v_s`.

  `F_f * v_s <= 0`

- Friction is dissipative, removes energy from a system.
- Friction force magnitude is limited and proportional to the contact normal
  force.

### Approximations for mathematical model of friction

- Friction behavior in the two dimensions of the contact tangent plane can be
  decoupled and computed independently along two "friction directions."
  This is known as the "friction pyramid" approximation of the more general
  "friction cone" model.
  By choosing the "first friction direction," the second direction is implied.

  `t_2 = n \cross t_1`

- There are several options when choosing friction directions:
    - Fixed to a global frame
    - Fixed to a rigid body frame
    - Aligned with the slip velocity (which reproduces the "friction cone")
- The choice of friction directions can be significant
    - [Comparison of boxes sliding with pyramid vs cone friction](https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics#Frictionparameters)
    - Careful selection of a body-fixed friction direction allows [simulation
      of omni-directional Mecanum wheels](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/mecanum_drive.sdf)

### Numerical representation of friction

A numerical constraint for each friction direction is enforced at each timestep
to limit slip.

- The constraint force magnitude in each friction direction is limited by the
  normal force and a friction coefficient.

  `-µ F_N <= F_f <= µ F_N`

- The constraint force attempts to drive the slip velocity to zero.
- Constraint relaxation parameters allow tuning the friction response.

<!-- plot showing effect of slip compliance -->
