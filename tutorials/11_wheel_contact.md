\page wheelcontact Simulating wheel-terrain contact

This tutorial expands on the overview of contact, slip and friction with special
considerations for simulating wheel-terrain contact in Gazebo.

## Prerequisites

- \ref contactoverview "Overview of contact, slip, and friction"

## Contact simulation: special considerations for wheels and terrain

### Choosing a wheel collision shape

One can choose a cylinder, ellipsoid, sphere or triangle mesh for a wheel
collision shape. Keys to consider are:

- The actual shape of the wheel you are trying to model
- The quality of contact points that will be generated with the chosen terrain
  shape

For example with a heightmap terrain shape in Gazebo-Classic using
Open Dynamics Engine (ODE), there are known issues
([odedevs/ode#71](https://bitbucket.org/odedevs/ode/issues/71/small-spheres-fall-through-convex-edges-of)
and [gazebo-classic#684](https://github.com/gazebosim/gazebo-classic/issues/684))
with all of the mentioned wheel shapes, but for spheres and heightmaps,
the issue of a sphere falling though a convex edge can be avoided if the
radius of the sphere is larger than the resolution of the heightmap grid.

- Computational complexity of collision checking (primitive shape vs mesh
  with many triangles)

### Choosing a terrain collision shape

- For flat terrain, a combination of boxes and a ground plane can be used,
  which are simple and will have reasonable contact points.
- For more general 3-D terrain surfaces, a triangle mesh is the most flexible,
  though the collision checkers provided by different physics engines vary in
  the quality and number of contact points generated for triangle meshes.
- A heightmap can be used to model a 2.5-D surface, which is defined by a set
  of elevations over a regularly-spaced 2-D grid of points. A heightmap has some
  advantages in collision checking compared to a triangle mesh, since it only
  needs to check for collisions over the portion of the heightmap covered by
  the other object's AABB. Despite this efficiency, there may be issues with
  the contact points generated with a heightmap, such as those noted in
  [odedevs/ode#71](https://bitbucket.org/odedevs/ode/issues/71/small-spheres-fall-through-convex-edges-of)
  and [gazebo-classic#684](https://github.com/gazebosim/gazebo-classic/issues/684).

## Slip calculation for wheels

From the Overview of contact, slip, and friction documentation, slip is defined
as a measure of relative motion along the surface of contacting objects with
units of [m/s] for translational slip or [rad/s] for rotational / torsional
slip.
When simulating wheel-terrain contact, it is common to use a nondimensional
representation of slip and to compute slip differently along the rolling
direction (also known as the longitudinal direction) and the non-rolling
lateral direction.

### Definition of wheel directions and wheel contact coordinate frame

The longitudinal direction of a wheel can be loosely defined as the
"rolling direction" of the wheel and the lateral direction as "parallel to
the wheel's rolling axis."
The axes can be defined mathematically for a wheel in contact with terrain
as follows:

* Define `W` as the center of a wheel with radius `R`.
* Define `w` as the body-fixed axis about which the wheel rotates
  (typically an axis of symmetry) with rotational speed `ω`.
* At each contact point `C` with normal direction `n`, define a wheel contact
  coordinate frame whose origin is `C` and one of the axes is `n`.
  If `w` is not parallel to `n`, then the lateral wheel contact direction
  `t_lat` is defined by projecting `w` into the contact tangent plane of `C`
  and normalizing to unit length. The longitudinal wheel contact direction
  `t_lon` is then computed directly as `t_lon = n \cross t_lat`.
* The wheel contact coordinate frame has its origin at `C` with orthogonal
  axis directions `n`, `t_lat`, and `t_lon`.

The translational slip at `C` in units of `m/s` is given as `V_C`, with
components in each tangent plane direction defined as:

* Longitudinal slip velocity `V_C,lon`
* Lateral slip velocity `V_C,lat`

### Definition of nondimensional longitudinal wheel slip

The longitudinal slip velocity is nondimensionalized by dividing by one of two
measured velocities, depending on the slip condition.
For illustration, consider the case when the wheel center `W` lies a distance
`R` from the contact point `C` along normal direction `n` (typical of contact
with a flat surface).
In this case, the longitudinal slip velocity can be expressed as the difference
between the longitudinal wheel center velocity and the linear "spin speed" of
the wheel `V_C,lon = V_W,lon - R*ω`.

In Yoshida and Hamano, 2002
(DOI: [10.1109/ROBOT.2002.1013712](https://dx.doi.org/10.1109/ROBOT.2002.1013712)),
the two slip conditions used for defining nondimensional longitudinal wheel
slip are

* `R * ω > V_W,lon`: the wheel is spinning faster than its translational speed
  and is accelerating.
* `R * ω < V_W,lon`: the wheel is spinning slower than its translational speed
  and is braking.

Nondimensional longitudinal wheel slip `s` is defined as follows for each
slip condition:

* Accelerating: `s = (R * ω - V_W,lon) / (R * ω)`
* Braking: `s = (R * ω - V_W,lon) / (V_W,lon)`

In each case, the numerator is the negative of longitudinal slip velocity
`-V_C,lon`, and the denominator varies.

When the wheel is rolling without slip (`V_C,lon = 0`), the longitudinal wheel
center velocity matches the linear "spin speed" `V_W,lon = R * <D-c>ω` and `s = 0`.

During acceleration, if the wheel is turning (`ω > 0`) but is not producing any
forward motion relative to the ground (`V_W,lon = 0`), then `s = 1`.

During braking, if the brakes are fully engaged(`ω = 0`) but the wheel keeps
moving forward (`V_W,lon > 0`), then `s = -1`.

Using the slip definition from [Yoshida and Hamano, 2002](https://dx.doi.org/10.1109/ROBOT.2002.1013712),
the nondimensional longitudinal slip `s` is in the interval [-1, 1] provided
that `R * ω` and `V_W,lon` are both positive.
The definition of `s` can be generalized to account for driving in reverse as
follows:

* if `|R * ω| > |V_W,lon|`, Accelerating: `s = (R * ω - V_W,lon) / (R * ω)`
* if `|R * ω| < |V_W,lon|`, Braking: `s = (R * ω - V_W,lon) / (V_W,lon)`

With this generalized definition,
the nondimensional longitudinal slip `s` is in the interval [-1, 1] provided
that `R * ω` and `V_W,lon` have the same sign (`R * ω` and `V_W,lon >= 0`).
However when the velocity values have different signs, the magnitude of `s` is
greater than 1. For example, when attempting to drive up a steep slope with
`R * ω > 0`, if the vehicle actually slides backwards with `V_W,lon < 0`, the
slip value will be greater than 1.

### Definition of nondimensional lateral wheel slip

The lateral slip velocity is represented in nondimensional form by projecting
the wheel center velocity into the contact tangent plane and computing the angle
α between the projected velocity vector and the longitudinal wheel contact
direction: `α = arctan(V_W,lat / V_W,lon)`.

## Friction modeling for wheels

When simulating wheel-terrain contact and friction, it is common to model
wheel-terrain friction forces acting in the longitudinal and lateral wheel
directions as functions of the nondimensional slip variables `s` and `α`.
See Brach & Brach, 2009 (DOI: [10.4271/2009-01-0102](https://dx.doi.org/10.4271/2009-01-0102))
for an example of use in tire modeling for accident reconstruction and
Yoshida and Hamano, 2002
(DOI: [10.1109/ROBOT.2002.1013712](https://dx.doi.org/10.1109/ROBOT.2002.1013712))
for an example in modeling tire-soil interaction of interplanetary rovers.

### Wheel-fixed friction directions

To specify wheel-fixed friction directions in the longitudinal and lateral
directions, follow the approach used to define the longitudinal and lateral
directions in the wheel contact coordinate frame by setting the `fdir1`
parameter to be parallel to the revolute joint axis corresponding to the wheel
axle. With this approach, specify lateral friction / slip parameters in first
friction direction and longitudinal parameters in 2nd direction.

### WheelSlipPlugin / WheelSlip system

Since the slip compliance parameters used by Gazebo's physics engines apply
forces based on slip velocity rather than nondimensional slip, Gazebo's
wheel slip plugin ([WheelSlipPlugin](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/WheelSlipPlugin.hh)
in gazebo-classic and the [WheelSlip system](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/wheel_slip/WheelSlip.hh)
in gz-sim have been written to dynamically update the dimensional slip
compliance parameters to emulate the use of nondimensional slip.

Document wheel slip plugin

## Contact pressure distributions

deformability
https://github.com/gazebosim/gazebo-classic/issues/3084
https://github.com/gazebosim/gazebo-classic/issues/3085

### Approximate deformable wheels with rolling friction

### Approximate deformable terrain with plowing angle

by changing contact point, normal

and changing slip compliance based on gravity component in contact tangent plane
