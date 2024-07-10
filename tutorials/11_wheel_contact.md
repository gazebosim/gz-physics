\page wheelcontact Simulating wheel-terrain contact

This tutorial expands on the overview of contact, slip and friction with special
considerations for simulating wheel-terrain contact in Gazebo.

## Prerequisites

- \ref contactoverview "Overview of contact, slip, and friction"

## Choosing a terrain collision shape

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

## Choosing a wheel collision shape

One can choose a cylinder, ellipsoid, sphere or triangle mesh for a wheel
collision shape. Keys to consider are:

- The actual shape of the wheel you are trying to model
- The quality of contact points that will be generated with the chosen terrain
  shape

For example with a heightmap terrain shape in Gazebo-Classic using
Open Dynamics Engine (ODE), there are known issues with all of the mentioned
wheel shapes, but for spheres and heightmaps, the issue of a sphere falling
though a convex edge can be avoided if the radius of the sphere is larger than
the resolution of the heightmap grid.
  
## Nondimensional wheel slip

Two forms: accelerating and braking

wheel slip plugin

## Wheel-fixed friction directions

Set first friction direction parallel to revolute joint axis and specify
lateral friction / slip parameters in 1st direction and longitudinal
parameters in 2nd direction

## Contact pressure distributions

deformability
https://github.com/gazebosim/gazebo-classic/issues/3084
https://github.com/gazebosim/gazebo-classic/issues/3085

### Approximate deformable wheels with rolling friction

### Approximate deformable terrain with plowing angle

by changing contact point, normal

and changing slip compliance based on gravity component in contact tangent plane
