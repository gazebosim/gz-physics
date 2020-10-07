\page physicsconcepts Ignition Physics simulation concepts

This tutorial introduces simulation concepts that are used in Ignition Physics.
As the SDF model is used to describe simulation scenarios in Ignition, the
Ignition Physics simulation concepts are described closely related to the SDF model.

## Physics simulation versus animation

Dynamics simulators for robotics is more challenging than the ones used for
animating virtual characters due to these aspects:
- The forces, torques, frictions, etc. are not crucial in animation since the
law of physics can be violated.
- Real-time requirements and physical reality can be less constraining for
purely visual display.
- Several challenges for simulating complex physical structures, such as
humanoid robot:
  - Numerical instability in computing inverse kinematics, fluid flows, etc.
    prevents correct physics simulation.
  - Contact forces between bodies are hard to model due to their discontinuity
    characteristics, especially in the soft body case, may result in unrealistic
    contacts or physically unfeasible contact forces.

Ignition adopts SDF structure to describe not only the visual element but also
the dynamic physics aspects.

## Terminology

### SDF structure

In general, an SDF file defining a simulation scenario has the following
structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <physics type="dartsim">
      ...
    </physics>

    <scene>
      ...
    </scene>

    <model name="box">
      ...
    </model>

    <model name="sphere">
      ...
    </model>

    <light name="spotlight">
      ...
    </light>
  </world>
</sdf>
```

The next level of the root element `sdf` specifying the SDF format version
is the `world` element, which encapsulates entire simulation scenarios
description including these elements:
- `physics`: specifies the type of physics engines for dynamic simulation (in
  our case they are `TPE` or `dartsim`).
- `scene`: specifies the view of the environment.
- `model`: defines a complex robot structure or an object. Note there can be
  multiple `model` inside `world` tag, which enables multi-body simulation.
- `light`: describes light source in simulation scenarios.

### Physics simulation concepts corresponding to the SDF model

Looking closer to the `model` tag, an example structure defining a simple car is
described below with comments:

```xml
<model name="car">
    <pose>0 0 0.5 0 0 0</pose> <!--specifies where the car pose in world frame-->
    <static>false</static> <!--specifies whether the care is static (unmovable) or not-->

    <link name="chassic"> <!--specifies a component of a car with physics properties-->
      <inertial>
        <mass>1.0</mass> <!--specifies the link mass-->
        <inertia> <!--specifies the 3x3 rotational symmetric inertia matrix of the link -->
          <ixx>0.016</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.016</iyy>
          <iyz>0</iyz>
          <izz>0.016</izz>
        </inertia>
      </inertial>
      <visual name='visual'> <!--describes visual properties of the link for visualization-->
        <geometry> <!--describes visual shape-->
          <box> <!--could be box, cyclinder, sphere, polyline, heightmap, image, mesh, plane-->
            <size>2.0 1.0 0.5</size>
          </box>
        </geometry>
        <material>  <!--describes visual characteristics such as ambient, diffuse, specular-->
          <ambient>0.0 1 0.0 1</ambient>
          <diffuse>0.0 1 0.0 1 1</diffuse>
          <specular>0.0 1 0.0 1</specular>
        </material>
      </visual>
      <collision name='collision'> <!--describes collision properties of the link for physics simulation, could be different from visual properties-->
        <geometry> <!--describes collision shape-->
          <box>
            <size>2.0 1.0 0.5</size>
          </box>
        </geometry>
        <surface> <!--describes collision surface properties, such as contact softness and frictions, depending on supported physics engines-->
          ...
        </surface>
      </collision>
    </link>
    <link name="right-wheel">
      ...
    </link>
    ...

    <!--the joint element specifies the joint connecting two links with kinematics and dynamics properties,
    could be revolute, prismatic, fixed, ball, screw, universal-->
    <joint name='right_wheel_joint' type='revolute'>
      <pose relative_to='right_wheel'/> <!--specifies the joint pose relating to a link-->
      <parent>chassis</parent> <!--specifies parent link that connects to a child link-->
      <child>right_wheel</child> <!--specifies child link-->
      <axis> <!--specifies axis of rotation for revolute joint or axis of translation for prismatic joint-->
        <xyz expressed_in='__model__'>0 1 0</xyz> <!--x,y,z components of the normalized axis vector expressed in a frame-->
        <limit> <!--specifies the limit of the joint-->
          <lower>-1.79769e+308</lower>    <!--negative infinity-->
          <upper>1.79769e+308</upper>     <!--positive infinity-->
        </limit>
      </axis>
    </joint>

    <plugin filename="libMyPlugin.so" name="my_plugin"/>
</model>
```

Most of the physical properties are mentioned as comments above in the example
car SDF model. In general, every model is a group of `link`s (can be just one
link) connected with `joint`s. Note that a `model` element can be
nested in another `model`. Finally, the `plugin` element could be defined in
`world`, `model`, and `sensor` tags for dynamically loading a demanded physics
engine that simulates the dynamic model.

To get more in-depth of what you can define the environment in the SDF file, please
refer to this [SDF specification](http://sdformat.org/spec?ver=1.7&elem=sdf).
For a comprehensive tutorial for constructing your robot model, please refer
to [this tutorial](https://ignitionrobotics.org/docs/dome/building_robot).
