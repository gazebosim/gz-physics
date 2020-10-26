\page physicsconcepts Ignition Physics simulation concepts

This tutorial introduces simulation concepts that are used in Ignition Physics.
As the SDFormat is used to describe simulation scenarios in Ignition, the
Ignition Physics simulation concepts are described closely related to the SDFormat.

## Prerequisites

In the previous tutorial \ref installation "Installation", you have installed
the Ignition Physics corresponding to the desired Ignition release. Note that
the recommended Ignition release is Dome.

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

Ignition adopts SDFormat structure to describe not only the visual element but
also the dynamic physics aspects.

## Physics concepts in Ignition Gazebo simulation

First, please see [Understand the GUI tutorial](https://ignitionrobotics.org/docs/dome/gui)
for an overview of Ignition Gazebo GUI. In this tutorial, we will show how to
manipulate and visualize some physics aspects of the differential car model
in Ignition Gazebo.

### Manipulating the model

Please download the differential car model to your home folder by:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/main/examples/worlds/diff_drive.sdf -P ~
```

We can now start our differential car simulation on Ignition Gazebo by:

```bash
ign gazebo ~/diff_drive.sdf
```

On the Ignition Gazebo GUI, manipulating the model is easy. In general, there
are two ways to transform the model pose.

#### Use Transform control

For translation movement, we can enter into translate mode by clicking the
second icon from the left in the top left toolbar, the Transform Control plugin,
or by pressing the keyboard shortcut: T. The red arrow represents the x-axis,
green the y-axis, and blue the z-axis. Click and hold on any of the arrows
while moving your mouse to move the entity in that direction.

We can hold down any one of the X, Y, or Z keys, or a combination of them, while
clicking and dragging will constrain the model's movement along those axes as follow:

<img src="https://user-images.githubusercontent.com/18066876/97185768-c018f780-17a0-11eb-9580-a662a578030f.gif"/>

For rotation movement, we select the third icon from the left in the top left
toolbar,  the Transform Control plugin, or by pressing the keyboard shortcut: R.
The red circle represents roll, green is pitch, and blue is yaw. Click and hold
on any of the circles while moving your mouse to rotate the entity around that axis.

<img src="https://user-images.githubusercontent.com/18066876/97185874-e63e9780-17a0-11eb-955b-20200c3be4e4.gif"/>

#### Input model pose on Component Inspector

If exact pose input is required, we select the car model
(e.g. `vehicle_blue`), then select the `Pose` drop down list. We can input the
desired pose, for example `X: 0.0, Y: 1.0, Z: 2.0, Roll: 1.0, Pitch: 0.0, Yaw: 1.0`
as follow:

<img src="https://user-images.githubusercontent.com/18066876/97185954-fe161b80-17a0-11eb-8777-9e667889c468.gif"/>

Regardless of the simulation is pausing or not, the car model will jump to the
desired pose. For more detail about specifying pose, see this [tutorial](http://sdformat.org/tutorials?tut=specify_pose&cat=specification&).

### Monitoring model pose

To see the pose changing when the simulation running, we publish a
\ref ignition::msgs::Twist "Twist" message to command the `vehicle_blue` car to
move in a circle with 2 meters radius as follow:

```bash
ign topic -t "/model/vehicle_blue/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 1.0}, angular: {z: 0.5}"
```

This command tells the car to move in its coordinate frame with velocity
1.0 meter per second in the X-axis and angular velocity 0.5 radians per second in
Z-axis. We can monitor the model pose by selecting the moving car and then
select the drop-down list `Pose`. Moreover, we could also read the model
links' poses relative to their parent link by selecting the
corresponding link on the model tree:

<img src="https://user-images.githubusercontent.com/18066876/97189395-d45ef380-17a4-11eb-9c29-cf33b5b184c4.gif"/>

Note that using the model tree as shown in the above gif, we can view the
parameters and properties such as `visual` or `collision` of each link or joint
in the model. For more detail about the kinematics of the model tree, see this [tutorial](http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification&).

### Visualizing collision

One of the most wonderful features of physics simulation is the capability to
simulate collision. We can do a fun experiment like this: while the `vehicle_blue`
car is moving in a circle, we move the `vehicle_green` to be on `vehicle_blue`'s
upcoming path. We will see the blue car will push the green car!

<img src="https://user-images.githubusercontent.com/18066876/97190565-28b6a300-17a6-11eb-9ed5-0d6d50c15ad4.gif"/>

To see where these collision parameters are set and how it works, please see
this [tutorial](http://sdformat.org/tutorials?tut=spec_shapes&cat=specification&).

## Physics concepts in SDFormat

In this section, we will go into detail about the model parameters and
explain the physics concepts which correspond to SDFormat defining simulation.

### SDFormat structure

In general, an SDFormat file defining a simulation scenario has the following
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

The next level of the root element `sdf` specifying the SDFormat version
is the `world` element, which encapsulates entire simulation scenarios
description including these elements:
- `physics`: specifies the type of physics engines for dynamic simulation (in
  our case they are `TPE` or `dartsim`).
- `scene`: specifies the view of the environment.
- `model`: defines a complex robot structure or an object. Note there can be
  multiple `model` inside `world` tag, which enables multi-body simulation.
- `light`: describes light source in simulation scenarios.

### Physics simulation concepts corresponding to the SDFormat

Looking closer to the `model` tag, an example structure defining the above
diffential driver car is described below with comments:

```xml
<model name="car">
    <pose>0 0 0.5 0 0 0</pose> <!--specifies where the car pose in world frame-->
    <static>false</static> <!--specifies whether the care is static (unmovable) or not-->

    <link name="chassis"> <!--specifies a component of a car with physics properties-->
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
car SDFormat model. In general, every model is a group of `link`s (can be just one
link) connected with `joint`s. Note that a `model` element can be
nested in another `model`. Finally, the `plugin` element could be defined in
`world`, `model`, and `sensor` tags for dynamically loading a demanded physics
engine that simulates the dynamic model.

To get more in-depth of what you can define the environment in the SDFormat file,
please refer to this [SDFormat specification](http://sdformat.org/spec?ver=1.7&elem=sdf).

For a comprehensive tutorial for constructing your robot model, please refer
to [this tutorial](https://ignitionrobotics.org/docs/dome/building_robot).
