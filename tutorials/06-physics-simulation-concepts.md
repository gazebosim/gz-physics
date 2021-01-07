\page physicsconcepts Ignition Physics simulation concepts

This tutorial introduces simulation concepts that are used in Ignition Physics.

## Prerequisites

In the previous tutorial \ref installation "Installation", you have installed
the Ignition Physics corresponding to the desired Ignition release.

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
also the dynamic physics aspects. To get more in-depth of what you can define
the environment in the SDFormat file, please refer to this
[SDFormat specification](http://sdformat.org/spec?ver=1.7&elem=sdf).
For a comprehensive tutorial for constructing your robot model as SDFormat,
please refer to [this tutorial](https://ignitionrobotics.org/docs/dome/building_robot).


## Physics concepts in Ignition Gazebo simulation

First, please see [Understand the GUI tutorial](https://ignitionrobotics.org/docs/dome/gui)
for an overview of Ignition Gazebo GUI. In this tutorial, we will show how to
manipulate and visualize some physics aspects using interesting
models powered by Ignition Physics on Ignition Gazebo.

### Differential Drive

This demo world demonstrates how we can control simulated differential drive on
cars depending on physics engines and visualize the collision concept. Please
download the Differential Drive simulation world demo to your home folder by:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/main/examples/worlds/diff_drive.sdf -P ~
```

We can now start our differential drive simulation on Ignition Gazebo by:

```bash
ign gazebo ~/diff_drive.sdf
```

To see the pose changing when the simulation running, we publish a
\ref ignition::msgs::Twist "Twist" message to command the `vehicle_blue` car to
move in a circle with 2 meters radius as follow:

```bash
ign topic -t "/model/vehicle_blue/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 1.0}, angular: {z: 0.5}"
```

Then please press the Play button to start the simulation.
This command tells the car to move in its coordinate frame with velocity
1.0 meter per second in the X-axis and angular velocity of 0.5 radians per
second in Z-axis.

Note that the mechanism to move the car is different depending on the used physics
engine. Using [dartsim](https://github.com/ignitionrobotics/ign-physics/tree/master/dartsim),
the car is moved by applying force on the joints, please see [DiffDrive.cc](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/diff_drive/DiffDrive.cc#L333) code.
Using [TPE](https://github.com/ignitionrobotics/ign-physics/tree/main/tpe),
TPE directly sets model velocity in [VelocityControl.cc](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/velocity_control/VelocityControl.cc#L117) code.

#### Monitoring the model and its links pose

We can monitor the model pose by selecting the moving car and then
select the drop-down list `Pose`. Moreover, we could also read the model
links' poses relative to their parent link by selecting the
corresponding link on the model tree:

<img src="https://user-images.githubusercontent.com/18066876/97189395-d45ef380-17a4-11eb-9c29-cf33b5b184c4.gif"/>

Note that using the model tree as shown in the above gif, we can view the
parameters and properties such as `visual` or `collision` of each link or joint
in the model. For more detail about the kinematics of the model tree, see this
[tutorial](http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification&).

#### Visualizing collision

One of the most wonderful features of physics simulation is the capability to
simulate collision. We can do a fun experiment like this: while the `vehicle_blue`
car is moving in a circle, we move the `vehicle_green` to be on `vehicle_blue`'s
upcoming path. We will see the blue car will push the green car!

<img src="https://user-images.githubusercontent.com/18066876/97190565-28b6a300-17a6-11eb-9ed5-0d6d50c15ad4.gif"/>

To see where these collision parameters are set in SDFormat and how it works,
please see this [tutorial](http://sdformat.org/tutorials?tut=spec_shapes&cat=specification&).

### Lift Drag

This demo world shows how joint force, torque, and pressure are supported in
Ignition Physics. Please download the Lift Drag demo to your home folder by:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/main/examples/worlds/lift_drag.sdf -P ~
```

Like above, please start the Lift Drag demo world on Ignition Gazebo by:

```bash
ign gazebo ~/lift_drag.sdf
```

To see how the rotor lifts the cube due to wind force pressure, we publish a
\ref ignition::msgs::Double "Double" message represeting the torque (Nm) applying to
the rotor rod axis as follow:

```bash
ign topic -t "/model/lift_drag_demo_model/joint/rod_1_joint/cmd_force" -m ignition.msgs.Double  -p "data: 0.7"
```

Then please press Play button to start the simulation. We stops exerting torque by:

```bash
ign topic -t "/model/lift_drag_demo_model/joint/rod_1_joint/cmd_force" -m ignition.msgs.Double  -p "data: 0.0"
```

You will see the cube drops due to no lift force from support torque on the rod,
and the blades will stop after some time due to friction.

<img src="https://user-images.githubusercontent.com/18066876/99107715-8d675f80-25e6-11eb-91f3-90f83bbc93e9.gif"/>

The command applies a constant torque to the rotor rod, together with
the mechanism to compute the upward/downward lift and drag force due to the
wind pressure simulation supported by Ignition Physics, the cube is lifted.
For more detail, please see the [LiftDrag.cc](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/lift_drag/LiftDrag.cc)
code.

### Buoyancy

This demo world shows how buoyancy is supported in Ignition Physics. This world
contains the following three models:

  1. submarine: A simple submarine model that floats in place.
  2. submarine_sinking: A simple submarine model that is not buoyant and sinks.
  3. submarine_buoyant: A simple submarine model that is buoyant and floats.

Please download the Buoyancy demo to your home folder by:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/ign-gazebo4/examples/worlds/buoyancy.sdf -P ~
```

Like above, please start the Buoyancy demo on Ignition Gazebo by:

```bash
ign gazebo ~/buoyancy.sdf
```

After pressing the Play button, you will see the behaviors of the submarine as
the above description.

<img src="https://user-images.githubusercontent.com/18066876/99107675-7c1e5300-25e6-11eb-9b6c-2745c894c1ba.gif"/>

As an overview, the buoyancy concept is realized by
simulating fluid density and applying the force on the object in the fluid
proportional to its volume. Hence, you can change the model buoyancy by modifying its
inertia, please see [link specification](http://sdformat.org/spec?ver=1.7&elem=link).
For more detail on simulating buoyancy, please see the
[Buoyancy.cc](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/buoyancy/Buoyancy.cc)
code.

### Pendulum

This demo world demonstrates how simulated inertia and gravity affect the object
movement by showing free swing of the pendulum. Please download the
Pendulum demo to your home folder by:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/main/examples/worlds/video_record_dbl_pendulum.sdf -P ~
```

and start the Pendulum demo on Ignition Gazebo by:

```bash
ign gazebo ~/video_record_dbl_pendulum.sdf
```

After pressing the Play button, you will see that the pendulum will oscillate around
its main revolute joint forever due to lack of friction (it is undeclared in the
SDFormat file).

<img src="https://user-images.githubusercontent.com/18066876/99107609-69a41980-25e6-11eb-8792-7c348e07b4c0.gif"/>

The oscillation period or the max angular speed of the joint
will change if we modify the inertia of the rods. According to the demo below,
you will see that gravity is defined as -9.8 m/s^2 on the Z-axis. Please refer to
[link specification](http://sdformat.org/spec?ver=1.7&elem=link) for modifying
the inertia and mass of the links.

### Multicopter

This demo world shows how Ignition Physics supports gravity, actuators and
inertia to control object velocity.
Please download the Multicopter demo to your home folder by:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/ign-gazebo4/examples/worlds/multicopter_velocity_control.sdf -P ~
```

and start the Multicopter demo on Ignition Gazebo by:

```bash
ign gazebo ~/multicopter_velocity_control.sdf
```

To control the multicopter to ascend and hover, we send a
\ref ignition::msgs::Twist "Twist" message to command the `X3` multicopter
ascending 0.1 m.s as follow:

```bash
ign topic -t "/X3/gazebo/command/twist" -m ignition.msgs.Twist -p "linear: {x:0 y: 0 z: 0.1} angular {z: 0}"
```

then hovering:

```bash
ign topic -t "/X3/gazebo/command/twist" -m ignition.msgs.Twist -p " "
```

<img src="https://user-images.githubusercontent.com/18066876/99107458-2f3a7c80-25e6-11eb-8cff-40fbc1036d1f.gif"/>

Do the same for the `X4` multicopter. After pressing the Play button, you will see
both of the multicopters will ascend, this demonstrates how the physics engine
utilizes model kinematics and dynamics to support simulating complex model and
its controller. For more detail about the multicopter controller, please see
[MulticopterVelocityControl.cc](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/multicopter_control/MulticopterVelocityControl.cc).
