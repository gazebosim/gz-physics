\page physicsconcepts Ignition Physics simulation concepts

This tutorial introduces simulation concepts that are used in Ignition Physics.
As SDF model is used to describe simulation objects in Ignition, the Ignition
Physics simulation concepts are described closely related to SDF model.

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

Ignition Physics addresses these problems for physics simulation by introducing
the concepts below.  

## Terminology

### SDF structure


### Physics simulation concepts via SDF model
