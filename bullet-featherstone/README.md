# Bullet Featherstone Physics Plugin

This component enables the use of this [Bullet Physics](https://github.com/bulletphysics/bullet3) `btMultiBody` Featherstone implementation.
The Featherstone uses minimal coordinates to represent articulated bodies and efficiently simulate them.
 
# Features

Originally tracked via: [Bullet featherstone meta-ticket](https://github.com/gazebosim/gz-physics/issues/423)

## High-level feature list

* Constructing SDF Models
* Constructing SDF Worlds
* Joint Types: 
  * Fixed
  * Prismatic
  * Revolute
  * Ball Joint
* Stepping the simulation world
* Applying external forces/torques
* Free Groups
* Contact sensor

## Physics features list

These are the specific physics API features implemented.

* Entity Management Features 
  * ConstructEmptyWorldFeature
  * GetEngineInfo
  * GetJointFromModel
  * GetLinkFromModel
  * GetModelFromWorld
  * GetShapeFromLink
  * GetWorldFromEngine
  * RemoveModelFromWorld
* FreeGroupFeatures
  * FindFreeGroupFeature
  * SetFreeGroupWorldPose
  * SetFreeGroupWorldVelocity
* Kinematics Features
  * LinkFrameSemantics
  * ModelFrameSemantics
  * FreeGroupFrameSemantics
* Joint Features
  * GetBasicJointState
  * SetBasicJointState
  * GetBasicJointProperties
  * SetJointVelocityCommandFeature
  * SetJointTransformFromParentFeature
  * AttachFixedJointFeature
  * DetachJointFeature
  * GetRevoluteJointProperties
  * GetPrismaticJointProperties
  * FixedJointCast
* Link Features
  * AddLinkExternalForceTorque
* Sdf Features
  * sdf::ConstructSDFModel
  * sdf::ConstructSDFWorld
  * sdf::ConstructSDFCollision
* Shapes Features
  * GetShapeBoundingBox
  * GetBoxShapeProperties
  * AttachBoxShapeFeature
  * GetCapsuleShapeProperties
  * AttachCapsuleShapeFeature
  * GetConeShapeProperties
  * AttachConeShapeFeature
  * GetCylinderShapeProperties
  * AttachCylinderShapeFeature
  * GetEllipsoidShapeProperties
  * AttachEllipsoidShapeFeature
  * GetSphereShapeProperties
  * AttachSphereShapeFeature
* Simulation Features
  * ForwardStep
  * GetContactsFromLastStepFeature
* World Features
  * Gravity

# Caveats

* All links _must_ have a collision element.

In order for links to be active, bullet checks for the presence of collision elements.
In the case that collisions aren't available, adding a small sphere at the link origin is sufficient (but won't get good collision behavior)

* All links/joints of a model must be constructed with the model

Additional links/joints cannot be added to a model after initial construction.

* Ellipse axis-aligned bounding box returns invalid values on Ubuntu Focal

Tracked via: https://github.com/gazebosim/gz-physics/issues/440

* Joint transmitted wrench doesn't accurately report forces/torques

Tracked via: https://github.com/gazebosim/gz-physics/pull/434
