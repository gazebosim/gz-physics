# Tutorial 10: Friction Parameters for Physics Engines

This tutorial provides comprehensive documentation on friction parameters supported by Gazebo Physics engines. Understanding these parameters is crucial for creating realistic robot simulations with accurate contact behavior.

## Overview

Friction modeling in physics simulation is essential for realistic robot behavior, especially for:
- Wheeled and legged locomotion
- Manipulation and grasping
- Surface interactions
- Stability analysis

Gazebo Physics supports multiple physics engines (DART, Bullet, TPE), each with slightly different friction parameter implementations.

## Friction Parameters

### mu (Primary Friction Coefficient)

The `mu` parameter defines the primary friction coefficient in the first friction direction (typically the x-axis of the contact surface).

**SDF Example:**
```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>
    </ode>
  </friction>
</surface>
```

**Typical Values:**
- 0.0: Frictionless (ice-like)
- 0.3-0.5: Low friction (plastic on plastic)
- 0.7-1.0: Medium friction (rubber on concrete)
- 1.0-1.5: High friction (sticky surfaces)

**Engine Support:**
- DART: ✓ Fully supported
- Bullet: ✓ Fully supported
- TPE: ✓ Fully supported

### mu2 (Secondary Friction Coefficient)

The `mu2` parameter defines friction in the second friction direction (typically the y-axis of the contact surface). When `mu2` is 0, it defaults to the value of `mu`.

**SDF Example:**
```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>
      <mu2>0.5</mu2>
    </ode>
  </friction>
</surface>
```

**Use Cases:**
- Anisotropic friction surfaces (e.g., brushed metal)
- Directional grip patterns
- Tread patterns on wheels

**Engine Support:**
- DART: ✓ Fully supported
- Bullet: ✓ Fully supported
- TPE: ✓ Supported (may approximate)

### slip1 (Longitudinal Slip)

The `slip1` parameter defines the slip coefficient in the primary friction direction. Higher values allow more sliding before friction engages.

**SDF Example:**
```xml
<surface>
  <friction>
    <ode>
      <slip1>0.1</slip1>
    </ode>
  </friction>
</surface>
```

**Typical Values:**
- 0.0: No slip (ideal friction)
- 0.01-0.1: Low slip (dry surfaces)
- 0.1-0.5: Moderate slip (wet or slippery surfaces)

**Engine Support:**
- DART: ✓ Fully supported
- Bullet: ✓ Supported
- TPE: ⚠ Limited support

### slip2 (Lateral Slip)

The `slip2` parameter defines slip in the secondary friction direction.

**SDF Example:**
```xml
<surface>
  <friction>
    <ode>
      <slip1>0.1</slip1>
      <slip2>0.2</slip2>
    </ode>
  </friction>
</surface>
```

**Use Cases:**
- Simulating tire slip in vehicles
- Skid-steer robot modeling
- Sliding contacts

**Engine Support:**
- DART: ✓ Fully supported
- Bullet: ✓ Supported
- TPE: ⚠ Limited support

### fdir1 (Friction Direction 1)

The `fdir1` parameter defines a custom direction vector for the primary friction direction. This is useful for anisotropic surfaces or specialized contact models.

**SDF Example:**
```xml
<surface>
  <friction>
    <ode>
      <fdir1>1 0 0</fdir1>
    </ode>
  </friction>
</surface>
```

**Use Cases:**
- Directional friction surfaces
- Conveyor belt simulations
- Custom contact models

**Engine Support:**
- DART: ✓ Fully supported
- Bullet: ⚠ Partial support
- TPE: ✗ Not supported

**Note:** In DART, `fdir1` can be expressed in different frames using the `expressed_in` attribute (see below).

### expressed_in (Frame Specification)

The `expressed_in` attribute (DART-specific) specifies the reference frame in which `fdir1` is expressed.

**SDF Example:**
```xml
<surface>
  <friction>
    <dart>
      <fdir1 expressed_in="__model__">1 0 0</fdir1>
    </dart>
  </friction>
</surface>
```

**Common Frame Values:**
- `__model__`: Model frame
- `__world__`: World frame
- Link name: Specific link frame

**Engine Support:**
- DART: ✓ Fully supported
- Bullet: ✗ Not supported
- TPE: ✗ Not supported

## Complete SDF Example

Here's a complete example showing all friction parameters for a robot wheel:

```xml
<model name="robot_wheel">
  <link name="wheel">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.1</radius>
          <length>0.05</length>
        </cylinder>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.2</mu>
            <mu2>0.8</mu2>
            <slip1>0.05</slip1>
            <slip2>0.1</slip2>
            <fdir1>1 0 0</fdir1>
          </ode>
          <dart>
            <fdir1 expressed_in="__model__">1 0 0</fdir1>
          </dart>
        </friction>
      </surface>
    </collision>
  </link>
</model>
```

## Physics Engine Comparison

| Parameter | DART | Bullet | TPE |
|-----------|------|--------|-----|
| mu | ✓ Full | ✓ Full | ✓ Full |
| mu2 | ✓ Full | ✓ Full | ⚠ Approx |
| slip1 | ✓ Full | ✓ Full | ⚠ Limited |
| slip2 | ✓ Full | ✓ Full | ⚠ Limited |
| fdir1 | ✓ Full | ⚠ Partial | ✗ None |
| expressed_in | ✓ Full | ✗ None | ✗ None |

## Practical Robotics Use Cases

### 1. Wheeled Mobile Robots

For differential drive robots:
```xml
<mu>0.8</mu>
<mu2>0.6</mu2>
<slip1>0.05</slip1>
<slip2>0.1</slip2>
```

### 2. Legged Robots

For foot contacts:
```xml
<mu>1.0</mu>
<mu2>1.0</mu2>
<slip1>0.01</slip1>
<slip2>0.01</slip2>
```

### 3. Manipulator Grippers

For grasping surfaces:
```xml
<mu>1.5</mu>
<mu2>1.5</mu2>
<slip1>0.0</slip1>
<slip2>0.0</slip2>
```

## Troubleshooting

### Common Issues

1. **Robot sliding unexpectedly**
   - Increase `mu` and `mu2` values
   - Reduce `slip1` and `slip2` values
   - Check contact surface normals

2. **Simulation instability**
   - Very high friction values can cause instability
   - Try values between 0.5-1.5 for most applications
   - Adjust physics step size if needed

3. **Anisotropic friction not working**
   - Ensure `mu` and `mu2` have different values
   - Verify `fdir1` is correctly oriented
   - Check engine support for `fdir1`

## References

- [SDF Specification - Surface](https://sdformat.org/spec)
- [DART Physics Engine Documentation](https://dartsim.github.io/)
- [Bullet Physics Documentation](https://pybullet.org/)
- [Gazebo Physics Tutorial 04: Physics Engines](04_physics_engines.md)

## Contributing

If you find discrepancies in friction behavior across engines, please report them as issues in the [gz-physics repository](https://github.com/gazebosim/gz-physics/issues).

---

*This tutorial is part of the Gazebo Physics tutorial series. For installation instructions, see [Tutorial 02](02_installation.md).*
