# SO101 Robot Model Documentation

## Overview

The SO101 robotic arm model is defined using MuJoCo's MJCF (MuJoCo XML Format), derived from URDF (Unified Robot Description Format). The model consists of two main XML files that describe the robot's kinematics, dynamics, and visual appearance.

**Model Files**:
- `model/robot_from_urdf.xml` - Robot-only MJCF model
- `model/scene_urdf.xml` - Complete scene (robot + environment)
- `model/assets/` - 3D meshes (STL files)

## File Structure

```
model/
├── scene_urdf.xml              # Complete simulation scene
├── robot_from_urdf.xml         # Robot definition only
└── assets/                     # 3D geometry
    ├── *.stl                   # Mesh files (26 files)
    └── *.part                  # CAD reference files
```

---

## scene_urdf.xml

### Purpose

Top-level scene file that includes the robot and defines the simulation environment.

### Contents

```xml
<mujoco model="scene">
  <include file="robot_from_urdf.xml"/>  <!-- Robot -->

  <visual>...</visual>      <!-- Rendering settings -->
  <asset>...</asset>        <!-- Textures/materials -->
  <worldbody>
    <light>...</light>      <!-- Lighting -->
    <geom name="floor"/>    <!-- Ground plane -->
  </worldbody>
</mujoco>
```

### Visual Settings

```xml
<visual>
  <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />
  <rgba haze="0.15 0.25 0.35 1" />
  <global azimuth="160" elevation="-20" />
</visual>
```

**Parameters**:
- **Headlight**: Camera-attached light (diffuse, ambient, specular)
- **Haze**: Atmospheric effect (RGBA color)
- **Global**: Default camera view (azimuth=160°, elevation=-20°)

### Environment

#### Floor
```xml
<geom name="floor" size="0 0 0.05" pos="0 0 0" type="plane"
      material="groundplane" />
```

- **Type**: Infinite plane
- **Height**: 0.05m thickness
- **Material**: Checkerboard texture
- **Collision**: Enabled

#### Lighting
```xml
<light pos="0 0 3.5" dir="0 0 -1" directional="true" />
```

- **Position**: 3.5m above origin
- **Direction**: Downward (-Z)
- **Type**: Directional (like sunlight)

#### Textures
```xml
<texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
         width="512" height="3072" />
<texture type="2d" name="groundplane" builtin="checker"
         mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
         markrgb="0.8 0.8 0.8" width="300" height="300" />
```

---

## robot_from_urdf.xml

### Purpose

Complete robot definition including:
- Kinematic chain (links and joints)
- Dynamic properties (masses, inertias)
- Geometry (visual and collision meshes)
- Actuators (position-controlled servos)

### Model Hierarchy

```
base
└── shoulder (joint: shoulder_pan)
    └── upper_arm (joint: shoulder_lift)
        └── lower_arm (joint: elbow_flex)
            └── wrist (joint: wrist_flex)
                └── gripper (joint: wrist_roll)
                    └── moving_jaw (joint: gripper)
```

### Compiler Settings

```xml
<compiler angle="radian" meshdir="assets" autolimits="true"/>
```

- **angle**: All angles in radians
- **meshdir**: Mesh files in `assets/` folder
- **autolimits**: Compute joint limits from URDF

---

## Joint Specifications

### Joint Details

| Joint Name | Type | Range (rad) | Range (deg) | DOF |
|------------|------|-------------|-------------|-----|
| shoulder_pan | Hinge | -1.920 to 1.920 | -110 to 110 | 1 |
| shoulder_lift | Hinge | -1.745 to 1.745 | -100 to 100 | 1 |
| elbow_flex | Hinge | -1.690 to 1.690 | -97 to 97 | 1 |
| wrist_flex | Hinge | -1.658 to 1.658 | -95 to 95 | 1 |
| wrist_roll | Hinge | -2.744 to 2.841 | -157 to 163 | 1 |
| gripper | Hinge | -0.175 to 1.745 | -10 to 100 | 1 |

**Total DOF**: 6

### Joint Properties (STS3215 Class)

All joints use `sts3215` class defaults:

```xml
<joint damping="0.60" frictionloss="0.052" armature="0.028"/>
```

**Parameters**:
- **damping**: 0.60 N·m·s/rad (viscous damping)
- **frictionloss**: 0.052 N·m (dry friction)
- **armature**: 0.028 kg·m² (rotor inertia)

---

## Actuator System

### Position Actuators

All 6 joints controlled by position actuators:

```xml
<position class="sts3215" name="shoulder_pan" joint="shoulder_pan"
          forcerange="-3.35 3.35" ctrlrange="-1.91986 1.91986"/>
```

### Actuator Class (STS3215)

```xml
<position kp="998.22" kv="2.731" forcerange="-2.94 2.94"/>
```

**Control Law**:
```
τ = kp*(ctrl - qpos) + kv*(0 - qvel)
τ = clip(τ, forcerange[0], forcerange[1])
```

**Parameters**:
| Parameter | Value | Description |
|-----------|-------|-------------|
| kp | 998.22 | Position gain (stiffness) |
| kv | 2.731 | Velocity gain (damping) |
| forcerange | ±2.94 N·m | Torque limits (per class) |

### Per-Actuator Force Limits

```xml
<actuator>
  <position ... name="shoulder_pan"  forcerange="-3.35 3.35" ... />
  <position ... name="shoulder_lift" forcerange="-3.35 3.35" ... />
  <position ... name="elbow_flex"    forcerange="-3.35 3.35" ... />
  <position ... name="wrist_flex"    forcerange="-3.35 3.35" ... />
  <position ... name="wrist_roll"    forcerange="-3.35 3.35" ... />
  <position ... name="gripper"       forcerange="-3.35 3.35" ... />
</actuator>
```

All actuators: **±3.35 N·m** max torque.

---

## Link Properties

### Link Masses

| Link | Mass (kg) | Description |
|------|-----------|-------------|
| base | 0.147 | Base mount |
| shoulder | 0.100 | Shoulder rotation assembly |
| upper_arm | 0.103 | Upper arm link |
| lower_arm | 0.104 | Lower arm link |
| wrist | 0.079 | Wrist assembly |
| gripper | 0.087 | Gripper base |
| moving_jaw | 0.012 | Gripper jaw |

**Total Mass**: ~0.732 kg

### Inertia Tensors

Example (base link):
```xml
<inertial pos="0.0137179 -5.19711e-05 0.0334843"
          mass="0.147"
          fullinertia="0.000114686 0.000136117 0.000130364
                       -4.59787e-07 4.97151e-06 9.75275e-08"/>
```

**Components**:
- `pos`: Center of mass offset from link frame
- `mass`: Link mass (kg)
- `fullinertia`: 6D inertia tensor [Ixx Iyy Izz Ixy Ixz Iyz] (kg·m²)

---

## Geometry

### Mesh Files

26 STL mesh files in `model/assets/`:

**Structural Parts**:
- `base_so101_v2.stl`
- `base_motor_holder_so101_v1.stl`
- `upper_arm_so101_v1.stl`
- `under_arm_so101_v1.stl`
- `rotation_pitch_so101_v1.stl`
- `wrist_roll_pitch_so101_v2.stl`
- `wrist_roll_follower_so101_v1.stl`
- `moving_jaw_so101_v1.stl`

**Motor Components**:
- `sts3215_03a_v1.stl` (with horn)
- `sts3215_03a_no_horn_v1.stl` (no horn)

**Mounting**:
- `motor_holder_so101_base_v1.stl`
- `motor_holder_so101_wrist_v1.stl`
- `waveshare_mounting_plate_so101_v2.stl`

### Visual vs. Collision Geometry

```xml
<default class="visual">
  <geom type="mesh" contype="0" conaffinity="0" group="2"/>
</default>
<default class="collision">
  <geom group="3"/>
</default>
```

**Visual Geometry** (group 2):
- For rendering only
- No collision detection
- All structural parts

**Collision Geometry** (group 3):
- For physics/contact
- Selected parts only (shoulder, arms, wrist)
- Base has no collision (fixed to world)

### Materials

Golden/yellow color scheme:

```xml
<material name="base_so101_v2_material" rgba="1 0.82 0.12 1"/>
<material name="sts3215_03a_v1_material" rgba="0.1 0.1 0.1 1"/>
```

**Colors**:
- Structural parts: Gold (1.0, 0.82, 0.12, 1.0)
- Motors: Dark gray (0.1, 0.1, 0.1, 1.0)

---

## Coordinate Frames

### Base Frame

```xml
<body name="base" pos="0 0 0" quat="1 0 0 0">
  <site group="3" name="baseframe" pos="0 0 0" quat="1 0 0 0"/>
</body>
```

- **Position**: Origin (0, 0, 0)
- **Orientation**: Identity quaternion
- **Height**: Floor contact at Z=0

### End-Effector Frame

```xml
<site group="3" name="gripperframe"
      pos="-0.0079 -0.000218121 -0.0981274"
      quat="0.707107 -0 0.707107 -2.37788e-17"/>
```

Located in gripper body, used for:
- Task-space control
- Cartesian path planning
- Tool center point (TCP) reference

---

## Physics Parameters

### Default Settings

```xml
<default class="so101_new_calib">
  <joint damping="1" frictionloss="0.1" armature="0.005"/>
  <position kp="50"/>
  ...
</default>
```

**General defaults**:
- Joint damping: 1.0 N·m·s/rad
- Friction loss: 0.1 N·m
- Armature: 0.005 kg·m²
- Position kp: 50 (overridden by sts3215 class)

### Timestep

Not specified in XML, uses MuJoCo default: **0.002s** (500 Hz)

Can be modified in Python:
```python
m.opt.timestep = 0.001  # 1ms timestep
```

---

## Loading the Model

### Python Usage

```python
import mujoco

# Load complete scene
m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)

# Robot-only (no floor/lights)
m_robot = mujoco.MjModel.from_xml_path("model/robot_from_urdf.xml")
```

### Model Information

```python
print(f"Number of joints: {m.njnt}")        # 6
print(f"Number of DOFs: {m.nv}")            # 6
print(f"Number of actuators: {m.nu}")       # 6
print(f"Number of bodies: {m.nbody}")       # 8 (world + 7 links)
print(f"Timestep: {m.opt.timestep}")        # 0.002
```

---

## Model Modification

### Change Actuator Gains

```python
# Find actuator ID
act_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "shoulder_pan")

# Modify gains (requires recompilation or careful indexing)
# Safer: modify XML and reload
```

### Adjust Joint Limits

In XML:
```xml
<joint ... range="-2.0 2.0" .../>
```

Or in Python (after loading):
```python
jnt_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "shoulder_pan")
m.jnt_range[jnt_id] = [-2.0, 2.0]
```

### Change Colors

```xml
<material name="base_so101_v2_material" rgba="1 0 0 1"/>  <!-- Red -->
```

---

## Visualization Groups

MuJoCo visualization groups:

| Group | Content | Purpose |
|-------|---------|---------|
| 0 | World geometry (floor) | Scene |
| 1 | - | Unused |
| 2 | Visual meshes | Rendering |
| 3 | Collision meshes + frames | Physics |

**Toggle in viewer**:
- Press `0`, `1`, `2`, `3` to toggle groups
- Default: Groups 0 and 2 visible

---

## Backlash Modeling (Optional)

Commented-out section in XML:

```xml
<default class="backlash">
  <!-- +/- 0.5° of backlash -->
  <joint damping="0.01" frictionloss="0" armature="0.01" limited="true"
         range="-0.008726646259971648 0.008726646259971648"/>
</default>
```

**Purpose**: Model gear backlash (currently not used).

**If enabled**: Would add small passive joints between actuated joints and links.

---

## Derived from URDF

**Original source**: OnShape CAD → `onshape-to-robot` → URDF → MJCF

**Conversion notes**:
- Inertia tensors computed from CAD
- Mesh files exported from CAD assemblies
- Joint limits measured from physical robot
- Actuator parameters from STS3215 datasheet

---

## Model Validation

### Check Joint Limits

```python
for i in range(m.njnt):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
    lim = m.jnt_range[i]
    print(f"{name}: [{lim[0]:.3f}, {lim[1]:.3f}] rad")
```

### Check Total Mass

```python
import numpy as np
total_mass = np.sum(m.body_mass)
print(f"Total robot mass: {total_mass:.3f} kg")
# Expected: ~0.73 kg
```

### Verify Actuator Limits

```python
for i in range(m.nu):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    frc = m.actuator_forcerange[i]
    print(f"{name}: {frc[0]:.2f} to {frc[1]:.2f} N·m")
```

---

## Troubleshooting

### Model Won't Load

**Error**: `File not found: model/scene_urdf.xml`

**Solution**: Run from `simulation_code/` directory or use absolute paths.

### Meshes Missing

**Error**: `Cannot read STL file: assets/base_so101_v2.stl`

**Solution**: Ensure `meshdir="assets"` in `<compiler>` and files exist.

### Robot Falls Through Floor

**Cause**: Collision geometry disabled or floor not defined

**Solution**: Check `scene_urdf.xml` includes floor geom and robot has collision meshes.

### Unstable Simulation

**Cause**: Timestep too large, actuator gains too high

**Solution**:
- Reduce timestep: `m.opt.timestep = 0.001`
- Lower actuator kp/kv in XML

---

## Reference

### Key Entities

```python
# Bodies
"base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"

# Joints
"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"

# Actuators (same names as joints)

# Sites
"baseframe", "gripperframe"
```

### Typical Robot Pose

```python
home_pose = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
```

---

## Summary

**Model Characteristics**:
- **Type**: 6-DOF robotic arm
- **Mass**: ~0.73 kg
- **Actuators**: Position-controlled servos (STS3215)
- **Torque**: ±3.35 N·m per joint
- **Format**: MJCF (MuJoCo XML)
- **Source**: URDF (OnShape CAD)

**Files**:
- `scene_urdf.xml`: Complete scene
- `robot_from_urdf.xml`: Robot only
- `assets/*.stl`: Geometry meshes

**Usage**:
```python
m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)
# Ready to simulate!
```
