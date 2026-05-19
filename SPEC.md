# gz_sim_spray_painting_plugin — Specification Sheet

**Version:** 0.3.0  
**License:** Apache-2.0  
**Date:** 2026-04-24

---

## 1. Project Overview

A Gazebo Sim 8 (Harmonic) + ROS 2 Humble simulation of a UR5e spray-painting robot.
The project has two layers:

1. **SprayPaintPlugin** (`libSprayPaintPlugin.so`) — a Gazebo system plugin that projects a
   cone-shaped paint volume from a configurable nozzle link. When the spray trigger is active:
   - Casts the spray-axis ray from the nozzle tip.
   - Ray-tests every visual in the scene (BOX / SPHERE / CYLINDER supported) and computes the
     exact hit point and surface normal.
   - Spawns a thin circular paint disc (CYLINDER slab) at the hit location, parented to the
     visual's link so it moves with the object.
   - Drives a particle emitter on the nozzle link to depict the spray cone.
   - Paint discs accumulate with spatial deduplication (2 cm min spacing).

2. **UR5e demo stack** — a complete ROS 2 / MoveIt 2 robot demo:
   - UR5e arm with a spray gun end-effector (`ur_spray_gz.urdf.xacro`)
   - `gz_ros2_control` hardware interface (position control, 500 Hz)
   - MoveIt 2 (`move_group` + RViz) with OMPL planning
   - Cartesian raster-scan executor (`cartesian_path_executor.py`)

| Property | Value |
|---|---|
| Package name | `gz_sim_spray_painting_plugin` |
| Plugin shared library | `libSprayPaintPlugin.so` |
| Plugin alias | `gz::sim::systems::SprayPaintPlugin` |
| Gazebo version | Harmonic (gz-sim8) |
| ROS 2 version | Humble |
| Language standard | C++17 |
| Build system | ament_cmake + colcon |
| Robot arm | Universal Robots UR5e |
| MoveIt planning group | `ur_manipulator` |
| End-effector / tip link | `spray_gun_nozzle_link` |

---

## 2. Directory Structure

```
gz_sim_spray_painting_plugin/          ← colcon workspace root
├── src/
│   ├── gz_sim_spray_painting_plugin/  ← main plugin package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/
│   │   │   └── gz_sim_spray_painting_plugin/
│   │   │       └── SprayPaintPlugin.hh
│   │   ├── src/
│   │   │   └── SprayPaintPlugin.cc
│   │   ├── urdf/
│   │   │   ├── spray_nozzle.urdf          # Legacy 2-DOF demo arm
│   │   │   └── ur_spray_gz.urdf.xacro    # UR5e + spray gun (active)
│   │   ├── worlds/
│   │   │   ├── spray_painting.sdf         # Legacy world
│   │   │   └── ur_spray_painting.sdf     # UR5e demo world (active)
│   │   ├── launch/
│   │   │   ├── gz_sim.launch.py          # Start Gazebo + world + clock bridge
│   │   │   ├── ur_spray_demo.launch.py   # Spawn UR5e + controllers + MoveIt
│   │   │   └── spawn_robot.launch.py     # Legacy spawn launch
│   │   └── config/
│   │       └── ros_gz_bridge.yaml        # /clock bridge config
│   ├── ur_simulation_gz/                 # UR Gazebo simulation helpers
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   │   ├── ur_sim_controllers.yaml   # controller_manager parameters
│   │   │   ├── kinematics.yaml           # MoveIt KDL kinematics
│   │   │   └── cartesian_poses.yaml      # Raster-scan waypoints
│   │   ├── launch/
│   │   │   └── cartesian_spray.launch.py # Cartesian executor launch
│   │   ├── scripts/
│   │   │   ├── cartesian_path_executor.py # Cartesian MoveIt demo
│   │   │   └── spray_painting_demo.py     # Legacy spray demo
│   │   └── urdf/
│   │       ├── ur_gz.urdf.xacro
│   │       └── ur_gz.ros2_control.xacro
│   ├── ur_description/                   # UR URDF/meshes (cloned from source)
│   └── gz_ros2_control/                  # gz_ros2_control (cloned from source)
├── docker/
│   └── entrypoint.sh
├── run_scripts/
│   ├── build_code.py                     # Build inside Docker
│   ├── run_stack.py                      # Launch full simulation stack
│   ├── run_docker.sh                     # Low-level Docker runner
│   └── container_tmux_setup.sh          # tmux session layout
├── Dockerfile                            # Main Docker image definition
├── .gitignore
├── file_logs/                            # Per-session spray_paint_*.log files
├── startScript.sh                        # Interactive local menu
└── SPEC.md                               # This document
```

---

## 3. Dependencies

### 3.1 Built from Source (in `src/`)

| Package | Branch | Purpose |
|---|---|---|
| `ur_description` | humble | UR URDF macros, meshes, config |
| `gz_ros2_control` | humble | Gazebo ↔ ros2_control hardware bridge |

Both are cloned into `src/` and built as part of the workspace. `gz_ros2_control` **must** be
built with `GZ_VERSION=harmonic` so it links against `gz-sim8`.

### 3.2 Installed via apt (in Docker image)

| Package | Purpose |
|---|---|
| `gz-harmonic` | Gazebo Harmonic runtime |
| `libgz-sim8-dev` + dev libs | Plugin build dependencies |
| `ros-humble-ros-base` | ROS 2 core |
| `ros-humble-moveit` | MoveIt 2 planning stack |
| `ros-humble-ur-moveit-config` | UR SRDF, OMPL config, joint limits |
| `ros-humble-ros2-control` | controller_manager, spawner |
| `ros-humble-ros2-controllers` | JointStateBroadcaster, JointTrajectoryController |
| `ros-humble-robot-state-publisher` | TF from URDF |
| `ros-humble-ros-gz-sim` | ROS ↔ Gazebo bridge (clock, etc.) |
| `ros-humble-ros-gz-bridge` | Topic bridge |
| `python3-colcon-common-extensions` | Build tool |
| `xacro` (pip) | URDF/xacro processing at launch time |

---

## 4. Plugin Architecture

### 4.1 System Interfaces Implemented

| Interface | Method | Role |
|---|---|---|
| `ISystemConfigure` | `Configure()` | One-time init; reads SDF params, subscribes to trigger topic, opens log file |
| `ISystemPreUpdate` | `PreUpdate()` | Per-sim-step: resolve nozzle, manage particle emitter, cast rays, spawn paint discs |

### 4.2 Plugin Registration

```cpp
GZ_ADD_PLUGIN(gz::sim::systems::SprayPaintPlugin,
              gz::sim::System,
              gz::sim::systems::SprayPaintPlugin::ISystemConfigure,
              gz::sim::systems::SprayPaintPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::SprayPaintPlugin,
                    "gz::sim::systems::SprayPaintPlugin")
```

### 4.3 Class: `gz::sim::systems::SprayPaintPlugin`

**Header:** [src/gz_sim_spray_painting_plugin/include/gz_sim_spray_painting_plugin/SprayPaintPlugin.hh](src/gz_sim_spray_painting_plugin/include/gz_sim_spray_painting_plugin/SprayPaintPlugin.hh)  
**Implementation:** [src/gz_sim_spray_painting_plugin/src/SprayPaintPlugin.cc](src/gz_sim_spray_painting_plugin/src/SprayPaintPlugin.cc)

#### Public Methods

| Signature | Description |
|---|---|
| `SprayPaintPlugin()` | Default constructor |
| `~SprayPaintPlugin() override = default` | Default destructor |
| `void Configure(entity, sdf, ecm, eventMgr)` | Reads SDF params; subscribes to trigger topic |
| `void PreUpdate(info, ecm)` | Resolves nozzle; manages emitter; ray-tests visuals; spawns paint discs |

#### Private Methods

| Signature | Description |
|---|---|
| `void OnSprayMsg(const gz::msgs::Boolean &)` | Atomically sets `sprayActive_` from trigger topic |
| `PaintPatch MakePatch(hitWorld, normalWorld, dist)` | Shared helper: builds a disc patch from world hit point + normal |
| `PaintPatch ComputePaintPatch(ro, rd, boxPose, boxSize)` | Ray-AABB slab test for box geometry |
| `PaintPatch ComputeSpherePatch(ro, rd, pose, radius)` | Analytical ray-sphere intersection |
| `PaintPatch ComputeCylinderPatch(ro, rd, pose, radius, length)` | Analytical ray-cylinder intersection (barrel + end-caps) |
| `std::string Timestamp()` | Returns `[HH:MM:SS.mmm]` for log lines |
| `void Log(level, step, msg)` | Writes timestamped line to gzmsg and log file |
| `void Log(step, msg)` | Convenience overload; level defaults to "INFO" |
| `void LogSection(title)` | Writes visual separator to log |

#### Member Variables

| Member | Type | Default | Description |
|---|---|---|---|
| `nozzleLink_` | `std::string` | `"nozzle_link"` | Name of the link used as spray origin |
| `coneHalfAngle_` | `double` | `0.2618` rad (15°) | Half-angle of the paint cone |
| `coneMaxRange_` | `double` | `3.0` m | Maximum spray reach |
| `sprayColor_` | `gz::math::Color` | `(1, 0.2, 0.1, 1)` | RGBA paint colour |
| `sprayTopic_` | `std::string` | `"/spray_paint/trigger"` | Trigger topic name |
| `particleLifetime_` | `double` | `0.75` s | Particle age before despawn |
| `particleRate_` | `double` | `100.0` /s | Particles emitted per second |
| `particleInitialSize_` | `double` | `0.015` m | Particle radius at birth |
| `sprayActive_` | `std::atomic<bool>` | `false` | Thread-safe spray state |
| `transportNode_` | `gz::transport::Node` | — | Gazebo transport subscriber node |
| `nozzleEntity_` | `gz::sim::Entity` | `kNullEntity` | Cached ECM entity ID of nozzle link |
| `robotModelEntity_` | `gz::sim::Entity` | `kNullEntity` | Parent model; its visuals are never painted |
| `eventMgr_` | `gz::sim::EventManager *` | `nullptr` | Pointer cached from Configure; required by `SdfEntityCreator` |
| `emitterEntity_` | `gz::sim::Entity` | `kNullEntity` | Particle emitter entity on the nozzle link |
| `lastEmitterState_` | `bool` | `false` | Tracks last emitting state to avoid redundant ECM updates |
| `nonPaintableSkipped_` | `unordered_set<Entity>` | empty | Permanently skipped visuals (unsupported geometry or robot body) |
| `patchCenters_` | `unordered_map<Entity, vector<Vector3d>>` | empty | Per-visual patch centres in link-local frame for spatial deduplication |
| `logFile_` | `std::ofstream` | — | Per-session log file handle |
| `debugDumped_` | `bool` | `false` | Reset on each spray-ON edge; triggers one full diagnostic dump |

---

## 5. SDF Plugin Configuration

Embed within the URDF `<gazebo>` block. The UR5e URDF uses:

```xml
<plugin filename="libSprayPaintPlugin.so"
        name="gz::sim::systems::SprayPaintPlugin">
  <nozzle_link>spray_gun_nozzle_link</nozzle_link>
  <cone_half_angle_deg>10</cone_half_angle_deg>
  <cone_max_range>0.8</cone_max_range>
  <spray_color>1.0 0.2 0.1 1.0</spray_color>
  <spray_topic>/spray_paint/trigger</spray_topic>
  <particle_rate>100</particle_rate>
</plugin>
```

| SDF Element | Type | Default | Unit | Description |
|---|---|---|---|---|
| `<nozzle_link>` | string | `nozzle_link` | — | Link name whose +X axis is the spray direction |
| `<cone_half_angle_deg>` | double | `15` | degrees | Half-angle of paint cone |
| `<cone_max_range>` | double | `3.0` | metres | Entities beyond this distance are skipped |
| `<spray_color>` | `R G B A` | `1.0 0.2 0.1 1.0` | [0,1] | RGBA paint colour for discs and particles |
| `<spray_topic>` | string | `/spray_paint/trigger` | — | `gz.msgs.Boolean` trigger topic |
| `<particle_rate>` | double | `100` | /second | Particle cloud density |

---

## 6. Spray Painting Algorithm

### 6.1 Per-Frame Flow (`PreUpdate`)

```
1. Nozzle validity
   ├── If nozzleEntity_ missing from ECM → clear state, re-resolve next frame
   └── If unresolved → scan ECM for Link with matching name
         ├── Walk up to parent model → record robotModelEntity_
         ├── Pre-populate nonPaintableSkipped_ with all visuals in the robot model
         └── Create particle emitter on the nozzle link

2. Particle emitter toggle
   └── On sprayActive_ state change → set ParticleEmitterCmd component

3. Early exit if !sprayActive_

4. Compute nozzle world pose; derive spray origin + axis (+X of nozzle link)

5. For every Visual entity in ECM:
   a. Skip if in nonPaintableSkipped_ (robot body or unsupported geometry)
   b. Skip if name starts with "paint_patch_" (own spawned slabs)
   c. Geometry dispatch → ComputeXxxPatch → PaintPatch{worldPose, size, valid}
   d. Range guard: skip if dist < 1 mm or dist > coneMaxRange_
   e. Spatial dedup: transform to link-local frame; skip if < 2 cm from existing centre
   f. Spawn paint disc via SdfEntityCreator; parent to visual's link
   g. Record patch centre in link-local frame
```

### 6.2 Geometry Intersectors

#### `MakePatch` — shared helper
| Step | Formula |
|---|---|
| Cone radius at hit | `r = max(dist × tan(coneHalfAngle_), 0.025)` |
| Disc centre | `hitWorld + normalWorld × 0.0025` |
| Disc orientation | `Quaternion::From2Axes(Z̃, normalWorld)` |
| Slab depth | 5 mm (prevents z-fighting) |

#### `ComputePaintPatch` — BOX (Ray-AABB slab test)
Transform ray to box-local frame; compute tMin/tMax per axis; record hit face and outward normal.

#### `ComputeSpherePatch` — SPHERE
Quadratic `at² + bt + c = 0`; take nearest positive root; normal = `(hit − center).Normalized()`.

#### `ComputeCylinderPatch` — CYLINDER
2-D quadratic for barrel (XY plane) + planar test for end-caps (z = ±L/2); take nearest valid t.

### 6.3 Patch Entity Creation
Paint discs are created with `SdfEntityCreator::CreateEntities()` then `SetParent()`.
This fires `events::NewEntity` → `SceneBroadcaster` → `RenderUtil` so the GUI renders in the same frame.  
Raw `ecm.CreateEntity()` does **not** fire this event and produces invisible patches.

### 6.4 Spatial Deduplication
Patch centres are stored in **link-local frame** so they follow moving objects correctly.  
Minimum spacing: **2 cm** (`kPatchSpacing = 0.02` m).

---

## 7. Particle Emitter

Automatically derived from cone geometry. Only `particle_rate` is user-configurable.

```
kSprayVelocity  = 2.0 m/s  (internal constant)
lifetime        = cone_max_range / kSprayVelocity
minVelocity     = kSprayVelocity × 0.9
maxVelocity     = kSprayVelocity × 1.1
coneRadius@max  = cone_max_range × tan(cone_half_angle)
scaleRate       = (coneRadius@max − 0.001) / lifetime
colorStart      = spray_color  (opaque)
colorEnd        = spray_color  (alpha = 0)
```

The emitter is placed 3 cm along the nozzle's +X (`pose="0.03 0 0 0 0 0"`). Gazebo's Ogre2
backend emits particles along the entity's local +X, which is the spray axis.

---

## 8. UR5e Robot Description — `ur_spray_gz.urdf.xacro`

**File:** [src/gz_sim_spray_painting_plugin/urdf/ur_spray_gz.urdf.xacro](src/gz_sim_spray_painting_plugin/urdf/ur_spray_gz.urdf.xacro)

The URDF is built at launch time via `xacro` and includes:

### Kinematic Chain

```
world (fixed anchor)
  └── base_link  (UR5e base)
        └── shoulder_pan_joint  (revolute)
              └── shoulder_link
                    └── shoulder_lift_joint  (revolute)
                          └── upper_arm_link
                                └── elbow_joint  (revolute)
                                      └── forearm_link
                                            └── wrist_1_joint  (revolute)
                                                  └── wrist_1_link
                                                        └── wrist_2_joint  (revolute)
                                                              └── wrist_2_link
                                                                    └── wrist_3_joint  (revolute)
                                                                          └── wrist_3_link
                                                                                └── tool0
                                                                                      └── finger_joint (fixed, rpy="0 -1.5708 0")
                                                                                            └── spray_gun_base_link
                                                                                                  └── nozzle_joint (fixed)
                                                                                                        └── spray_gun_nozzle_link  ← spray origin (+X)
```

**Orientation notes:**
- `finger_joint rpy="0 -1.5708 0"` maps `tool0` +Z (UR flange forward) → spray gun +X.
- `nozzle_joint xyz="0.12 0 0.1" rpy="0 -0.25 0"` places the nozzle 12 cm forward, 10 cm up,
  pitched ~14° downward for a natural spray angle.
- Both fixed joints use `<disableFixedJointLumping>` and `<preserveFixedJoint>` so the links
  remain distinct ECM entities (not merged by the URDF→SDF converter).

### ros2_control Block

```xml
<ros2_control name="ur" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <!-- 6 joints: position + velocity command; position + velocity + effort state -->
</ros2_control>
```

Initial joint positions (matching Gazebo `initial_value` and MoveIt initial_positions):

| Joint | Initial value |
|---|---|
| `shoulder_pan_joint` | 0.0 |
| `shoulder_lift_joint` | −1.5708 |
| `elbow_joint` | 1.5708 |
| `wrist_1_joint` | −1.5708 |
| `wrist_2_joint` | −1.5708 |
| `wrist_3_joint` | 0.0 |

### gz_ros2_control Plugin

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(arg simulation_controllers)</parameters>
    <ros><namespace/></ros>
  </plugin>
</gazebo>
```

`gz_ros2_control` internally force-sets `use_sim_time=true` on the controller_manager node and
stamps joint states with `_info.simTime` (Gazebo world simulation time).

---

## 9. Controller Configuration — `ur_sim_controllers.yaml`

**File:** [src/ur_simulation_gz/config/ur_sim_controllers.yaml](src/ur_simulation_gz/config/ur_sim_controllers.yaml)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    forward_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController
    forward_position_controller:
      type: position_controllers/JointGroupPositionController

joint_trajectory_controller:
  ros__parameters:
    joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
             wrist_1_joint, wrist_2_joint, wrist_3_joint]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
```

**Important:** Only standard `ros2_controllers` types are used. `ur_controllers` types
(e.g., `ur_controllers/GPIOController`) require `ros-humble-ur-robot-driver` and must be
avoided — they cause the controller_manager to hang during `load_controller`.

---

## 10. MoveIt 2 Configuration

### Kinematics — `kinematics.yaml`

**File:** [src/ur_simulation_gz/config/kinematics.yaml](src/ur_simulation_gz/config/kinematics.yaml)

```yaml
ur_manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
  tip_link: spray_gun_nozzle_link
```

The tip link is set to `spray_gun_nozzle_link` (not `tool0`) so MoveIt plans to the spray
nozzle position.

### SRDF Patch

The `ur.srdf.xacro` from `ur_moveit_config` is post-processed at launch time to replace
`tip_link="tool0"` with `tip_link="spray_gun_nozzle_link"` via regex substitution:

```python
srdf_str = re.sub(
    r'(<chain\b[^>]*\btip_link=")[^"]+(")',
    r'\1spray_gun_nozzle_link\2',
    srdf_str,
)
```

### MoveIt Parameters (in `ur_spray_demo.launch.py`)

| Parameter | Value |
|---|---|
| Planning pipeline | `ompl_interface/OMPLPlanner` |
| Default planner | `geometric::RRTConnect` |
| Trajectory controller | `joint_trajectory_controller` (default) |
| `scaled_joint_trajectory_controller` | disabled |
| `use_sim_time` | `true` |
| `allowed_start_tolerance` | `0.01` |
| `execution_duration_monitoring` | `false` |

Kinematics is loaded as a Python dict (not a `--params-file`) to avoid the
`Cannot have a value before ros__parameters` error:
```python
kinematics_yaml = load_yaml("ur_simulation_gz", "config/kinematics.yaml")
robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
```

---

## 11. Launch Files

### 11.1 `gz_sim.launch.py`

**Package:** `gz_sim_spray_painting_plugin`

Starts Gazebo Harmonic with the spray-painting world. Sets all required environment variables
for the current process (not via `docker run -e`, to avoid conflicts).

| Action | Details |
|---|---|
| `GZ_SIM_SYSTEM_PLUGIN_PATH` | spray plugin lib + gz_ros2_control lib |
| `GZ_SIM_RESOURCE_PATH` | spray plugin share parent + ur_description share |
| Default world | `ur_spray_painting.sdf` |
| Clock bridge | `ros_gz_bridge` with `ros_gz_bridge.yaml` (GZ→ROS /clock) |

Launch arguments: `world` (default `ur_spray_painting.sdf`), `headless` (default `false`).

### 11.2 `ur_spray_demo.launch.py`

**Package:** `gz_sim_spray_painting_plugin`

Spawns the UR5e robot into an already-running Gazebo world and starts all ROS 2 nodes.

**Startup sequence:**

| Time | Action |
|---|---|
| T + 0 s | `robot_state_publisher` starts (publishes TF from URDF) |
| T + 0 s | MoveIt `move_group` + RViz start |
| T + 5 s | **Gazebo world reset** (`/world/ur_spray_painting/control` reset service) — resets sim time to 0, removes previously spawned robots |
| T + 20 s | Robot spawned via `gz service /world/ur_spray_painting/create` (URDF→SDF conversion, ~58 s until controller_manager ready) |
| T + 35 s | `joint_state_broadcaster` spawner (waits up to 60 s for controller_manager) |
| T + 40 s | `joint_trajectory_controller` spawner (waits up to 60 s) |

**Clock synchronisation:** The world reset at T+5 s is critical. `gz_ros2_control` timestamps
joint states with Gazebo's accumulated `_info.simTime`. If Gazebo ran for 40+ minutes before
the robot was spawned, sim time is thousands of seconds ahead of the newly-started MoveIt
nodes, causing MoveIt to reject all trajectories with:
```
Requested time 2481.7, but latest received state has time 68.8
```
Resetting the world brings sim time back to ~0 for all nodes.

**Launch arguments:** `ur_type` (default `ur5e`).

### 11.3 `cartesian_spray.launch.py`

**Package:** `ur_simulation_gz`

Launches `cartesian_path_executor.py` with configurable parameters.

| Argument | Default | Description |
|---|---|---|
| `poses_file` | `ur_simulation_gz/config/cartesian_poses.yaml` | Waypoint file |
| `planning_group` | `ur_manipulator` | MoveIt group name |
| `eef_step` | `0.01` | Cartesian step size (m) |
| `velocity_scaling` | `0.3` | Velocity scale factor |
| `spray_enabled` | `true` | Whether to publish spray trigger |

---

## 12. Cartesian Path Executor — `cartesian_path_executor.py`

**File:** [src/ur_simulation_gz/scripts/cartesian_path_executor.py](src/ur_simulation_gz/scripts/cartesian_path_executor.py)

A ROS 2 Python executable that drives the UR5e through a raster-scan painting pass.

### Execution Flow

1. Reads waypoints from the YAML poses file.
2. Moves to the first pose in **joint space** (`move_group.go()`) to establish a known start.
3. Publishes `std_msgs/Bool(True)` to `/spray_paint/trigger` → spray ON.
4. Computes Cartesian path through all waypoints with `compute_cartesian_path()`.
5. Executes the trajectory.
6. Publishes `std_msgs/Bool(False)` → spray OFF.

### Raster-Scan Waypoints — `cartesian_poses.yaml`

**File:** [src/ur_simulation_gz/config/cartesian_poses.yaml](src/ur_simulation_gz/config/cartesian_poses.yaml)

8 poses forming 4 horizontal sweeps over a vertical panel at x ≈ 0.75 m:

| Row | Direction | y range | z |
|---|---|---|---|
| 1 | left → right | −0.35 → +0.35 | 0.72 |
| 2 | right → left | +0.35 → −0.35 | 0.57 |
| 3 | left → right | −0.35 → +0.35 | 0.42 |
| 4 | right → left | +0.35 → −0.35 | 0.27 |

Nozzle standoff: 0.15 m from panel (x = 0.60 m).
Orientation: 90° pitch about Y → spray gun +X pointing toward panel.

---

## 13. Simulation World — `ur_spray_painting.sdf`

**File:** [src/gz_sim_spray_painting_plugin/worlds/ur_spray_painting.sdf](src/gz_sim_spray_painting_plugin/worlds/ur_spray_painting.sdf)

| Plugin | Role |
|---|---|
| `gz-sim-physics-system` | DART physics, Bullet collision |
| `gz-sim-user-commands-system` | GUI interactive commands + world reset service |
| `gz-sim-scene-broadcaster-system` | Scene state → GUI |
| `gz-sim-sensors-system` (ogre2) | Keeps OGRE2 render thread alive for `PreRender` events |
| `gz-sim-particle-emitter-system` | Processes `ParticleEmitterCmd` each step |

| Parameter | Value |
|---|---|
| World name | `ur_spray_painting` |
| Physics engine | DART |
| Collision detector | Bullet |
| Max step size | 0.001 s |
| Real-time factor | 1.0 |

Static models: ground plane, paint panel (1.2 × 0.05 × 0.8 m at (0.75, 0, 0.4)).

---

## 14. Build & Install

### Build Commands

```bash
# Standard build (inside container or after sourcing ROS)
python3 run_scripts/build_code.py

# Manual colcon (GZ_VERSION required for gz_ros2_control)
source /opt/ros/humble/setup.bash
GZ_VERSION=harmonic colcon build \
  --packages-select ur_description gz_ros2_control ur_simulation_gz gz_sim_spray_painting_plugin \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

`GZ_VERSION=harmonic` is mandatory — without it, `gz_ros2_control` links against a wrong
Gazebo version and the system plugin fails to load.

### Install Paths

| Artifact | Install Path |
|---|---|
| `libSprayPaintPlugin.so` | `install/gz_sim_spray_painting_plugin/lib/gz_sim_spray_painting_plugin/` |
| World SDF | `install/gz_sim_spray_painting_plugin/share/.../worlds/` |
| URDF/xacro | `install/gz_sim_spray_painting_plugin/share/.../urdf/` |
| Launch files | `install/gz_sim_spray_painting_plugin/share/.../launch/` |
| gz_ros2_control system libs | `install/gz_ros2_control/lib/` |
| ur_description meshes | `install/ur_description/share/ur_description/` |

---

## 15. Docker Infrastructure

**Dockerfile:** [Dockerfile](Dockerfile)  
**Base:** `ubuntu:22.04`

### Runtime Environment Variables

| Variable | Value | Set by |
|---|---|---|
| `GZ_VERSION` | `harmonic` | Dockerfile ENV + `run_docker.sh` |
| `GZ_SIM_SYSTEM_PLUGIN_PATH` | `install/gz_sim_spray_painting_plugin/lib/...:install/gz_ros2_control/lib` | `gz_sim.launch.py` via `SetEnvironmentVariable` |
| `GZ_SIM_RESOURCE_PATH` | `install/.../share` parent dirs | `gz_sim.launch.py` via `SetEnvironmentVariable` |
| `DISPLAY` | Forwarded from host (GUI mode only) | `run_docker.sh` |

**Note:** `GZ_SIM_SYSTEM_PLUGIN_PATH` and `GZ_SIM_RESOURCE_PATH` are managed by
`gz_sim.launch.py`, not by `run_docker.sh`, to prevent conflicts between the Dockerfile
`ENV` defaults and the bind-mounted `install/`.

### Volume Mounts (`run_docker.sh`)

| Host Path | Container Path | Mode |
|---|---|---|
| Project root | `/ws` | read-only |
| `install/` (if exists on host) | `/ws/install` | read-write (overlays Docker image) |
| `file_logs/` | `/ws/file_logs` | read-write (log output) |
| `$HOME/.ros/` | `$HOME/.ros/` | read-write |
| `$HOME/.gz/` | `$HOME/.gz/` | read-write |

---

## 16. tmux Session Layout

`container_tmux_setup.sh` creates a `spray_paint` tmux session with three windows:

| Window | Pane | Split | Command | Auto-run |
|---|---|---|---|---|
| `gz_sim` | 0 | — | `ros2 launch gz_sim_spray_painting_plugin gz_sim.launch.py` | Yes |
| `gz_sim` | 1 | horizontal | `ros2 launch gz_sim_spray_painting_plugin ur_spray_demo.launch.py` | Yes |
| `cartesian_spray` | 0 | — | `ros2 launch ur_simulation_gz cartesian_spray.launch.py` | **No** (pre-typed) |
| `cartesian_spray` | 1 | horizontal | `ros2 topic echo /joint_states --once` | Yes |
| `spray_control` | 0 | — | `gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: true"` | **No** |
| `spray_control` | 1 | vertical | `gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: false"` | **No** |

### Full Runtime Flow

```
./startScript.sh → "Start Stack" → python3 run_scripts/run_stack.py
  └── run_docker.sh tmux_stack → Docker container
        └── container_tmux_setup.sh
              ├── gz_sim pane 0: Gazebo loads ur_spray_painting.sdf
              ├── gz_sim pane 1: ur_spray_demo.launch.py
              │     ├── T+5s:  world reset (sim time → 0)
              │     ├── T+20s: robot spawned
              │     ├── T+35s: joint_state_broadcaster activated
              │     ├── T+40s: joint_trajectory_controller activated
              │     └── T+0s:  move_group + RViz ready (waits for joint states)
              ├── cartesian_spray pane 0: (user presses Enter to start demo)
              │     └── cartesian_path_executor.py → MoveIt Cartesian path
              └── spray_control panes: manual trigger ON/OFF
```

---

## 17. Communication Interfaces

### Gazebo Transport Topics

| Topic | Direction | Message Type | Description |
|---|---|---|---|
| `/spray_paint/trigger` | → plugin | `gz.msgs.Boolean` | `data: true` = ON, `data: false` = OFF |
| `/clock` | → ROS | `gz.msgs.Clock` → `rosgraph_msgs/Clock` | Sim time bridge |

### ROS 2 Topics

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `move_group`, RViz |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | `move_group`, RViz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` | `move_group`, RViz |
| `/spray_paint/trigger` | `std_msgs/Bool` | `cartesian_path_executor.py` | gz bridge → plugin |

### Services

| Service | Type | Description |
|---|---|---|
| `/world/ur_spray_painting/create` | `gz.msgs.EntityFactory` | Spawn robot URDF into Gazebo |
| `/world/ur_spray_painting/control` | `gz.msgs.WorldControl` | Reset sim (called at T+5 s) |
| `/controller_manager/list_controllers` | ros2_control | Queried by spawners |

### Manual Spray Trigger

```bash
# Spray ON
gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: true"

# Spray OFF
gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: false"
```

---

## 18. Configuration Reference

| Parameter | Where Set | Type | Default |
|---|---|---|---|
| `nozzle_link` | URDF `<plugin>` | string | `spray_gun_nozzle_link` |
| `cone_half_angle_deg` | URDF `<plugin>` | double | `10` degrees |
| `cone_max_range` | URDF `<plugin>` | double | `0.8` m |
| `spray_color` | URDF `<plugin>` | RGBA | `1.0 0.2 0.1 1.0` |
| `spray_topic` | URDF `<plugin>` | string | `/spray_paint/trigger` |
| `particle_rate` | URDF `<plugin>` | double | `100` /s |
| `world` | `gz_sim.launch.py` arg | string | `ur_spray_painting.sdf` |
| `headless` | `gz_sim.launch.py` arg | bool | `false` |
| `ur_type` | `ur_spray_demo.launch.py` arg | string | `ur5e` |
| `poses_file` | `cartesian_spray.launch.py` arg | path | `cartesian_poses.yaml` |
| `velocity_scaling` | `cartesian_spray.launch.py` arg | double | `0.3` |
| `spray_enabled` | `cartesian_spray.launch.py` arg | bool | `true` |

---

## 19. Supported Geometry Types

| Geometry | Support | Method |
|---|---|---|
| `BOX` | Full | Ray-AABB slab test |
| `SPHERE` | Full | Quadratic ray-sphere |
| `CYLINDER` | Full | 2-D quadratic barrel + planar caps |
| `CAPSULE` | Not implemented | Future work |
| `ELLIPSOID` | Not implemented | Future work |
| `MESH` | Not implemented | Requires BVH / triangle traversal |
| `PLANE` | Skipped | Ground plane should not be painted |

---

## 20. Known Limitations & Notes

- **Clock synchronisation:** `gz_ros2_control` stamps joint states with Gazebo's
  accumulated `_info.simTime`. If Gazebo runs for minutes before the robot is spawned,
  sim time is ahead of newly-started MoveIt nodes. The world reset in `ur_spray_demo.launch.py`
  (T+5 s) mitigates this by resetting sim time to ~0 before spawn.

- **URDF→SDF conversion is slow:** Gazebo converts the URDF on spawn, taking ~58 s for the
  UR5e model. Controller spawners use `--controller-manager-timeout 60` to wait.

- **Mesh painting not supported:** Objects with mesh geometry are silently skipped.

- **No persistence:** Paint state is in-memory; restarting Gazebo resets all patches.

- **Single colour per plugin instance:** Multi-colour painting requires multiple plugin
  instances or a runtime colour topic.

- **Static `nonPaintableSkipped_`:** If the robot is re-spawned, this set is rebuilt
  automatically when the nozzle entity is lost and re-resolved.
