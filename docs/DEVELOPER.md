# Developer guide - `so_arm101_urdf`

ROS 2 **Jazzy** description package for the **SO-ARM101** 6-DOF follower arm.
Bringup, hardware plugin, and cameras live in **`lucy_ros_packages`**.

**First contact with a physical SO-101:** follow
[REAL_HARDWARE_SETUP.md](REAL_HARDWARE_SETUP.md) (LeRobot calibrate → apply →
regenerate → `real:=true`). Do not enable torque until that guide is done.

## Architecture

Hardware YAML schema lives in `config/hardware/` (radians).  
Architecture index: [`lucy_ws/docs/architecture/README.md`](../../../docs/architecture/README.md) ·  
System schematic (do not duplicate here): [`lucy_ws/docs/architecture/overview.md`](../../../docs/architecture/overview.md).

---

## 1. Role of this package

| Concern | Lives in |
|---------|----------|
| Robot description (xacro, meshes, joint limits, collisions) | `description/` |
| ros2_control hardware interfaces (real / mock) | `description/ros2_control/` |
| Gazebo physics + generated plugin | `description/gazebo/` |
| Hardware mapping (board, actuators, encoder placeholders) | `config/hardware/active.yaml` |
| Controller parameters | `config/controllers.yaml` (generated) |
| RViz layout | `config/so_arm101_rviz.rviz` |
| Launches | `launch/` |

**Robot selection:** `robot_package:=so_arm101_urdf` or `LUCY_ROBOT_PACKAGE=so_arm101_urdf`.

Bringup and `lucy_config_pipeline` both read `config/control.launch.yaml` for the
entry xacro / base path / controllers file (do not hardcode `inmoov.urdf.xacro`).

**Control-panel 3D viewer:** meshes are served by `/mesh/get`. STL payloads use
`encoding=zlib_base64` (zlib then base64). Rebuild `lucy_msgs` +
`lucy_config_pipeline` and refresh the panel after mesh-service changes.

---

## 2. Layout

```text
so_arm101_urdf/
├── package.xml / CMakeLists.txt / version.txt
├── docs/
│   ├── DEVELOPER.md              # this file
│   └── REAL_HARDWARE_SETUP.md    # calibrate + first real bringup
├── launch/                 # control, gazebo, joint_preview, rviz, rviz_standalone
├── config/
│   ├── control.launch.yaml
│   ├── controllers.yaml    # generated
│   ├── so_arm101_rviz.rviz
│   └── hardware/
│       ├── active.yaml
│       ├── active_meta.yaml
│       └── configs/default.yaml   # stock template (not arm-specific)
├── description/
│   ├── urdf/robot.urdf.xacro
│   ├── robot_description/
│   │   ├── urdf/properties.xacro
│   │   ├── urdf/robot_description.urdf.xacro
│   │   └── meshes/stl/
│   ├── ros2_control/so_arm101_ros2_control.xacro   # generated
│   └── gazebo/
├── worlds/default.sdf
├── scripts/apply_lerobot_calibration.py
└── test/
```

---

## 3. Joints and hardware

| Joint | Typical URDF range (rad) | Servo |
|-------|--------------------------|-------|
| `Rotation` | ±1.92 | STS3215 |
| `Pitch` | ±1.75 | STS3215 |
| `Elbow` | −1.81 … 1.59 | STS3215 |
| `Wrist_Pitch` | ±1.66 | STS3215 |
| `Wrist_Roll` | ±2.79 | STS3215 |
| `Jaw` | −0.17 … 1.75 | STS3215 |

After `apply_lerobot_calibration.py`, URDF `lower`/`upper` are overwritten from
that arm's measured travel (joint space). Table values above are stock defaults.

Kinematics come from the calibrated `so101_new_calib.urdf` (SO-ARM100 / onshape-to-robot lineage).

**Lucy schema note:** `servo_type` must be `180` / `270` / `300` (PWM-family field; SO-ARM uses `'300'` as a stand-in). Actuators are `board_class: bus_servo_only` → Rust crate `firmwares/rp2040_servo2040` (UART bus gated by YAML `HAS_BUS`). `physical_pin` is the STS3215 bus id. Optional board **`slave_address`** is the Modbus slave id on that board’s USB CDC (default `1`). `firmware.source_dir` is `lucy_embedded_firmware`; UF2 target is `lucy_so_arm`.

Gazebo: stock `gz_ros2_control/GazeboSimSystem` ignores Lucy firmware-space `offset_rad` / `servo_*_rad` params on the gazebo xacro — joint limits come from the URDF.

---

## 4. Joint-state feedback (blue dots)

Today Lucy is **open-loop** for real hardware: `LucySystemHardware::read()` echoes
the last commanded position; control-panel blue dots follow `/joint_states`.

STS3215 encoders exist but are not wired into the HI yet (disabled `type: encoder`
rows in `active.yaml`). Mock / Gazebo behave like other Lucy robots.

---

## 5. Launch files

| Launch | Purpose |
|--------|---------|
| `joint_preview.launch.py` | RSP + JSP GUI + RViz (no HW) |
| `control.launch.py` | `lucy_control_supervisor` (RSP + CM + spawners; real or mock) |
| `gazebo.launch.py` | gz-sim + spawn + controllers |
| `rviz.launch.py` / `rviz_standalone.launch.py` | RViz only |

```bash
ros2 launch so_arm101_urdf joint_preview.launch.py
ros2 launch so_arm101_urdf control.launch.py use_mock_hardware:=true
# Real hardware: see REAL_HARDWARE_SETUP.md
LUCY_ROBOT_PACKAGE=so_arm101_urdf pixi run core -- real:=true
```

---

## 6. Xacro entry

`description/urdf/robot.urdf.xacro` includes properties + body, then either
generated `so_arm101_ros2_control.xacro` or Gazebo physics + `gazebo.xacro`.

```bash
ros2 run xacro xacro description/urdf/robot.urdf.xacro \
  base_path:=$(pwd)/description \
  controller_config:=$(pwd)/config/controllers.yaml \
  use_mock_hardware:=true
```

Config regeneration commands live in [REAL_HARDWARE_SETUP.md](REAL_HARDWARE_SETUP.md) §4.

---

## 7. Build and test

```bash
colcon build --symlink-install --packages-select so_arm101_urdf lucy_ros2_control
source install/setup.bash
colcon test --packages-select so_arm101_urdf --event-handlers console_direct+
```

No `package.xml` dependency on `lucy_ros2_control` (avoid cycles). Keep both
packages built in the same workspace for control launches.

---

## 8. Differences from InMoov / Thais

| | InMoov / Thais | SO-ARM101 |
|--|----------------|-----------|
| Form factor | Full humanoid | Single 6-DOF arm |
| Entry xacro | `inmoov.urdf.xacro` | `robot.urdf.xacro` |
| Meshes | Collada DAE | STL |
| Servos | PWM hobby | STS3215 bus (schema still PWM-typed) |
| Controllers | left/right arm + torso_head | `so_arm_controller` |
| Encoders | None | Prepared (disabled) |

---

## 9. Future work

- Hardware serial IDs / bus-servo firmware
- Closed-loop encoder → `/joint_states`
- Leader → follower teleoperation bridge
- Dual-arm (leader + follower) configs
