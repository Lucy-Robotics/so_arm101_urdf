# Developer guide - `so_arm101_urdf`

ROS 2 **Jazzy** description package for the **SO-ARM101** 6-DOF follower arm.
Bringup, hardware plugin, and cameras live in **`lucy_ros_packages`**.

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
├── docs/DEVELOPER.md
├── launch/                 # control, gazebo, joint_preview, rviz, rviz_standalone
├── config/
│   ├── control.launch.yaml
│   ├── controllers.yaml    # generated
│   ├── so_arm101_rviz.rviz
│   └── hardware/
│       ├── active.yaml
│       ├── active_meta.yaml
│       └── configs/default.yaml
├── description/
│   ├── urdf/robot.urdf.xacro
│   ├── robot_description/
│   │   ├── urdf/properties.xacro
│   │   ├── urdf/robot_description.urdf.xacro
│   │   └── meshes/stl/     # 13 STL meshes
│   ├── ros2_control/so_arm101_ros2_control.xacro   # generated
│   └── gazebo/
│       ├── so_arm101_gazebo_physics.xacro
│       ├── gazebo.xacro                            # generated
│       └── gazebo_bridge.yaml                      # generated
├── worlds/default.sdf
├── scripts/apply_lerobot_calibration.py
└── test/                   # test_xacro_smoke.py, test_apply_lerobot_calibration.py
```

---

## 3. Joints and hardware

| Joint | Approx. range (rad) | Servo |
|-------|---------------------|-------|
| `Rotation` | ±1.92 | STS3215 |
| `Pitch` | ±1.75 | STS3215 |
| `Elbow` | −1.81 … 1.59 | STS3215 |
| `Wrist_Pitch` | ±1.66 | STS3215 |
| `Wrist_Roll` | ±2.79 | STS3215 |
| `Jaw` | −0.17 … 1.75 | STS3215 |

Kinematics come from the calibrated `so101_new_calib.urdf` (SO-ARM100 / onshape-to-robot lineage).

**Lucy schema note:** `servo_type` must be `180` / `270` / `300` (PWM-family field; SO-ARM uses `'300'` as a stand-in). Actuators are `board_class: bus_servo_only` → Rust crate `firmwares/rp2040_servo2040` (UART bus gated by YAML `HAS_BUS`). `physical_pin` is the STS3215 bus id. Optional board **`slave_address`** is the Modbus slave id on that board’s USB CDC (default `1`). `firmware.source_dir` is `lucy_embedded_firmware`; UF2 target is `lucy_so_arm`.

Gazebo: stock `gz_ros2_control/GazeboSimSystem` ignores Lucy firmware-space `offset_rad` / `servo_*_rad` params on the gazebo xacro — joint limits come from the URDF. Those params are emitted for consistency with the HI ros2_control xacro, not because Gazebo applies them today.

---

## 4. Joint-state feedback (blue dots)

Today Lucy is **open-loop** for real hardware:

- `LucySystemHardware::read()` echoes the last commanded position
- Control-panel blue dots subscribe to `/joint_states` (command echo, not encoders)

STS3215 servos **do** have magnetic encoders (`Present_Position`, 4096 ticks/rev). This package prepares disabled `type: encoder` sensor rows in `active.yaml`. Enabling closed-loop feedback later requires:

1. Bus-servo driver (half-duplex serial)
2. Firmware publishing measured positions on `sensors/so_arm`
3. `LucySystemHardware::read()` consuming measured joints
4. Schema / firmware templates that understand encoder sensors

Until then, mock / Gazebo behave like other Lucy robots: mock echoes commands; Gazebo physics can lag.

---

## 5. Launch files

| Launch | Purpose |
|--------|---------|
| `joint_preview.launch.py` | RSP + JSP GUI + RViz (no HW) |
| `control.launch.py` | `lucy_control_supervisor` (real or `use_mock_hardware:=true`) |
| `gazebo.launch.py` | gz-sim + spawn + controllers |
| `rviz.launch.py` / `rviz_standalone.launch.py` | RViz only |

```bash
ros2 launch so_arm101_urdf joint_preview.launch.py
ros2 launch so_arm101_urdf control.launch.py use_mock_hardware:=true
LUCY_ROBOT_PACKAGE=so_arm101_urdf pixi run core
```

---

## 6. Xacro entry

`description/urdf/robot.urdf.xacro`:

- Includes properties + body
- Unless `use_gazebo_sim`: includes generated `so_arm101_ros2_control.xacro`
- If `use_gazebo_sim`: includes physics + generated `gazebo.xacro`

Standalone expand:

```bash
ros2 run xacro xacro description/urdf/robot.urdf.xacro \
  base_path:=$(pwd)/description \
  controller_config:=$(pwd)/config/controllers.yaml \
  use_mock_hardware:=true
```

---

## 7. Applying a LeRobot calibration

`lerobot-calibrate` records each servo's travel in raw STS3215 ticks. The
hardware YAML stores the same windows in **radians**, on the scale the firmware
maps onto ticks (`BusServoConfig`: 0-2π rad over 0-4096 ticks, so
`rad = ticks * 2π / 4096`).

```bash
cd src/so_arm101_urdf
scripts/apply_lerobot_calibration.py ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<arm>.json
```

It patches `config/hardware/active.yaml` and the preset named in
`active_meta.yaml`, so re-activating that preset does not undo the calibration.
`--dry-run` reports without writing; `--config` targets a different file.

Records are paired with actuators by servo bus id against `physical_pin`, not by
name - LeRobot's joint names have no relation to `urdf_joint`. A record matching
no actuator aborts the run before anything is written.

`offset_rad` is set to π (2048 ticks), the homed centre the calibration
establishes, rather than the midpoint of the window - those differ on any joint
whose travel is not symmetric, the gripper especially. `direction` is left alone:
it is checked against the 3D view on hardware, and `drive_mode` describes an
inversion relative to LeRobot's URDF, not ours. A non-zero `drive_mode` is
reported so it is not silently dropped.

Regenerating afterwards (below) is required, not optional: the script writes
only the hardware YAML, and both `so_arm101_ros2_control.xacro` and
`gazebo.xacro` carry the same windows. Skipping it leaves the stack, and Gazebo
especially, on the previous calibration.

---

## 8. Regenerating configs

From `lucy_ws` (with workspace sourced / pixi shell):

```bash
# Generate into a temp dir, then install into package paths
OUT=/tmp/so_arm101_gen
mkdir -p "$OUT"
generate_config \
  --input src/so_arm101_urdf/config/hardware/active.yaml \
  --urdf src/so_arm101_urdf/description/urdf/robot.urdf.xacro \
  --base-path src/so_arm101_urdf/description \
  --controller-config src/so_arm101_urdf/config/controllers.yaml \
  --output-dir "$OUT" \
  --targets all

cp "$OUT"/so_arm101_ros2_control.xacro \
  src/so_arm101_urdf/description/ros2_control/
cp "$OUT"/controllers.yaml src/so_arm101_urdf/config/
cp "$OUT"/gazebo.xacro "$OUT"/gazebo_bridge.yaml \
  src/so_arm101_urdf/description/gazebo/

# Register layout for the firmware. Needs a reflash to take effect.
cp "$OUT"/config_rp2040_so_arm.rs \
  src/lucy_embedded_firmware/firmwares/rp2040/src/generated_config.rs
```

`config_<board>.rs` carries `BUS_SERVO_SLOTS`, the number of register blocks the firmware scans.

Or use the control-panel **VALIDATE → ACTIVATE → RELOAD** pipeline with `robot_package:=so_arm101_urdf`.

---

## 9. Build and test

```bash
colcon build --symlink-install --packages-select so_arm101_urdf
source install/setup.bash
colcon test --packages-select so_arm101_urdf --event-handlers console_direct+
```

No `package.xml` dependency on `lucy_ros2_control` (avoid cycles). Keep both packages built in the same workspace for control launches.

---

## 10. Differences from InMoov / Thais

| | InMoov / Thais | SO-ARM101 |
|--|----------------|-----------|
| Form factor | Full humanoid | Single 6-DOF arm |
| Entry xacro | `inmoov.urdf.xacro` | `robot.urdf.xacro` |
| Meshes | Collada DAE | STL |
| Servos | PWM hobby | STS3215 bus (schema still PWM-typed) |
| Controllers | left/right arm + torso_head | `so_arm_controller` |
| Encoders | None | Prepared (disabled) |

---

## 11. Future work

- Hardware serial IDs / bus-servo firmware
- Closed-loop encoder → `/joint_states`
- Leader → follower teleoperation bridge
- Dual-arm (leader + follower) configs
