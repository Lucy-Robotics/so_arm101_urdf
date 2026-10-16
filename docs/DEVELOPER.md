# Developer guide - `so_arm101_urdf`

ROS 2 **Jazzy** description package for the **SO-ARM101** 6-DOF follower arm.
Bringup, hardware plugin, and cameras live in **`lucy_ros_packages`**.

> **CRITICAL — real SO-101 hardware:** Do **not** enable torque or command a
> physical arm with Lucy until LeRobot calibration for **that** arm has been
> applied with `scripts/apply_lerobot_calibration.py` and configs regenerated
> (§7–§9). Stock `configs/default.yaml` and URDF limits are a generic template,
> not a safe travel envelope for an uncalibrated unit. Skipping calibration can
> drive STS3215 servos past mechanical stops. Mock / Gazebo do not replace this
> step before first contact with real hardware.

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
│       └── configs/default.yaml   # stock template (not arm-specific)
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
| `control.launch.py` | `lucy_control_supervisor` (RSP + CM + spawners; real or mock) |
| `gazebo.launch.py` | gz-sim + spawn + controllers |
| `rviz.launch.py` / `rviz_standalone.launch.py` | RViz only |

```bash
ros2 launch so_arm101_urdf joint_preview.launch.py
ros2 launch so_arm101_urdf control.launch.py use_mock_hardware:=true
# Real Feetech USB path — `real:=true` starts lucy-embedded-firmware-linux:
LUCY_ROBOT_PACKAGE=so_arm101_urdf pixi run core -- real:=true
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

## 7. Desk bringup (plug → calibrate → Lucy)

**Required before any real SO-101 with Lucy.** Sections 7.2–7.4 are not
optional polish: without a per-arm LeRobot calibration applied into this package
(and regenerated xacros), joint limits and zero are wrong for that mechanism.
Do not turn **Control Robot ON** until they are done.

End-to-end path for a physical SO-101 on the Feetech USB adapter (`1a86:55d3`,
typically `/dev/ttyACM0`).

### 7.1 USB access

From `lucy_ws`:

```bash
pixi run firmware-setup   # udev rules + temporary chmod for Pico/Feetech
```

Confirm the adapter is visible and writable (`ls -l /dev/ttyACM*`).

### 7.2 LeRobot calibration

Pose the arm in the **URDF default pose** (Lucy joint 0). LeRobot half-turn
homing records that pose as tick 2048. Then sweep each joint through its full
mechanical range.

```bash
cd ~/Projects/sentience/lerobot
# optional: lerobot-find-port
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_follower_arm
```

JSON lands under
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json`.
Leave the motors' `Homing_Offset` as LeRobot wrote them — Lucy goals use that
Present-Position frame with `offset = π`.

### 7.3 Apply calibration into this package

```bash
cd ~/Projects/sentience/lucy_ws/src/so_arm101_urdf
scripts/apply_lerobot_calibration.py \
  ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json
# optional: --dry-run   or   --no-urdf
```

Writes:

| Target | Fields |
|--------|--------|
| `config/hardware/active.yaml` (+ preset) | `offset`/`default` = π; `limit_min`/`limit_max` = travel in **servo** rad |
| `description/robot_description/urdf/robot_description.urdf.xacro` | `lower`/`upper` = same window in **joint** rad |

Mapping used everywhere (HI, LCP, apply script):

```text
joint = (servo - offset) * direction * scale
servo = joint / (direction * scale) + offset
```

With `offset = default = π`, the calibrated centre is URDF joint 0, so the real
arm and the 3D view share the same zero.

### 7.4 Regenerate ros2_control / Gazebo artifacts

See §9. Required after every apply — YAML alone does not update the xacros.

### 7.5 Build host firmware (once / after firmware changes)

```bash
cd ~/Projects/sentience/lucy_ws
pixi run firmware-build-linux
# binary: src/lucy_embedded_firmware/target/release/lucy-embedded-firmware-linux
```

### 7.6 Launch

```bash
cd ~/Projects/sentience/lucy_ws
LUCY_ROBOT_PACKAGE=so_arm101_urdf pixi run core -- real:=true
```

`real` defaults to **false** in `lucy.launch.py`; without `real:=true` the
Feetech SHM consumer never starts and the arm will not move.

Open the control panel, turn **Control Robot ON** (enables torque via
`/lucy/active_client`), then move joints. Motor daisy-chain LEDs must be lit.

---

## 8. Applying a LeRobot calibration

`lerobot-calibrate` records each servo's travel in STS3215 ticks (Present frame
after half-turn homing). Lucy stores the same windows in **radians**
(`rad = ticks * 2π / 4096`).

```bash
cd src/so_arm101_urdf
scripts/apply_lerobot_calibration.py \
  ~/.cache/huggingface/lerobot/calibration/robots/so_follower/<arm>.json
```

It patches `config/hardware/active.yaml` and the preset named in
`active_meta.yaml`, plus URDF joint limits (unless `--no-urdf`). `--dry-run`
reports without writing; `--config` targets a different YAML.

Keep stock `configs/default.yaml` as the generic template. After apply, copy the
preset to a personal file under `configs/` (gitignored / not pushed) and
activate that for desk work — do not commit one-arm limits into `default.yaml`.

Records are paired with actuators by servo bus id against `physical_pin`, not by
name — LeRobot's joint names have no relation to `urdf_joint`. A record matching
no actuator aborts before anything is written.

`offset` / `default` are set to π (2048 ticks), the homed centre (= URDF joint
0), not the midpoint of the window — those differ on asymmetric joints (gripper).
`direction` is left alone: check it against the 3D view on hardware. A non-zero
`drive_mode` is reported so it is not silently dropped.

Regenerating afterwards (§9) is required: the script does not rewrite
`so_arm101_ros2_control.xacro` / `gazebo.xacro`.

---

## 9. Regenerating configs

From `lucy_ws` (pixi env; `generate_config` from `lucy_config_generator` source
or a built install):

```bash
cd ~/Projects/sentience/lucy_ws
OUT=/tmp/so_arm101_gen
rm -rf "$OUT" && mkdir -p "$OUT"
pixi run bash -lc '
  export PYTHONPATH=src/lucy_ros_packages/lucy_config_generator:${PYTHONPATH:-}
  python -m lucy_config_generator.process \
    --input src/so_arm101_urdf/config/hardware/active.yaml \
    --urdf src/so_arm101_urdf/description/urdf/robot.urdf.xacro \
    --base-path src/so_arm101_urdf/description \
    --controller-config src/so_arm101_urdf/config/controllers.yaml \
    --output-dir /tmp/so_arm101_gen \
    --targets all
'
cp "$OUT"/so_arm101_ros2_control.xacro \
  src/so_arm101_urdf/description/ros2_control/
cp "$OUT"/controllers.yaml src/so_arm101_urdf/config/
cp "$OUT"/gazebo.xacro "$OUT"/gazebo_bridge.yaml \
  src/so_arm101_urdf/description/gazebo/
```

Or use the control-panel **VALIDATE → ACTIVATE → RELOAD** pipeline with
`robot_package:=so_arm101_urdf`.

---

## 10. Build and test

```bash
colcon build --symlink-install --packages-select so_arm101_urdf lucy_ros2_control
source install/setup.bash
colcon test --packages-select so_arm101_urdf --event-handlers console_direct+
```

No `package.xml` dependency on `lucy_ros2_control` (avoid cycles). Keep both
packages built in the same workspace for control launches.

---

## 11. Differences from InMoov / Thais

| | InMoov / Thais | SO-ARM101 |
|--|----------------|-----------|
| Form factor | Full humanoid | Single 6-DOF arm |
| Entry xacro | `inmoov.urdf.xacro` | `robot.urdf.xacro` |
| Meshes | Collada DAE | STL |
| Servos | PWM hobby | STS3215 bus (schema still PWM-typed) |
| Controllers | left/right arm + torso_head | `so_arm_controller` |
| Encoders | None | Prepared (disabled) |

---

## 12. Future work

- Hardware serial IDs / bus-servo firmware
- Closed-loop encoder → `/joint_states`
- Leader → follower teleoperation bridge
- Dual-arm (leader + follower) configs
