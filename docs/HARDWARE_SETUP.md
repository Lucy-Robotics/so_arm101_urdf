# SO-ARM101 hardware setup

Commands assume the current directory is the **`lucy_ws` root**.

Do this once per physical arm **before** Control Robot ON. Skipping calibration can damage the servos.

Package internals: [DEVELOPER.md](DEVELOPER.md).

## 1. USB

```bash
pixi run firmware-setup
ls -l /dev/ttyACM*
```

## 2. Calibrate (LeRobot)

Hold the arm in the URDF default pose (Lucy joint 0), then from your LeRobot checkout:

```bash
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_follower_arm
```

## 3. Apply into Lucy

```bash
src/so_arm101_urdf/scripts/apply_lerobot_calibration.py \
  ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json
```

Writes `src/so_arm101_urdf/config/hardware/configs/my_custom_so101_config.yaml`
(and activates it). Stock `default.yaml` is not modified. Optional:
`--preset-name other_name`.

## 4. Regenerate

Control panel: **VALIDATE → ACTIVATE → RELOAD** with `robot_package:=so_arm101_urdf`.

## 5. Host firmware

```bash
pixi run firmware-build-linux
```

## 6. Run

Control panel (`robot_package:=so_arm101_urdf`, `real:=true`): start the stack →
**Control Robot ON** → move joints.
