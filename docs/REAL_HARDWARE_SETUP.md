# SO-ARM101 — real hardware setup

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

Do not commit one-arm limits into `configs/default.yaml` — copy the preset to a personal file under `src/so_arm101_urdf/config/hardware/configs/` if you need to keep it.

## 4. Regenerate

Control panel: **VALIDATE → ACTIVATE → RELOAD** with `robot_package:=so_arm101_urdf`,

## 5. Host firmware

```bash
pixi run firmware-build-linux
```

## 6. Run

```bash
LUCY_ROBOT_PACKAGE=so_arm101_urdf pixi run core -- real:=true
```

Control panel → **Control Robot ON** → move joints.
