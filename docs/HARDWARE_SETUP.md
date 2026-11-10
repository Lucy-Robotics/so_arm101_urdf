# SO-ARM101 hardware setup

Commands assume the current directory is the **`lucy_ws` root**.

Do this once per physical arm **before** Control Robot ON. Skipping calibration can damage the servos.

Package internals: [DEVELOPER.md](DEVELOPER.md).

## 1. USB

```bash
pixi run firmware-setup
ls -l /dev/ttyACM*
```

## 2. Host firmware

```bash
pixi run firmware-build-linux
```

## 3. Calibrate in the control panel

Start the stack (`robot_package:=so_arm101_urdf`, `real:=true`), then open
**Robot Configuration → Calibrate** and follow the wizard. It takes a few minutes:

1. **Health checks**: adapter, servos, torque off.
2. **Mid pose**: support the arm and hold it in the URDF's default pose shown in the 3D view
   (every joint at zero). This writes each servo's `Homing_Offset` to its EEPROM.
3. **Each joint in turn**: recording starts on its own. Move the joint to both stops while the
   bar fills, then press Enter, Space or **Accept and continue** to save it and go to the next.
4. **Review, then activate**: saves `configs/so101_<serial>.yaml` and a LeRobot-format
   `configs/so101_<serial>.lerobot.json` next to it, regenerates, and reloads.
5. The control page opens with torque off. Check that the 3D view follows the arm by hand
   before turning **Control Robot** on.

The arm is identified by its USB adapter's serial number (iSerial). Lucy remembers the
preset for each adapter, and at startup activates it or sends you back here if the arm
isn't recognised. Control Robot stays refused until then.

Cancelling or restarting mid-way is safe: the servos' previous values are kept and can be
restored. If the wizard reports an interrupted calibration, choose **Resume** or **Restore**.

### Already calibrated with LeRobot

```bash
src/so_arm101_urdf/scripts/apply_lerobot_calibration.py \
  ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json \
  --preset-name my_follower_arm
```

Then in the control panel: **VALIDATE → ACTIVATE → RELOAD**, and on the Calibrate tab,
**Already calibrated: use the active preset**. Presets applied by an older version of this
script have no `calibration:` block, so activating them restores the stock URDF limits:
run the script again to add it.

## 4. Run

Control panel (`robot_package:=so_arm101_urdf`, `real:=true`): start the stack →
**Control Robot ON** → move joints.
