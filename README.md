# SO-ARM101 URDF

ROS 2 **Jazzy** description package for the **SO-ARM101** 6-DOF follower arm
(STS3215 smart servos), integrated with Lucy bringup / ros2_control / Gazebo.

## Overview

Standalone robot package (selected via `robot_package:=so_arm101_urdf`) with:

- Calibrated kinematics from `so101_new_calib.urdf`
- ros2_control + `LucySystemHardware` (mock or real)
- Gazebo Harmonic simulation
- Hardware YAML prepared for future magnetic-encoder feedback

## Features

- 6 revolute joints: `Rotation`, `Pitch`, `Elbow`, `Wrist_Pitch`, `Wrist_Roll`, `Jaw`
- STL meshes under `description/robot_description/meshes/stl/`
- Launch files: joint preview, control, Gazebo, RViz
- Config pipeline: `config/hardware/active.yaml` → generated ros2_control / controllers / gazebo

## Quick start (mock / preview)

```bash
# From lucy_ws
colcon build --packages-select so_arm101_urdf
source install/setup.bash

ros2 launch so_arm101_urdf joint_preview.launch.py
LUCY_ROBOT_PACKAGE=so_arm101_urdf pixi run core
```

## Real hardware (required first)

**Before enabling torque on a physical SO-101**, follow:

**[docs/REAL_HARDWARE_SETUP.md](docs/REAL_HARDWARE_SETUP.md)** — LeRobot calibrate → apply limits → regenerate → `real:=true`.

## Documentation

- **Real hardware setup:** [docs/REAL_HARDWARE_SETUP.md](docs/REAL_HARDWARE_SETUP.md)
- Package developer notes: [docs/DEVELOPER.md](docs/DEVELOPER.md)
- Workspace: [lucy_ws README — How to use your own robot?](../../README.md#how-to-use-your-own-robot)

## Code of Conduct

Please read and adhere to our [Code of Conduct](CODE_OF_CONDUCT.md).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

GNU GPL v3 — see [LICENSE](LICENSE).

## Acknowledgments

- [TheRobotStudio / SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) — SO101 simulation URDF lineage
- [Sentience Robotics](https://github.com/Sentience-Robotics) — Lucy integration
- All contributors

## Contact

- Email: [contact@sentience-robotics.fr](mailto:contact@sentience-robotics.fr)
- GitHub: [Sentience Robotics](https://github.com/Sentience-Robotics)
