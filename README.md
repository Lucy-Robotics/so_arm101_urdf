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

## Documentation

- Hardware setup (calibrate before Control Robot ON): [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md)
- Package developer notes: [docs/DEVELOPER.md](docs/DEVELOPER.md)
- Workspace quickstart and robot selection: [lucy_ws README](../../README.md)

## Code of Conduct

Please read and adhere to our [Code of Conduct](CODE_OF_CONDUCT.md).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

GNU GPL v3. See [LICENSE](LICENSE).

## Acknowledgments

- [TheRobotStudio / SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100): SO101 simulation URDF lineage

## Contact

- Email: [contact@sentience-robotics.fr](mailto:contact@sentience-robotics.fr)
- GitHub: [Sentience Robotics](https://github.com/Sentience-Robotics)
