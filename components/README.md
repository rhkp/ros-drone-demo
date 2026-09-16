# Components

Component image definitions are grouped by the subsystem they package:

- `simulation-world/` — Gazebo farm world, route visualization, and world image.
- `drone-observer/` — ROS observer image and its runtime environment.

The shared ROS workspace, scenario configuration, and operator scripts remain at
the repository root because both components consume them.
