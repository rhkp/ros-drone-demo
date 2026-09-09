# ROS Farm Survey Drone Demo

A ROS 2 and Gazebo farm simulation in which one observer drone surveys farm assets, captures rendered evidence images, and publishes structured detections.

The demo scenario contains:

- a well and elevated water tank;
- an item-storage silo;
- a cattle shed;
- a produce barn;
- a crop field and greenhouse;
- farmhouse, tractor, fencing, and crop-row scenery.

The five core detection types are `well`, `storage_silo`, `cattle_shed`, `produce_barn`, and `crop_field`. Scenarios are configured in [`config/scenarios.yaml`](config/scenarios.yaml): `inventory`, `crops`, `water`, and `all`.

## Repository layout

- `worlds/` — farm world definition.
- `ros_ws/src/drone_observer_msgs/` — reusable truth, detection, and survey action interfaces.
- `ros_ws/src/drone_observer/` — waypoint controller, rendered-camera subscriber, truth publisher, and observer action server.
- `scripts/launch-world.sh` — Gazebo startup plus ROS-Gazebo image and pose bridges.
- `images/` — separate world and observer image definitions.
- `helm/` — OpenShift/Kubernetes deployment with Zenoh connectivity and evidence PVC.

## Local run

Use a ROS 2 Jazzy environment with Gazebo Sim and build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ros_ws/src --ignore-src -r -y
colcon build --base-paths ros_ws/src
source install/setup.bash
./scripts/run-local.sh
```

In another shell, submit a mission:

```bash
source install/setup.bash
./scripts/submit-mission.sh all
```

Evidence and `report.json` are written below `artifacts/` by default.

The farm world contains the moving `observer_drone` model and a downward-facing Gazebo camera sensor. The world image bridges rendered camera frames to `/drone/camera/image_raw` and exposes `/world/farm_survey/set_pose`; the observer uses that service to keep the visible drone model synchronized with each waypoint. Evidence capture fails if no rendered frame is available.

## Images and OpenShift

The world image contains only the farm world and Gazebo runtime additions. The observer image contains the ROS package and mission logic. Build and publish them using the project’s validated build VM workflow, for example with the image names in [`versions.env.example`](versions.env.example). The images are intentionally separate so the farm scene and observer behavior can evolve independently.

```bash
helm upgrade --install ros-drone-demo ./helm \
  -f ./helm/values.yaml \
  --namespace ros-drone-demo --create-namespace --wait
```

Copy `helm/values.yaml.example` to a local, ignored `helm/values.yaml` before deployment. Never commit that copy, `.env`, cloud credentials, registry credentials, private keys, or other sensitive files. The committed example files contain only public placeholder configuration.
