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

The survey action runs an explicit mission state machine: `TAKEOFF`, `TRANSIT`,
`INSPECT`, `RETURN`, `LAND`, and `COMPLETE`. Cancellation or a movement/camera
failure enters `EMERGENCY` and attempts a return-to-home and landing sequence.
The current state is also published on `/drone/mission_state`.

Each accepted mission publishes its complete planned 3-D route on
`/drone/mission_path` as a `nav_msgs/Path`, including takeoff, survey
waypoints, return, and landing. The same route is recorded in the mission's
`report.json` under `planned_route` for downstream navigation and audit tools.
The navigation layer validates every route against the configured geofence and
publishes normalized execution progress on `/drone/navigation_progress`. It can
also route around configurable axis-aligned `no_fly_zones` without changing
the farm world or its target coordinates.
Runtime obstacle centers may be supplied as a `geometry_msgs/PoseArray` on
`/drone/dynamic_obstacles`. Each pose is expanded using the configured
`dynamic_obstacles` dimensions, stale feeds expire automatically, and an active
mission safely replans around newly blocked segments. Replan events are recorded
in the mission report for verification.

The drone starts and lands at the dedicated `DRONE STATION` deck at
`farm_map` coordinates `(10, 16)`. The observer and kinematic flight node use
the same home coordinates, so every normal or emergency return ends at that
deck rather than inside the crop field.

The visible Gazebo model follows the interpolated flight pose continuously,
with configurable acceleration (`DRONE_ACCELERATION_MPS2`) and cruise speed,
so waypoint transitions are rendered as smooth flight rather than teleports.

## Images and OpenShift

The world image contains only the farm world and Gazebo runtime additions. The observer image contains the ROS package and mission logic. Build and publish them using the project’s validated build VM workflow, for example with the image names in [`versions.env.example`](versions.env.example). The images are intentionally separate so the farm scene and observer behavior can evolve independently.

```bash
helm upgrade --install ros-drone-demo ./helm \
  -f ./helm/values.yaml \
  --namespace ros-drone-demo --create-namespace --wait
```

The example enables the private noVNC viewer. Forward it locally instead of exposing
the VNC service publicly:

```bash
oc port-forward -n ros-drone-demo svc/ros-drone-demo-ros-drone-demo-novnc 6080:8080
```

Open `http://127.0.0.1:6080/vnc.html` to view Gazebo. Survey evidence is stored on
the observer PVC and can be downloaded after a mission with:

```bash
POD=$(oc get pod -n ros-drone-demo \
  -l app.kubernetes.io/name=ros-drone-demo-observer \
  -o jsonpath='{.items[0].metadata.name}')
oc cp -n ros-drone-demo "$POD:/artifacts/<mission-id>" ./artifacts/<mission-id>
```

Copy `helm/values.yaml.example` to a local, ignored `helm/values.yaml` before deployment. Never commit that copy, `.env`, cloud credentials, registry credentials, private keys, or other sensitive files. The committed example files contain only public placeholder configuration.
