# Reproducible setup

This runbook is the clone/fork path for the OpenShift demo. It uses two
independent Helm releases so the farm can be reinstalled without deleting the
ML data:

| Release | Purpose | Default namespace |
| --- | --- | --- |
| `farm-drone` | Gazebo, ROS, observer, noVNC, mission evidence | `farm-drone` |
| `farm-drone-ml-data` | datasets, labels, models, detector, Showcase | `farm-drone-ml` |

Change the release or namespace names if needed, but keep the ML values'
`demo.namespace` and `demo.zenohService` aligned with the farm release.

## Prerequisites

You need:

- an OpenShift cluster with `oc` access and Helm 3;
- a registry where the world and observer images can be pushed;
- a Linux build host with Podman and access to the configured base images;
- an NVIDIA GPU node and device plugin for Faster R-CNN training or inference;
- a storage class that can provision a ReadWriteOnce PVC (the example uses
  `gp3-csi`).

Log in to OpenShift and the registry before installing. Do not commit local
`values.yaml`, `versions.env`, credentials, or private keys.

## 1. Build and publish images

On the build host:

```bash
cp versions.env.example versions.env
$EDITOR versions.env
source versions.env
./scripts/build-images.sh all
podman push "${DRONE_WORLD_IMAGE}"
podman push "${DRONE_OBSERVER_IMAGE}"
```

Set the same published world and observer image names in
`helm/drone-demo/values.yaml`, copied from the example. The observer image must
contain the current ROS workspace; it provides the mission observer, camera
detector, and dataset recorder. The Zenoh and noVNC images in the example values
must also be accessible from your cluster.

## 2. Install the farm and ML releases

The examples are safe templates. Copy them to ignored local files and update the
image references and ML connection values:

```bash
cp helm/drone-demo/values.yaml.example helm/drone-demo/values.yaml
cp helm/drone-ml-pipeline/values.yaml.example helm/drone-ml-pipeline/values.yaml
```

For the default release names, the ML values should contain:

```yaml
demo:
  namespace: farm-drone
  zenohService: farm-drone-ros-drone-demo-zenoh
```

Install the farm first so its Zenoh service exists:

```bash
helm lint helm/drone-demo --values helm/drone-demo/values.yaml
helm upgrade --install farm-drone helm/drone-demo \
  --namespace farm-drone --create-namespace \
  --values helm/drone-demo/values.yaml --wait

helm lint helm/drone-ml-pipeline --values helm/drone-ml-pipeline/values.yaml
helm upgrade --install farm-drone-ml-data helm/drone-ml-pipeline \
  --namespace farm-drone-ml --create-namespace \
  --values helm/drone-ml-pipeline/values.yaml --wait
```

Verify the deployments and PVCs:

```bash
oc get pods,pvc -n farm-drone
oc get pods,pvc -n farm-drone-ml
oc get route -n farm-drone-ml \
  -l app.kubernetes.io/component=perception-showcase
```

After the ML PVC binds, set its backing PV to `Retain`. This is an additional
guard against accidental data loss:

```bash
PVC=$(oc get pvc -n farm-drone-ml \
  -l app.kubernetes.io/component=perception-data \
  -o jsonpath='{.items[0].metadata.name}')
PV=$(oc get pvc "$PVC" -n farm-drone-ml -o jsonpath='{.spec.volumeName}')
oc patch pv "$PV" --type merge \
  -p '{"spec":{"persistentVolumeReclaimPolicy":"Retain"}}'
```

Get the Showcase URL at any time with:

```bash
SHOWCASE_HOST=$(oc get route -n farm-drone-ml \
  -l app.kubernetes.io/component=perception-showcase \
  -o jsonpath='{.items[0].spec.host}')
echo "http://${SHOWCASE_HOST}"
```

If Routes are unavailable, use:

```bash
oc port-forward -n farm-drone-ml svc/farm-drone-ml-data-viewer 8080:8080
```

Then open `http://127.0.0.1:8080`.

## 3. Run a mission and capture training data

The first capture uses simulator truth only to create offline labels. It does
not make the runtime detector truth-driven.

Set the recorder image to your published observer image and enable it:

```bash
source versions.env
helm upgrade farm-drone-ml-data helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --set recorder.enabled=true \
  --set recorder.image="${DRONE_OBSERVER_IMAGE}" \
  --set recorder.scenario=all \
  --set recorder.seed=flight-v1 \
  --set recorder.outputDir=/data/perception/datasets/flight-v1 \
  --wait
```

Submit the mission from the farm observer:

```bash
DEMO_POD=$(oc get pod -n farm-drone \
  -l app.kubernetes.io/name=ros-drone-demo-observer \
  -o jsonpath='{.items[0].metadata.name}')
oc exec -n farm-drone "pod/${DEMO_POD}" -- bash -lc \
  'source /opt/drone/scripts/ros-env.sh &&
   source /opt/drone_ws/install/setup.bash &&
   ros2 action send_goal /drone/survey drone_observer_msgs/action/SurveyMission
   "{mission_id: flight-v1, scenario: all}" --feedback'
```

Stop the recorder after the mission completes so the dataset is stable:

```bash
helm upgrade farm-drone-ml-data helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --set recorder.enabled=false --wait
```

Refresh the Showcase. It should list `flight-v1`, its manifest, images, and
truth labels. Capture at least one additional complete episode before training;
reserve a whole episode for validation rather than mixing frames from the same
episode across training and validation.

## 4. Curate and train the model

Edit the ignored ML values file to select the captured episodes. For example:

```yaml
curator:
  enabled: true
  sources: [flight-v1, flight-v2]
  outputDir: /data/perception/datasets/curated-v1
  splitMap:
    flight-v1: train
    flight-v2: validation

trainer:
  datasetDir: /data/perception/datasets/curated-v1
  modelDir: /data/perception/models/v1
  evalDir: /data/perception/evaluations/v1
```

Run the curator while the viewer and storage anchor are enabled:

```bash
helm upgrade farm-drone-ml-data helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --values helm/drone-ml-pipeline/values.yaml \
  --set curator.enabled=true --wait
CURATOR_JOB=$(oc get job -n farm-drone-ml \
  -l app.kubernetes.io/component=dataset-curator \
  -o jsonpath='{.items[0].metadata.name}')
oc logs -n farm-drone-ml "job/${CURATOR_JOB}"
```

The PVC is ReadWriteOnce. Before starting the GPU trainer, release it from the
viewer and storage anchor:

```bash
helm upgrade farm-drone-ml-data helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --values helm/drone-ml-pipeline/values.yaml \
  --set viewer.enabled=false --set anchor.enabled=false \
  --set curator.enabled=false --set trainer.enabled=true --wait
TRAINER_JOB=$(oc get job -n farm-drone-ml \
  -l app.kubernetes.io/component=model-trainer \
  -o jsonpath='{.items[0].metadata.name}')
oc wait -n farm-drone-ml --for=condition=complete \
  "job/${TRAINER_JOB}" --timeout=60m
oc logs -n farm-drone-ml "job/${TRAINER_JOB}"
```

The trainer writes a PyTorch checkpoint, model manifest, and evaluation report
under the configured versioned directories. Re-enable the viewer and anchor
after training:

```bash
helm upgrade farm-drone-ml-data helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --values helm/drone-ml-pipeline/values.yaml \
  --set trainer.enabled=false --set viewer.enabled=true --set anchor.enabled=true \
  --wait
```

## 5. Deploy the camera detector and validate in the Showcase

Point the detector at the trained checkpoint and use the same observer image that
contains the detector executable:

```bash
source versions.env
helm upgrade farm-drone-ml-data helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --values helm/drone-ml-pipeline/values.yaml \
  --set detector.enabled=true \
  --set detector.image="${DRONE_OBSERVER_IMAGE}" \
  --set detector.modelPath=/data/perception/models/v1/detector.pt \
  --set detector.modelVersion=v1 --wait
```

For prediction-aware capture, enable the recorder alongside the detector:

```bash
helm upgrade farm-drone-ml-data helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --set recorder.enabled=true \
  --set recorder.image="${DRONE_OBSERVER_IMAGE}" \
  --set recorder.seed=prediction-showcase-v1 \
  --set recorder.outputDir=/data/perception/datasets/prediction-showcase-v1 \
  --wait
```

Run the same mission again, then disable the recorder. The Showcase should now
show both kinds of boxes for frames where the model produced a prediction:

- yellow = simulator ground truth;
- green = camera-model prediction;
- the prediction caption = class, confidence, model version, and count.

This visual comparison is the primary validation/demo output. The optional
perception validator can additionally calculate IoU-based precision and recall,
but it is not required to inspect or demonstrate the captured predictions.

## Safe reinstall and data preservation

Both PVC templates use `helm.sh/resource-policy: keep`. To recreate workloads
without deleting the ML data:

```bash
helm uninstall farm-drone --namespace farm-drone
helm uninstall farm-drone-ml-data --namespace farm-drone-ml
```

Run the install commands again. Verify the ML PVC, datasets, model checkpoint,
and Showcase are still present before running new jobs. Never delete the ML PVC
or its retained PV unless deleting all perception data is intentional.

## What is and is not versioned in Git

The repository versions the charts, ROS code, recorder, label projection,
training script, and reproducible configuration. Captured images, labels, and
trained `.pt` weights live on the ML PVC and are intentionally not committed to
GitHub. A clone or fork therefore runs its own missions and training job to
produce its own model.
