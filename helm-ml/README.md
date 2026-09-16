# Farm drone ML data storage

This is intentionally a separate Helm release from `farm-drone`.

The farm release is disposable: `helm uninstall farm-drone` may remove resources
owned by that release. Perception datasets, trained models, and evaluation reports
must live in this independent namespace and PVC instead.

Install it with:

```bash
helm upgrade --install farm-drone-ml-data ./helm-ml \
  --namespace arhkp1-farm-drone-ml --create-namespace \
  --values ./helm-ml/values.yaml.example --wait
```

After the PVC binds, set its backing PV reclaim policy to `Retain`:

```bash
PVC=$(oc get pvc -n arhkp1-farm-drone-ml \
  -l app.kubernetes.io/component=perception-data \
  -o jsonpath='{.items[0].metadata.name}')
PV=$(oc get pvc "$PVC" -n arhkp1-farm-drone-ml -o jsonpath='{.spec.volumeName}')
oc patch pv "$PV" --type merge \
  -p '{"spec":{"persistentVolumeReclaimPolicy":"Retain"}}'
```

The dataset recorder and GPU training Jobs should run in this namespace and mount
the PVC. They can subscribe to the farm ROS graph through the farm Zenoh service;
they do not need to mount the farm deployment's evidence PVC.

The small storage-anchor deployment exists only to trigger provisioning for storage
classes such as `gp3-csi` that use `WaitForFirstConsumer`. It can later be removed
once the recorder or training Job permanently mounts the PVC.

## ML showcase viewer

The example values enable a small read-only viewer in the same project. It mounts
the perception PVC read-only and provides a human-friendly view of dataset
versions, sample images, model files, and evaluation JSON. It does not train
models or modify the data volume. The image gallery defaults to the newest
`flight-*` dataset, supports switching between dataset versions, and draws the
offline-generated bounding boxes over each sample image. It can display the
first 30, first 100, or all available frames so negative ground/field images can
be audited too.

After installation, get the OpenShift URL with:

```bash
oc get route farm-drone-ml-data-viewer -n arhkp1-farm-drone-ml
```

For local access without a Route:

```bash
oc port-forward -n arhkp1-farm-drone-ml \
  svc/farm-drone-ml-data-viewer 8080:8080
```

Then open `http://127.0.0.1:8080`. The viewer starts with an empty state and
will automatically show artifacts as the recorder, training Job, and perception
validator
write versioned content under `datasets/`, `models/`, and `evaluations/`.

## Dataset recorder

The recorder is an opt-in Deployment because it requires an observer image that
contains the `dataset_recorder` executable. After building and publishing that
image, enable it with an override such as:

```bash
helm upgrade --install farm-drone-ml-data ./helm-ml \
  --namespace arhkp1-farm-drone-ml \
  --values ./helm-ml/values.yaml.example \
  --set recorder.enabled=true \
  --set recorder.image=quay.io/rhkp/hbr-drone-observer:v0.6.9 \
  --wait
```

The recorder subscribes to the farm camera, camera info, odometry, and truth
topics through Zenoh. Truth is used only to create offline labels. It writes
PNG images, YOLO labels, JSON audit labels, frame metadata, and a dataset
manifest to the persistent volume. Stop it after the desired flight by setting
`recorder.enabled=false`; the dataset remains in the ML PVC.

## Raw-data curation

Run the one-shot curator after a capture with `curator.enabled=true`. It copies
all labeled frames and a deterministic 35% sample of no-label frames into
`datasets/curated-v6/`. Raw flight directories are not modified or deleted.
The curator assigns whole episodes to provisional train/validation splits. In the
current example, `flight-v3`, `flight-v4`, `flight-crops-v1`, `flight-water-v1`,
and `flight-v6`
are training data, while the complete `flight-v5` episode is held out for
validation. A final test episode is still required before promotion.

## GPU baseline training

The chart includes an optional free PyTorch/torchvision baseline Job. It trains a
small Faster R-CNN detector from `curated-v6`, uses class-balanced sampling and
lightweight flips, and writes a checkpoint under `models/v6/` with per-class
validation metrics under `evaluations/v6/`.
It uses one NVIDIA GPU and does not use `/drone/target_truth` at runtime.

Because the PVC is ReadWriteOnce, disable the viewer and storage anchor while
the trainer runs, then enable them again after the Job completes. The promoted
artifact is the PyTorch checkpoint itself and runs directly with CUDA.

## Camera detector

The detector Deployment runs the camera-only ROS node using the trained Faster
R-CNN checkpoint:

```bash
helm upgrade farm-drone-ml-data ./helm-ml \
  --namespace arhkp1-farm-drone-ml --reuse-values \
  --set detector.enabled=true --wait
```

It subscribes only to `/drone/camera/image_raw` and publishes predictions,
bounding boxes, confidence, and latency to `/drone/camera_detections`; it does
not subscribe to `/drone/target_truth` and does not replace the existing
mission detections.

## Perception validation

Run the validator for a bounded mission window after enabling the detector:

```bash
helm upgrade farm-drone-ml-data ./helm-ml \
  --namespace arhkp1-farm-drone-ml --reuse-values \
  --set perceptionValidator.enabled=true --wait
```

Submit the desired mission during that window, then disable the validator after
the Job completes. The validator uses truth only offline to calculate IoU-based
precision and recall and writes
`evaluations/perception-v7/report.json`. It never publishes to the live mission
detection topic.

The previous ONNX exporter and ONNX-specific templates are retained under
`archived/onnx/` for historical reference and are not part of the active Helm
workflow.
