# Farm drone ML data storage

This is intentionally a separate Helm release from `farm-drone`.

For the complete clone/fork workflow, including image publishing, mission
submission, curation, GPU training, detector deployment, and Showcase checks,
see [`../../docs/REPRODUCIBLE-SETUP.md`](../../docs/REPRODUCIBLE-SETUP.md).

The farm release is disposable: `helm uninstall farm-drone` may remove resources
owned by that release. Perception datasets, trained models, and evaluation reports
must live in this independent namespace and PVC instead.

Install it with:

```bash
helm upgrade --install farm-drone-ml-data ./helm/drone-ml-pipeline \
  --namespace farm-drone-ml --create-namespace \
  --values ./helm/drone-ml-pipeline/values.yaml.example --wait
```

After the PVC binds, set its backing PV reclaim policy to `Retain`:

```bash
PVC=$(oc get pvc -n farm-drone-ml \
  -l app.kubernetes.io/component=perception-data \
  -o jsonpath='{.items[0].metadata.name}')
PV=$(oc get pvc "$PVC" -n farm-drone-ml -o jsonpath='{.spec.volumeName}')
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
models or modify the data volume. The image gallery defaults to
`flight-v8-predictions` when that dataset exists, otherwise it falls back to the
newest available flight dataset. It supports switching between dataset versions
and draws truth and camera-model prediction boxes over each sample image. It can
display the first 30, first 100, or all available frames so negative ground/field
images can be audited too.

After installation, get the OpenShift URL with:

```bash
oc get route farm-drone-ml-data-viewer -n farm-drone-ml
```

For local access without a Route:

```bash
oc port-forward -n farm-drone-ml \
  svc/farm-drone-ml-data-viewer 8080:8080
```

Then open `http://127.0.0.1:8080`. The viewer starts with an empty state and
will automatically show artifacts as the recorder, training Job, and perception
validator write versioned content under `datasets/`, `models/`, and
`evaluations/`.

## Dataset recorder

The recorder is an opt-in Deployment because it requires an observer image that
contains the `dataset_recorder` executable. After building and publishing that
image, enable it with an override such as:

```bash
helm upgrade --install farm-drone-ml-data ./helm/drone-ml-pipeline \
  --namespace farm-drone-ml \
  --values ./helm/drone-ml-pipeline/values.yaml.example \
  --set recorder.enabled=true \
  --set recorder.image=YOUR_OBSERVER_IMAGE \
  --wait
```

The recorder subscribes to the farm camera, camera info, odometry, and truth
topics through Zenoh. Truth is used only to create offline labels. It writes
PNG images, YOLO labels, JSON audit labels, frame metadata, and a dataset
manifest to the persistent volume. Stop it after the desired flight by setting
`recorder.enabled=false`; the dataset remains in the ML PVC.

## Raw-data curation

Run the one-shot curator after a capture with `curator.enabled=true`. It copies
all labeled frames and a deterministic sample of no-label frames into the
configured curated dataset. Raw flight directories are not modified or deleted.
The curator assigns whole episodes to provisional train/validation splits;
reserve at least one complete episode for validation and, ideally, a separate
episode for a final test.

## GPU baseline training

The chart includes an optional free PyTorch/torchvision baseline Job. It trains a
small Faster R-CNN detector from the configured curated dataset, uses
class-balanced sampling and lightweight flips, and writes a checkpoint plus
per-class validation metrics under the configured versioned model and evaluation
directories.
It uses one NVIDIA GPU and does not use `/drone/target_truth` at runtime.

Because the PVC is ReadWriteOnce, disable the viewer and storage anchor while
the trainer runs, then enable them again after the Job completes. The promoted
artifact is the PyTorch checkpoint itself and runs directly with CUDA.

## Camera detector

The detector Deployment runs the camera-only ROS node using the trained Faster
R-CNN checkpoint:

```bash
helm upgrade farm-drone-ml-data ./helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --set detector.enabled=true \
  --set detector.image=YOUR_OBSERVER_IMAGE \
  --set detector.modelPath=/data/perception/models/v1/detector.pt --wait
```

It subscribes only to `/drone/camera/image_raw` and publishes predictions,
bounding boxes, confidence, and latency to `/drone/camera_detections`; it does
not subscribe to `/drone/target_truth`. The farm observer consumes this topic
during `INSPECT`, so runtime mission detections come from the camera model.
Simulator truth remains an offline labeling and scoring source only.

## Held-out model evaluation

The recommended numeric gate evaluates the trained checkpoint against a
reserved `test` episode. It reads only `images/test` and `labels/test`, never
changes the dataset, and writes a versioned JSON report to the ML PVC. The
repository also contains a small shell entry point for local or mounted-volume
use:

```bash
EVAL_SPLIT=test \
DATASET_DIR=/data/perception/datasets/curated-v7 \
MODEL_PATH=/data/perception/models/v7/detector.pt \
REPORT_PATH=/data/perception/evaluations/v7-held-out/report.json \
./scripts/evaluate-model.sh
```

For the GPU-backed OpenShift run, first ensure the test episode was curated
with `split=test`, then disable the detector/trainer while the single GPU is
reserved for the evaluator:

```bash
helm upgrade farm-drone-ml-data ./helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --set detector.enabled=false \
  --set trainer.enabled=false \
  --set heldOutEvaluator.enabled=true \
  --wait

EVALUATOR_JOB=$(oc get job -n farm-drone-ml \
  -l app.kubernetes.io/component=held-out-evaluator \
  -o jsonpath='{.items[0].metadata.name}')
oc wait -n farm-drone-ml --for=condition=complete \
  "job/${EVALUATOR_JOB}" --timeout=60m
oc logs -n farm-drone-ml "job/${EVALUATOR_JOB}"
```

The report appears under `evaluations/v7-held-out/report.json` and the
Showcase displays it in the Evaluation section. The evaluator records the
model version, split, device, IoU/confidence thresholds, per-class precision
and recall, false positives, missed detections, and latency. It refuses to
evaluate the training split. Add `--enforce-thresholds` locally, or set
`heldOutEvaluator.enforceThresholds=true` in Helm, to fail the job when the
starting gate is not met.

The current curated datasets contain `train` and `validation` only. Do not use
`validation` as the final promotion claim: capture one complete additional
episode, curate it with `splitMap: {flight-test-v1: test}`, and point
`heldOutEvaluator.datasetDir` at that new curated dataset. Until then, the
evaluator intentionally refuses to produce a final held-out report.

Re-enable the viewer and storage anchor after the GPU Job completes:

```bash
helm upgrade farm-drone-ml-data ./helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --set heldOutEvaluator.enabled=false \
  --set viewer.enabled=true --set anchor.enabled=true \
  --wait
```

## Live perception validation

Run the validator for a bounded mission window after enabling the detector:

```bash
helm upgrade farm-drone-ml-data ./helm/drone-ml-pipeline \
  --namespace farm-drone-ml --reuse-values \
  --set perceptionValidator.enabled=true --wait
```

Submit the desired mission during that window, then disable the validator after
the Job completes. The validator uses truth only offline to calculate IoU-based
precision and recall and writes
`evaluations/perception-v7/report.json`. It never publishes to the live mission
detection topic.

When the recorder runs alongside the detector, each captured frame stores the
camera model's predictions in its label JSON. The showcase draws simulator truth
in yellow and model predictions in green with confidence percentages.

The previous ONNX exporter and ONNX-specific templates are retained under
`archived/onnx/` for historical reference and are not part of the active Helm
workflow.
