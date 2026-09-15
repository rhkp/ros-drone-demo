# Next Feature: Drone Perception Flywheel

## Decision

The next major feature is a camera-based ML perception loop for the survey drone.
The drone should learn to detect farm assets from camera images instead of using
`/drone/target_truth` during live missions.

## Current baseline

On 2026-09-14, the fresh OpenShift deployment completed the `all` mission as
`recording-20260914`: 7 detections, full route execution, return, landing, and
`COMPLETE`. This is the baseline demo run to preserve before changing perception
behavior.

The corrected training loop is now operational as of 2026-09-15. Observer image
`v0.6.9` captured `flight-v6` (209 images, 209 audit labels) on the targeted
`inventory` route. `curated-v6` combines six raw episodes into 1,030 curated
frames, with `flight-v5` held out for validation. The free PyTorch/torchvision
baseline produced model v6 on CUDA with 78.1% macro precision, 86.3% macro
recall, and 12.8 ms inference latency. It is not deployed into the live mission
yet; the next gate is ONNX export plus a camera-only inference node and a repeat
mission comparison.

The ONNX exporter, isolated shadow detector, and offline IoU evaluator are now
implemented and deployed as `v0.8.1` in the separate ML namespace. The shadow
detector uses `/drone/camera/image_raw` only and publishes boxes, confidence,
model version, and latency to `/drone/camera_detections`; it must not be
confused with the existing truth-driven `/drone/detections` topic until the
promotion gates pass. The evaluator may read `/drone/target_truth` only to
score predictions and writes a separate shadow evaluation report.

The first valid end-to-end shadow mission completed on 2026-09-15. It scored
467 mission-active frames and 868 predictions at 57.6% macro precision and
58.5% macro recall, with 168.5 ms average runtime latency. This is below the
80%/80% demonstration promotion gate, so v6 remains an evaluated candidate,
not the live mission detector. The report is persisted at
`evaluations/shadow-v6/report.json` on the retained ML PVC.

Next implementation step: improve the v6 detector against this fixed shadow
benchmark—starting with the weak `well`, `greenhouse`, and `water_tank` classes—
then retrain/export a new version and repeat the same `all` mission. Do not
change the existing auto-labeling logic while doing this calibration work.

## Data persistence and reset policy

The farm Helm release is disposable. The 2026-09-14 reinstall showed that its
Helm-owned evidence PVC and backing volume were deleted during `helm uninstall`.
Training data must not live in that release.

Use a separate data/ML project for perception assets:

- namespace: `arhkp1-farm-drone-ml`;
- independent release or manifests: `farm-drone-ml-data`;
- persistent volume: `perception-data`;
- initial size: 10 GiB or larger after measuring the dataset;
- versioned directories for datasets, models, and evaluation reports.

The recorder and GPU training Job should run in the ML namespace and connect to
the farm's Zenoh service across namespaces. This lets the farm deployment be
uninstalled and reinstalled without touching training data.

Additional safeguards:

- set the data volume's backing PV reclaim policy to `Retain`, or export the data
  to object storage before deleting the data project;
- add `helm.sh/resource-policy: keep` in the data chart itself as a second guard;
- never put the ML PVC in the `farm-drone` Helm chart;
- use a reset script that only targets the farm release and verifies the ML data
  namespace remains present;
- copy/export important model and dataset versions before destructive cluster
  maintenance.

The farm `/artifacts` volume can continue holding disposable mission evidence,
but the perception dataset and trained models must use the independent ML storage.

The ML project now also includes a read-only showcase viewer. It mounts the
perception PVC read-only and presents dataset versions, sample images, model
files, and evaluation reports through an OpenShift Route. This gives the final
demo a human-friendly surface without allowing the viewer to modify training
artifacts.

The target lifecycle is:

```text
run survey -> record mission episode -> generate labels -> train detector
-> evaluate detector -> deploy better model -> run the next survey
```

## Important boundary

`/drone/target_truth` may be used offline to generate training labels and to
measure evaluation accuracy. It must not be the runtime source of detections.
The current truth-driven behavior in `observer_node.py` is therefore the main
piece to replace.

## Patterns to reuse from `hp-roscon-flywheel`

- Episode metadata: mission ID, scenario/seed, model version, route, frames,
  detections, outcome, and inference latency.
- Dataset assembly: select valid episodes and package them into a reproducible
  training dataset.
- Fixed evaluation: run detector versions on the same seeded scenes and compare
  precision, recall, missed detections, false positives, pose error, and latency.
- Model lineage: publish a latched model-version topic and include it in every
  detection and mission report.
- Promotion gate: use a new detector only when evaluation shows an improvement.

The first implementation should not copy the flywheel repo's Kafka, MinIO,
Model Registry, signing, or GitOps infrastructure. Those can come later.

## Free/open workflow

Use the free PyTorch/torchvision baseline already in this repository, with ONNX
Runtime as the planned inference target. Verify the license of any pretrained
weights separately.
Do not choose Ultralytics by default: its current free path is AGPL-3.0, while
private or proprietary use may require an Enterprise license.

## First milestone

Build a half-day to one-day spike that records Gazebo camera frames and uses
simulator truth to generate labels. Then train and evaluate a small baseline
detector offline before integrating it into the mission node.

Expected effort:

- proof of concept: about 1 day;
- credible ML v1: about 3-5 days;
- robust evaluation, deployment, and model promotion: about 1-2 weeks.

## Validation and demo plan

The flywheel is not complete when a model trains successfully. We need to prove
the complete loop is repeatable and that the live detector is actually using the
camera. Every run should leave inspectable artifacts on the persistent ML volume.

### Required evidence

Each version should produce:

- an episode manifest containing the scenario, seed, route, frame count, and
  recorder version;
- a dataset manifest containing class counts and train/validation/test seeds;
- a versioned model artifact and training configuration;
- an evaluation JSON/report with per-class precision, recall, missed detections,
  false positives, and inference latency;
- a mission report containing detector model version, detections, confidence,
  and mission outcome;
- a short recording or screenshots showing the live detection output.

The showcase viewer is available from the `farm-drone-ml-data-viewer` Route in
`arhkp1-farm-drone-ml`, or locally with a port-forward to its Service. It starts
empty and will populate as the recorder, trainer, and evaluator write versioned
artifacts to the ML volume.

### Acceptance gates

1. **Dataset gate:** all seven configured classes are represented, labels can be
   drawn back onto the source images, and the test scenes/seeds were not used for
   training.
2. **Offline model gate:** the held-out report is reproducible and meets the
   initial baseline of at least 80% macro precision and 80% macro recall, with
   no class below 60% recall. These are starting demonstration thresholds, not
   production guarantees; they can be tightened after the first baseline.
3. **Runtime isolation gate:** the detector subscribes to camera topics only.
   The running detector and observer must not subscribe to
   `/drone/target_truth`. Truth may be enabled separately after the mission for
   scoring.
4. **Mission gate:** on the fixed `all` scenario, the camera detector identifies
   the expected farm assets, publishes model version and confidence, and the
   mission reaches `COMPLETE` with return and landing. Missed or low-confidence
   objects must be visible in the report rather than silently converted into
   truth detections.
5. **Repeatability gate:** a second run with the same seed produces the same
   dataset manifest and comparable evaluation results. A new model is promoted
   only when it beats the fixed evaluation baseline.
6. **Persistence gate:** uninstalling and reinstalling only the farm Helm
   release does not remove the dataset, model, or evaluation report from the
   separate ML namespace.

### Recording sequence

The final demo should follow one story rather than showing disconnected tools:

```text
show baseline mission
  -> run recorder flight
  -> inspect images with projected labels
  -> show dataset manifest and train the free baseline
  -> show held-out evaluation report
  -> deploy the versioned ONNX model
  -> run the same mission using camera inference
  -> show live detections and final mission report
  -> optionally uninstall/reinstall farm and show ML artifacts remain
```

The first recording should preserve the current truth-driven run as a baseline,
then demonstrate the camera-only run separately. That makes it clear which
behavior existed before ML and which behavior was produced by the flywheel.

## Detailed implementation plan

### Phase 0 — Lock the perception contract

Before training, settle the class list and label format. The current configuration
has seven target types (`well`, `storage_silo`, `cattle_shed`, `produce_barn`,
`crop_field`, `greenhouse`, and `water_tank`), while the README describes five.
The configuration and documentation should agree.

Add the visible dimensions of each target to the scenario configuration, or derive
them from the SDF models. `TargetTruth` currently provides a target center pose,
which is not enough by itself to create a 2-D detector bounding box.

The first detector contract should contain:

- class name;
- confidence;
- image bounding box;
- estimated `farm_map` pose;
- model version;
- source image and timestamp.

### Phase 1 — Build a synthetic dataset recorder

Add a recorder that subscribes to the camera image, camera info, drone pose, and
offline truth. It should save:

- the image;
- the camera/pose metadata;
- the target labels;
- the scenario and randomization seed.

Generate labels by projecting the known target geometry into the camera image. The
truth topic is allowed here because this is dataset generation, not live inference.
Split data by scene/seed rather than by adjacent frames so train and test images do
not contain nearly identical views.

Deliverable: a small dataset and a script that can reproduce it.

### Phase 2 — Train and evaluate a free baseline

Use a free torchvision Faster R-CNN baseline first, then move to an Apache-2.0
detector stack such as YOLOX or Detectron2, and train on the synthetic dataset.
Export the promoted model to ONNX. Run offline evaluation on a
held-out seed set and record per-class precision, recall, missed detections, false
positives, pose error, and inference latency.

Deliverable: a versioned model artifact and a reproducible evaluation JSON/report.

### Phase 3 — Add runtime inference

Add a `vision_detector` ROS node that loads the ONNX model with ONNX Runtime and
subscribes to `/drone/camera/image_raw`. It publishes real detections without
subscribing to `/drone/target_truth`.

The observer should consume those detections during `INSPECT`, wait for a bounded
observation window, and handle both successful and missed detections. A low
confidence detection should not automatically become mission truth.

Deliverable: a live mission can produce detections from camera images alone.

### Phase 4 — Integrate and verify the mission

Replace `_targets_near()` and the fixed confidence in `observer_node.py`. Add the
detector model version to `TargetDetection` and `report.json`. Keep truth enabled
only in an explicit evaluation mode that calculates precision/recall after the
mission.

Add tests for:

- correct label projection;
- no-target frames;
- missed and low-confidence detections;
- model-version propagation;
- report generation;
- detector evaluation and promotion thresholds.

### Phase 5 — Add the flywheel later

Once detector v1 works locally, add the larger flywheel pieces in this order:

1. mission dataset curation;
2. reproducible training dataset assembly;
3. fixed-seed v1 versus v2 evaluation;
4. model promotion gate;
5. model artifact storage and version registry;
6. signing/GitOps rollout if deployment governance becomes a requirement.

Kafka, MinIO, and model signing are not required for the first ML milestone.

## Distant feature — GitOps deployment with Argo CD

Keep the current Helm workflow for local development. Once the drone has a
versioned detector and a persistent OpenShift deployment, add Argo CD as the
deployment and reconciliation layer.

Potential future flow:

```text
detector passes evaluation -> approved Git change -> Argo CD syncs deployment
-> farm runs the approved model -> Git revert provides rollback
```

Argo CD would manage the farm's Kubernetes/OpenShift resources—world, observer,
Zenoh, noVNC, storage, and later the detector model reference. It would not fly
the drone, run survey missions, or replace ROS/Nav2. Git would become the
approved desired state, while Argo CD would keep the cluster aligned with it.

Possible future files:

- `argocd/farm-demo-app.yaml`;
- environment-specific Helm values;
- immutable observer/model image digests;
- promotion and rollback documentation.

This is intentionally distant: do not add Argo CD before the perception model,
evaluation gate, and persistent OpenShift deployment are useful on their own.

## First coding slice

The first implementation should be limited to Phase 0 and the beginning of Phase 1:

1. reconcile the target classes;
2. add target dimensions to configuration;
3. capture a small set of images at varied drone poses;
4. write labels and inspect the resulting dataset;
5. do not integrate a model into the mission yet.

This is the fastest way to validate that the rendered camera view and projected
labels are useful before spending time on model training.

The recorder implementation now exists as an opt-in workload in `helm-ml`. It
writes image files plus YOLO and JSON labels to versioned `flight-*` directories,
and the ML showcase viewer exposes them. The v6 capture and corrected curation
are complete. The previous v3/v4 model metrics were based on badly positioned
labels and must not be used as promotion evidence.
