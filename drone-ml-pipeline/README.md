# Drone ML pipeline

This directory is the entry point for the perception data flywheel: capture,
label, train, validate, and promote a model.

The deployable implementation lives in [`../helm/drone-ml-pipeline/`](../helm/drone-ml-pipeline/), organized as:

- `training/` — GPU training job and model checkpoint workflow.
- `labeling/` — dataset assembly and curation from captured flights.
- `verification/` — perception validation against simulator truth.
- `showcase/` — read-only dataset, model, and evaluation viewer.
- `runtime/` — camera detector and dataset recorder deployments.

The current active model is the Faster R-CNN PyTorch checkpoint. The former ONNX
experiment is retained separately under [`../archived/onnx/`](../archived/onnx/).
