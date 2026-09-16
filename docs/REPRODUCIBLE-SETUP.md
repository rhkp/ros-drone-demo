# Reproducible setup

The repository has two independent Helm releases and two retained PVCs:

- `farm-drone` in namespace `farm-drone` — Gazebo/ROS demo and mission evidence.
- `farm-drone-ml-data` in namespace `farm-drone-ml` — datasets, models, and evaluation reports.

The chart paths are `helm/drone-demo` and `helm/drone-ml-pipeline`. The example
values files are safe templates; copy them to ignored `values.yaml` files only
when local overrides are needed.

## Build the images

On the validated build host, copy `versions.env.example` to `versions.env`, set
the registry and release tag, then run:

```bash
./scripts/build-images.sh all
```

Publish the resulting images to the registry referenced by the values files.

## Install the demo

```bash
helm upgrade --install farm-drone helm/drone-demo \
  --namespace farm-drone --create-namespace \
  --values helm/drone-demo/values.yaml.example --wait
```

## Install the ML pipeline

```bash
helm upgrade --install farm-drone-ml-data helm/drone-ml-pipeline \
  --namespace farm-drone-ml --create-namespace \
  --values helm/drone-ml-pipeline/values.yaml.example --wait
```

The ML chart defaults its Zenoh endpoint to the demo release and namespace
shown above. If those names change, update `demo.namespace` and
`demo.zenohService` in the ML values file.

## Safe reinstall

Both PVC templates use `helm.sh/resource-policy: keep`. Therefore this removes
the workloads while preserving mission evidence, datasets, checkpoints, and
evaluation reports:

```bash
helm uninstall farm-drone --namespace farm-drone
helm uninstall farm-drone-ml-data --namespace farm-drone-ml
```

Run the two install commands again to recreate the workloads. Deleting either
PVC is an explicit data-destructive operation and must be done separately.
