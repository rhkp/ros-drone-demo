#!/usr/bin/env bash
set -euo pipefail

# Run the reproducible ML flywheel and promote a model only when its held-out
# evaluation passes. The script intentionally leaves failed model artifacts on
# the persistent ML volume for inspection, but restores the previous runtime.

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
chart_dir="${CHART_DIR:-${repo_dir}/helm/drone-ml-pipeline}"
ml_release="${ML_RELEASE:-farm-drone-ml-data}"
ml_namespace="${ML_NAMESPACE:-farm-drone-ml}"
data_root="${DATA_ROOT:-/data/perception}"
run_id="${RUN_ID:-$(date +%Y%m%d-%H%M%S)}"
dataset_version="${DATASET_VERSION:-curated-${run_id}}"
model_version="${MODEL_VERSION:-model-${run_id}}"
eval_version="${EVAL_VERSION:-${model_version}-held-out}"
dataset_dir="${data_root}/datasets/${dataset_version}"
model_dir="${data_root}/models/${model_version}"
eval_dir="${data_root}/evaluations/${eval_version}"
report_path="${eval_dir}/report.json"
run_record_path="${data_root}/runs/${run_id}/run.json"

# Override these with JSON arrays/objects after creating a complete test episode.
# Every run needs train, validation, and test coverage for the trainer/evaluator.
if [[ -n "${SOURCE_DATASETS_JSON:-}" ]]; then
  source_datasets_json="${SOURCE_DATASETS_JSON}"
else
  source_datasets_json='["flight-v3","flight-v4","flight-v5","flight-v6","flight-test-v1"]'
fi
if [[ -n "${SPLIT_MAP_JSON:-}" ]]; then
  split_map_json="${SPLIT_MAP_JSON}"
else
  split_map_json='{"flight-v3":"train","flight-v4":"train","flight-v5":"validation","flight-v6":"train","flight-test-v1":"test"}'
fi

detector_was_enabled=false
viewer_was_enabled=false
anchor_was_enabled=false
finalized=false

log() {
  printf '[retrain] %s\n' "$*"
}

python3 - "${source_datasets_json}" "${split_map_json}" <<'PY'
import json
import sys

sources = json.loads(sys.argv[1])
split_map = json.loads(sys.argv[2])
required = {"train", "validation", "test"}
if not isinstance(sources, list) or not sources:
    raise SystemExit("SOURCE_DATASETS_JSON must be a non-empty JSON array")
if not isinstance(split_map, dict) or not required.issubset(split_map.values()):
    raise SystemExit("SPLIT_MAP_JSON must assign at least one episode to train, validation, and test")
missing = sorted(set(split_map) - set(sources))
if missing:
    raise SystemExit(f"SPLIT_MAP_JSON names sources not present in SOURCE_DATASETS_JSON: {missing}")
PY

resource_exists() {
  oc get deployment "$1" -n "${ml_namespace}" >/dev/null 2>&1
}

if resource_exists "${ml_release}-detector"; then detector_was_enabled=true; fi
if resource_exists "${ml_release}-viewer"; then viewer_was_enabled=true; fi
if resource_exists "${ml_release}-storage-anchor"; then anchor_was_enabled=true; fi

helm_ml() {
  helm upgrade "${ml_release}" "${chart_dir}" \
    --namespace "${ml_namespace}" --reuse-values "$@" --wait
}

wait_for_job() {
  local job_name="$1"
  local deadline=$(( $(date +%s) + 3600 ))
  local complete failed
  log "waiting for ${job_name}"
  while (( $(date +%s) < deadline )); do
    complete=$(oc get job -n "${ml_namespace}" "${job_name}" \
      -o jsonpath='{.status.conditions[?(@.type=="Complete")].status}' 2>/dev/null || true)
    failed=$(oc get job -n "${ml_namespace}" "${job_name}" \
      -o jsonpath='{.status.conditions[?(@.type=="Failed")].status}' 2>/dev/null || true)
    if [[ "${complete}" == "True" ]]; then
      oc logs -n "${ml_namespace}" "job/${job_name}"
      return 0
    fi
    if [[ "${failed}" == "True" ]]; then
      oc logs -n "${ml_namespace}" "job/${job_name}" || true
      return 1
    fi
    sleep 10
  done
  oc logs -n "${ml_namespace}" "job/${job_name}" || true
  return 1
}

mark_promoted_run() {
  local anchor_pod record_tmp
  anchor_pod=$(oc get pod -n "${ml_namespace}" \
    -l "app.kubernetes.io/component=storage-anchor" \
    -o jsonpath='{.items[0].metadata.name}' 2>/dev/null || true)
  if [[ -z "${anchor_pod}" ]]; then
    log "storage anchor unavailable; leaving run status at candidate_passed"
    return 0
  fi
  record_tmp=$(mktemp)
  if ! oc cp -n "${ml_namespace}" \
    "${anchor_pod}:${run_record_path}" "${record_tmp}" >/dev/null 2>&1; then
    rm -f "${record_tmp}"
    log "could not read persistent run record; leaving status at candidate_passed"
    return 0
  fi
  python3 - "${record_tmp}" "${model_version}" "${model_dir}/detector.pt" <<'PY'
import json
import sys
from datetime import datetime, timezone

path, model_version, model_path = sys.argv[1:]
with open(path, encoding="utf-8") as stream:
    record = json.load(stream)
record["status"] = "promoted"
record["completed_at"] = datetime.now(timezone.utc).isoformat()
record["promotion"] = {"status": "promoted", "model_version": model_version, "model_path": model_path}
with open(path, "w", encoding="utf-8") as stream:
    json.dump(record, stream, indent=2)
    stream.write("\n")
PY
  oc cp -n "${ml_namespace}" "${record_tmp}" \
    "${anchor_pod}:${run_record_path}" >/dev/null 2>&1 || \
    log "could not publish promoted status; run record remains available"
  rm -f "${record_tmp}"
}

delete_runtime_deployments() {
  # Helm does not prune Deployments that disappeared when an enabled flag is
  # set to false. Delete only these disposable consumers; the PVC and all
  # datasets/models remain untouched.
  oc delete deployment -n "${ml_namespace}" \
    "${ml_release}-detector" \
    "${ml_release}-viewer" \
    "${ml_release}-storage-anchor" \
    --ignore-not-found --wait=true >/dev/null

  local deadline=$(( $(date +%s) + 180 ))
  local remaining
  while (( $(date +%s) < deadline )); do
    remaining="$(
      oc get pods -n "${ml_namespace}" \
        -l 'app.kubernetes.io/component in (camera-detector,perception-showcase,storage-anchor)' \
        -o name 2>/dev/null || true
    )"
    if [[ -z "${remaining}" ]]; then
      return 0
    fi
    sleep 5
  done
  log "runtime pods are still terminating; continuing so Kubernetes can finish PVC detach"
}

restore_previous_runtime() {
  if [[ "${finalized}" == true ]]; then
    return
  fi
  log "restoring previous detector/viewer/storage state"
  # The PVC is RWO and the detector needs a GPU. Restore the detector first
  # while the viewer/anchor are stopped, then restore the read-only consumers
  # after the detector has claimed the volume on a GPU-capable node.
  helm_ml \
    --set curator.enabled=false \
    --set trainer.enabled=false \
    --set heldOutEvaluator.enabled=false \
    --set detector.enabled=false \
    --set viewer.enabled=false \
    --set anchor.enabled=false || true
  delete_runtime_deployments || true
  if [[ "${detector_was_enabled}" == true ]]; then
    helm_ml \
      --set curator.enabled=false \
      --set trainer.enabled=false \
      --set heldOutEvaluator.enabled=false \
      --set detector.enabled=true \
      --set viewer.enabled=false \
      --set anchor.enabled=false || true
  fi
  helm_ml \
    --set curator.enabled=false \
    --set trainer.enabled=false \
    --set heldOutEvaluator.enabled=false \
    --set detector.enabled="${detector_was_enabled}" \
    --set viewer.enabled="${viewer_was_enabled}" \
    --set anchor.enabled="${anchor_was_enabled}" || true
}

trap restore_previous_runtime EXIT

log "release=${ml_release} namespace=${ml_namespace}"
log "dataset=${dataset_dir} model=${model_dir} report=${report_path}"

# Remove any previous one-shot Jobs before creating this run's Jobs. Their data
# remains on the PVC; only the disposable Job resources are reconciled.
helm_ml \
  --set curator.enabled=false \
  --set trainer.enabled=false \
  --set heldOutEvaluator.enabled=false \
  --set detector.enabled=false \
  --set viewer.enabled="${viewer_was_enabled}" \
  --set anchor.enabled="${anchor_was_enabled}"

log "curating immutable dataset"
helm_ml \
  --set curator.enabled=true \
  --set-json "curator.sources=${source_datasets_json}" \
  --set-json "curator.splitMap=${split_map_json}" \
  --set curator.outputDir="${dataset_dir}" \
  --set curator.runId="${run_id}" \
  --set curator.runRecordPath="${run_record_path}" \
  --set trainer.enabled=false \
  --set heldOutEvaluator.enabled=false \
  --set detector.enabled=false
wait_for_job "${ml_release}-curator"

# Free the RWO volume and the GPU before starting the trainer.
helm_ml \
  --set curator.enabled=false \
  --set trainer.enabled=false \
  --set heldOutEvaluator.enabled=false \
  --set detector.enabled=false \
  --set viewer.enabled=false \
  --set anchor.enabled=false
delete_runtime_deployments

log "training ${model_version} on the GPU"
helm_ml \
  --set curator.enabled=false \
  --set trainer.enabled=true \
  --set trainer.datasetDir="${dataset_dir}" \
  --set trainer.modelDir="${model_dir}" \
  --set trainer.evalDir="${eval_dir}" \
  --set trainer.runId="${run_id}" \
  --set trainer.runRecordPath="${run_record_path}" \
  --set heldOutEvaluator.enabled=false \
  --set detector.enabled=false \
  --set viewer.enabled=false \
  --set anchor.enabled=false
wait_for_job "${ml_release}-trainer"

log "evaluating ${model_version} on the test split"
helm_ml \
  --set trainer.enabled=false \
  --set heldOutEvaluator.enabled=true \
  --set heldOutEvaluator.datasetDir="${dataset_dir}" \
  --set heldOutEvaluator.split=test \
  --set heldOutEvaluator.modelPath="${model_dir}/detector.pt" \
  --set heldOutEvaluator.modelVersion="${model_version}" \
  --set heldOutEvaluator.reportPath="${report_path}" \
  --set heldOutEvaluator.runId="${run_id}" \
  --set heldOutEvaluator.runRecordPath="${run_record_path}" \
  --set heldOutEvaluator.enforceThresholds=true \
  --set detector.enabled=false \
  --set viewer.enabled=false \
  --set anchor.enabled=false
wait_for_job "${ml_release}-held-out-evaluator"

evaluator_pod=$(oc get pod -n "${ml_namespace}" \
  -l "job-name=${ml_release}-held-out-evaluator" \
  -o jsonpath='{.items[0].metadata.name}')
local_report="$(mktemp)"
trap 'rm -f "${local_report}"; restore_previous_runtime' EXIT
oc exec -n "${ml_namespace}" "pod/${evaluator_pod}" -- cat "${report_path}" > "${local_report}"

python3 - "${local_report}" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
gate = report.get("promotion_gate", {})
print(
    "[retrain] evaluation: "
    f"precision={report.get('macro_precision', 0):.3f} "
    f"recall={report.get('macro_recall', 0):.3f} "
    f"min_class_recall={gate.get('minimum_observed_class_recall', 0):.3f}"
)
if not report.get("held_out"):
    raise SystemExit("evaluation did not use the test split")
if not gate.get("passed"):
    raise SystemExit("promotion gate failed; current detector will be preserved")
PY

log "promotion gate passed; deploying ${model_version}"
helm_ml \
  --set curator.enabled=false \
  --set trainer.enabled=false \
  --set heldOutEvaluator.enabled=false \
  --set detector.enabled=true \
  --set detector.modelPath="${model_dir}/detector.pt" \
  --set detector.modelVersion="${model_version}" \
  --set viewer.enabled="${viewer_was_enabled}" \
  --set anchor.enabled="${anchor_was_enabled}"
mark_promoted_run
finalized=true
log "model ${model_version} promoted successfully"
