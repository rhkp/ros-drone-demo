"""Persist command-triggered flywheel run state on the ML PVC."""

import json
import os
from datetime import datetime, timezone
from pathlib import Path


RUN_ID = os.environ.get("RUN_ID", "").strip()
RECORD_PATH = Path(os.environ.get("RUN_RECORD_PATH", "")).resolve() if os.environ.get("RUN_RECORD_PATH") else None


def _now():
    return datetime.now(timezone.utc).isoformat()


def _load():
    if RECORD_PATH and RECORD_PATH.is_file():
        try:
            value = json.loads(RECORD_PATH.read_text(encoding="utf-8"))
            if isinstance(value, dict):
                return value
        except (OSError, ValueError):
            pass
    return {
        "schema_version": "pipeline-run-1",
        "run_id": RUN_ID,
        "status": "running",
        "started_at": _now(),
        "stages": {},
    }


def _write(value):
    if not RECORD_PATH or not RUN_ID:
        return
    try:
        RECORD_PATH.parent.mkdir(parents=True, exist_ok=True)
        temporary = RECORD_PATH.with_name(f".{RECORD_PATH.name}.{os.getpid()}.tmp")
        temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, RECORD_PATH)
    except OSError:
        # Tracking must never turn a valid curation, training, or evaluation
        # result into a failed ML job.
        return


def mark_run(status=None, **fields):
    """Update the overall run without failing the ML job if tracking is unavailable."""
    if not RECORD_PATH or not RUN_ID:
        return
    value = _load()
    value["run_id"] = RUN_ID
    if status:
        value["status"] = status
        if status in {"promoted", "rejected", "failed"}:
            value["completed_at"] = _now()
    value.update(fields)
    _write(value)


def mark_stage(stage, status, **fields):
    """Record a stage transition and its useful artifact/metric fields."""
    if not RECORD_PATH or not RUN_ID:
        return
    value = _load()
    value["run_id"] = RUN_ID
    stages = value.setdefault("stages", {})
    entry = stages.setdefault(stage, {})
    if status == "running" and "started_at" not in entry:
        entry["started_at"] = _now()
    if status in {"completed", "failed"}:
        entry["completed_at"] = _now()
    entry["status"] = status
    entry.update(fields)
    _write(value)
