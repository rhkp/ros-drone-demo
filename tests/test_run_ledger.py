import importlib.util
import json
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "helm" / "drone-ml-pipeline" / "common" / "run_ledger.py"


def load_ledger(monkeypatch, tmp_path):
    record = tmp_path / "runs" / "run-1" / "run.json"
    monkeypatch.setenv("RUN_ID", "run-1")
    monkeypatch.setenv("RUN_RECORD_PATH", str(record))
    spec = importlib.util.spec_from_file_location("run_ledger_test", MODULE_PATH)
    ledger = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ledger)
    return ledger, record


def test_run_ledger_persists_stage_and_status(monkeypatch, tmp_path):
    ledger, record = load_ledger(monkeypatch, tmp_path)

    ledger.mark_stage("curation", "running", dataset="/data/perception/datasets/curated-run-1")
    ledger.mark_stage("curation", "completed", frames=12)
    ledger.mark_run("rejected", failure_stage="evaluation")

    payload = json.loads(record.read_text(encoding="utf-8"))
    assert payload["run_id"] == "run-1"
    assert payload["status"] == "rejected"
    assert payload["stages"]["curation"]["status"] == "completed"
    assert payload["stages"]["curation"]["frames"] == 12
