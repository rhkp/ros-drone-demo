#!/usr/bin/env python3
"""Small read-only browser for farm-drone ML artifacts.

The viewer intentionally uses only Python's standard library. It mounts the ML
PVC read-only and exposes metadata, reports, and sample images without adding a
database or a second persistence system.
"""

import html
import json
import mimetypes
import os
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse


DATA_ROOT = Path(os.environ.get("DATA_ROOT", "/data/perception")).resolve()
PORT = int(os.environ.get("PORT", "8080"))
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".webp"}
MODEL_SUFFIXES = {".pt", ".engine"}


def relative(path):
    return str(path.relative_to(DATA_ROOT))


def files_under(path, suffixes=None):
    if not path.is_dir():
        return []
    return sorted(
        (item for item in path.rglob("*") if item.is_file()
         and (suffixes is None or item.suffix.lower() in suffixes)),
        key=lambda item: str(item),
    )


def read_json(path):
    try:
        with path.open(encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, ValueError, TypeError):
        return None


def first_json(directory):
    for name in ("dataset.json", "manifest.json", "dataset_manifest.json", "report.json"):
        candidate = directory / name
        if candidate.is_file():
            value = read_json(candidate)
            if value is not None:
                return relative(candidate), value
    candidates = files_under(directory, {".json"})
    if candidates:
        value = read_json(candidates[0])
        if value is not None:
            return relative(candidates[0]), value
    return None, None


def class_counts(manifest):
    if not isinstance(manifest, dict):
        return {}
    values = manifest.get("class_counts") or manifest.get("classes") or {}
    if not isinstance(values, dict):
        return {}
    return values


def summarize_dataset(directory):
    manifest_path, manifest = first_json(directory)
    images = files_under(directory / "images", IMAGE_SUFFIXES)
    label_files = files_under(directory / "labels")
    label_json_files = {item.stem: item for item in label_files if item.suffix.lower() == ".json"}
    if not images:
        images = files_under(directory, IMAGE_SUFFIXES)
    sample_images = []
    for image in images:
        label_path = label_json_files.get(image.stem)
        label_payload = read_json(label_path) if label_path else None
        sample_images.append({
            "path": relative(image),
            "name": image.name,
            "label_path": relative(label_path) if label_path else None,
            "image_width": (label_payload or {}).get("image_width", 320),
            "image_height": (label_payload or {}).get("image_height", 240),
            "annotations": (label_payload or {}).get("annotations", []),
        })
    return {
        "name": directory.name,
        "path": relative(directory),
        "images": len(images),
        "labels": len(label_json_files),
        "label_files": len(label_files),
        "manifest": manifest_path,
        "class_counts": class_counts(manifest),
        "seed": manifest.get("seed") if isinstance(manifest, dict) else None,
        "sample_images": sample_images,
    }


def summarize_model(directory):
    models = files_under(directory, MODEL_SUFFIXES)
    manifest_path, manifest = first_json(directory)
    return {
        "name": directory.name,
        "path": relative(directory),
        "files": [{"path": relative(item), "bytes": item.stat().st_size} for item in models],
        "manifest": manifest_path,
        "version": (manifest or {}).get("model_version") if isinstance(manifest, dict) else None,
    }


def metric_value(report, *names):
    if not isinstance(report, dict):
        return None
    for name in names:
        if name in report:
            return report[name]
        metrics = report.get("metrics")
        if isinstance(metrics, dict) and name in metrics:
            return metrics[name]
    return None


def summarize_evaluation(directory):
    report_path, report = first_json(directory)
    return {
        "name": directory.name,
        "path": relative(directory),
        "report": report_path,
        "model_version": metric_value(report, "model_version"),
        "precision": metric_value(report, "macro_precision", "precision"),
        "recall": metric_value(report, "macro_recall", "recall"),
        "latency_ms": metric_value(report, "inference_latency_ms", "latency_ms"),
    }


def build_summary():
    datasets_root = DATA_ROOT / "datasets"
    models_root = DATA_ROOT / "models"
    evaluations_root = DATA_ROOT / "evaluations"
    datasets = [summarize_dataset(item) for item in sorted(datasets_root.iterdir()) if item.is_dir()] if datasets_root.is_dir() else []
    models = [summarize_model(item) for item in sorted(models_root.iterdir()) if item.is_dir()] if models_root.is_dir() else []
    evaluations = [summarize_evaluation(item) for item in sorted(evaluations_root.iterdir()) if item.is_dir()] if evaluations_root.is_dir() else []
    images = files_under(datasets_root, IMAGE_SUFFIXES)[:30]
    return {
        "data_root": str(DATA_ROOT),
        "data_root_exists": DATA_ROOT.is_dir(),
        "datasets": datasets,
        "models": models,
        "evaluations": evaluations,
        "images": [
            {"path": relative(item), "name": item.name}
            for item in images
        ],
    }


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Farm Drone ML Showcase</title>
<style>
:root { color-scheme: dark; --bg:#10151d; --panel:#192331; --line:#2d3b4d; --text:#e8eef5; --muted:#9eacbb; --accent:#61d095; --warn:#f4c95d; }
* { box-sizing:border-box; } body { margin:0; background:var(--bg); color:var(--text); font:15px system-ui,-apple-system,sans-serif; }
header { padding:30px max(22px,calc((100% - 1180px)/2)); background:linear-gradient(120deg,#153c3b,#1d2943); border-bottom:1px solid var(--line); }
h1 { margin:0 0 8px; font-size:28px; } h2 { margin:0 0 14px; font-size:19px; } p { color:var(--muted); }
main { max-width:1180px; margin:24px auto; padding:0 22px; } .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(230px,1fr)); gap:14px; margin-bottom:22px; }
.card, section { background:var(--panel); border:1px solid var(--line); border-radius:10px; padding:18px; } .metric { font-size:30px; font-weight:700; color:var(--accent); }
.label { color:var(--muted); font-size:13px; } table { width:100%; border-collapse:collapse; } th,td { text-align:left; padding:9px 7px; border-bottom:1px solid var(--line); vertical-align:top; } th { color:var(--muted); font-weight:500; }
code, pre { color:#c8e7d5; } code { word-break:break-word; } a { color:#8ac7ff; } .empty { color:var(--muted); padding:14px 0; } .gallery { display:grid; grid-template-columns:repeat(auto-fill,minmax(150px,1fr)); gap:12px; }
.gallery figure { margin:0; background:#111923; border:1px solid var(--line); border-radius:8px; padding:7px; } .thumb { position:relative; line-height:0; } .gallery img { width:100%; height:115px; object-fit:cover; border-radius:5px; background:#0b0f14; } .box { position:absolute; border:2px solid #ffdf5d; box-shadow:0 0 0 1px #13202b; pointer-events:none; } .box span { position:absolute; left:-2px; top:-18px; background:#ffdf5d; color:#10151d; font-size:10px; line-height:16px; padding:0 4px; white-space:nowrap; } figcaption { font-size:11px; color:var(--muted); overflow-wrap:anywhere; margin-top:5px; line-height:1.35; }
.pill { display:inline-block; padding:3px 8px; border-radius:999px; background:#24493c; color:#9ef0bd; font-size:12px; } .error { color:#ffb4a9; }
</style>
</head>
<body>
<header><h1>Farm Drone ML Showcase</h1><p>Human-friendly view of the perception flywheel: datasets → models → evaluation → mission.</p><span id="status" class="pill">Loading artifacts…</span></header>
<main>
<div class="grid"><div class="card"><div class="label">Dataset versions</div><div id="dataset-count" class="metric">—</div></div><div class="card"><div class="label">Model versions</div><div id="model-count" class="metric">—</div></div><div class="card"><div class="label">Evaluation reports</div><div id="eval-count" class="metric">—</div></div><div class="card"><div class="label">Persistent store</div><div id="root" class="metric">—</div></div></div>
<section><h2>Datasets</h2><div id="datasets" class="empty">No datasets recorded yet.</div></section><br>
<section><h2>Models</h2><div id="models" class="empty">No models trained yet.</div></section><br>
<section><h2>Evaluation</h2><div id="evaluations" class="empty">No evaluation reports yet.</div></section><br>
<section><h2>Images and labels</h2><p>Choose a dataset and how many frames to display. Bounding boxes are generated from simulator truth offline and drawn over the camera image for inspection.</p><label for="dataset-select" class="label">Dataset</label> <select id="dataset-select"></select> <label for="image-limit" class="label">Frames</label> <select id="image-limit"><option value="30">First 30</option><option value="100">First 100</option><option value="all" selected>All available</option></select><div id="image-count" class="label"></div><div id="images" class="gallery"><div class="empty">No images recorded yet.</div></div></section>
</main>
<script>
const esc = (value) => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const link = (path, label) => path ? `<a href="/files/${path.split('/').map(encodeURIComponent).join('/')}" target="_blank">${esc(label || path)}</a>` : '—';
const value = (x) => x === null || x === undefined || x === '' ? '—' : esc(x);
function table(headers, rows) { return `<table><thead><tr>${headers.map(h=>`<th>${h}</th>`).join('')}</tr></thead><tbody>${rows.join('')}</tbody></table>`; }
function fileUrl(path) { return `/files/${path.split('/').map(encodeURIComponent).join('/')}`; }
function renderGallery(dataset) {
  const target = document.querySelector('#images');
  const limit = document.querySelector('#image-limit').value;
  const images = !dataset ? [] : (limit === 'all' ? dataset.sample_images : dataset.sample_images.slice(0, Number(limit)));
  document.querySelector('#image-count').textContent = dataset ? `Showing ${images.length} of ${dataset.images} available frame(s)` : '';
  if (!images.length) { target.innerHTML = '<div class="empty">No images recorded yet.</div>'; return; }
  target.innerHTML = images.map(image => {
    const boxes = (image.annotations || []).map(annotation => {
      const box = annotation.bbox_pixels || {};
      const left = 100 * Number(box.x_min || 0) / Number(image.image_width || 320);
      const top = 100 * Number(box.y_min || 0) / Number(image.image_height || 240);
      const width = 100 * Number(box.width || 0) / Number(image.image_width || 320);
      const height = 100 * Number(box.height || 0) / Number(image.image_height || 240);
      return `<div class="box" style="left:${left}%;top:${top}%;width:${width}%;height:${height}%;"><span>${esc(annotation.class_name)}</span></div>`;
    }).join('');
    const labelLink = image.label_path ? ` · <a href="${fileUrl(image.label_path)}" target="_blank">labels</a>` : '';
    return `<figure><div class="thumb"><img loading="lazy" src="${fileUrl(image.path)}" alt="${esc(image.name)}">${boxes}</div><figcaption>${esc(image.path)}<br>${image.annotations.length ? `${image.annotations.length} label(s)` : 'no labels'}${labelLink}</figcaption></figure>`;
  }).join('');
}
function populateDatasetSelect(datasets) {
  const select = document.querySelector('#dataset-select');
  const current = select.value;
  select.innerHTML = datasets.map(d => `<option value="${esc(d.name)}">${esc(d.name)} — ${d.images} images</option>`).join('');
  const curatedDatasets = datasets.filter(d => d.name.startsWith('curated-'));
  const flightDatasets = datasets.filter(d => d.name.startsWith('flight-'));
  const preferred = current || (curatedDatasets.length ? curatedDatasets[curatedDatasets.length - 1].name : (flightDatasets.length ? flightDatasets[flightDatasets.length - 1].name : (datasets[0] || {}).name));
  if (preferred) select.value = preferred;
  renderGallery(datasets.find(d => d.name === select.value));
  select.onchange = () => renderGallery(datasets.find(d => d.name === select.value));
  document.querySelector('#image-limit').onchange = () => renderGallery(datasets.find(d => d.name === select.value));
}
async function refresh() {
  try {
    const data = await fetch('/api/summary', {cache:'no-store'}).then(r => r.json());
    document.querySelector('#status').textContent = data.data_root_exists ? 'ML volume connected' : 'ML volume not mounted';
    document.querySelector('#dataset-count').textContent = data.datasets.length;
    document.querySelector('#model-count').textContent = data.models.length;
    document.querySelector('#eval-count').textContent = data.evaluations.length;
    document.querySelector('#root').textContent = data.data_root_exists ? 'Ready' : 'Missing';
    document.querySelector('#datasets').innerHTML = data.datasets.length ? table(['Version','Images','Labels','Seed','Manifest'], data.datasets.map(d=>`<tr><td><code>${value(d.name)}</code></td><td>${value(d.images)}</td><td>${value(d.labels)}</td><td>${value(d.seed)}</td><td>${link(d.manifest,'view JSON')}</td></tr>`)) : '<div class="empty">No datasets recorded yet.</div>';
    document.querySelector('#models').innerHTML = data.models.length ? table(['Version','PyTorch model files','Manifest'], data.models.map(m=>`<tr><td><code>${value(m.version || m.name)}</code></td><td>${m.files.map(f=>link(f.path, `${f.path.split('/').pop()} (${f.bytes} bytes)`)).join('<br>') || '—'}</td><td>${link(m.manifest,'view JSON')}</td></tr>`)) : '<div class="empty">No models trained yet.</div>';
    document.querySelector('#evaluations').innerHTML = data.evaluations.length ? table(['Run','Model','Precision','Recall','Latency','Report'], data.evaluations.map(e=>`<tr><td><code>${value(e.name)}</code></td><td>${value(e.model_version)}</td><td>${value(e.precision)}</td><td>${value(e.recall)}</td><td>${value(e.latency_ms)} ms</td><td>${link(e.report,'view JSON')}</td></tr>`)) : '<div class="empty">No evaluation reports yet.</div>';
    populateDatasetSelect(data.datasets);
  } catch (error) { document.querySelector('#status').textContent = 'Viewer error'; document.querySelector('#status').className = 'pill error'; }
}
refresh(); setInterval(refresh, 15000);
</script>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def send_bytes(self, payload, content_type, status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        parsed = urlparse(self.path)
        path = unquote(parsed.path)
        if path == "/healthz":
            self.send_bytes(b"ok\n", "text/plain; charset=utf-8")
            return
        if path == "/api/summary":
            payload = json.dumps(build_summary(), sort_keys=True).encode()
            self.send_bytes(payload, "application/json; charset=utf-8")
            return
        if path.startswith("/files/"):
            candidate = (DATA_ROOT / path.removeprefix("/files/")).resolve()
            if candidate != DATA_ROOT and DATA_ROOT in candidate.parents and candidate.is_file():
                content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
                try:
                    self.send_bytes(candidate.read_bytes(), content_type)
                except OSError:
                    self.send_bytes(b"file unavailable\n", "text/plain; charset=utf-8", 404)
                return
            self.send_bytes(b"not found\n", "text/plain; charset=utf-8", 404)
            return
        if path == "/" or path == "/index.html":
            self.send_bytes(PAGE.encode(), "text/html; charset=utf-8")
            return
        self.send_bytes(b"not found\n", "text/plain; charset=utf-8", 404)

    def log_message(self, fmt, *args):
        print("viewer:", fmt % args, flush=True)


if __name__ == "__main__":
    DATA_ROOT.mkdir(parents=True, exist_ok=True)
    print(f"Farm Drone ML viewer listening on :{PORT}; data root: {DATA_ROOT}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
