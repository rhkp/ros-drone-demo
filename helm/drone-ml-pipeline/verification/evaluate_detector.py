#!/usr/bin/env python3
"""Evaluate a trained Faster R-CNN checkpoint on a held-out image split."""

import argparse
import json
import os
import time
from pathlib import Path

import torch
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn


DEFAULT_CLASSES = (
    "well,storage_silo,cattle_shed,produce_barn,crop_field,greenhouse,water_tank"
)


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def iou(left, right):
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union else 0.0


class HeldOutDataset(Dataset):
    """Read only one dataset split; training data is never touched or changed."""

    def __init__(self, root, split, class_to_id):
        self.root = Path(root)
        self.split = split
        self.class_to_id = class_to_id
        self.image_dir = self.root / "images" / split
        self.label_dir = self.root / "labels" / split
        self.images = sorted(self.image_dir.glob("*.png"))
        if not self.images:
            raise RuntimeError(
                f"No held-out PNG images found in {self.image_dir}. "
                "Create a separate test episode and curate it into split=test."
            )

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        image_path = self.images[index]
        image = Image.open(image_path).convert("RGB")
        width, height = image.size
        payload = read_json(self.label_dir / f"{image_path.stem}.json")
        boxes, labels = [], []
        for annotation in payload.get("annotations", []):
            class_id = self.class_to_id.get(annotation.get("class_name"))
            box = annotation.get("bbox_pixels") or {}
            if not class_id:
                continue
            x1 = max(0.0, min(float(box.get("x_min", 0)), width - 1.0))
            y1 = max(0.0, min(float(box.get("y_min", 0)), height - 1.0))
            x2 = max(0.0, min(float(box.get("x_max", 0)), width))
            y2 = max(0.0, min(float(box.get("y_max", 0)), height))
            if x2 > x1 and y2 > y1:
                boxes.append([x1, y1, x2, y2])
                labels.append(class_id)
        pixels = torch.from_numpy(__import__("numpy").array(image)).permute(2, 0, 1).float() / 255.0
        target = {
            "boxes": torch.tensor(boxes, dtype=torch.float32).reshape(-1, 4),
            "labels": torch.tensor(labels, dtype=torch.int64),
            "image_id": torch.tensor([index]),
            "path": str(image_path),
        }
        return pixels, target


def collate(batch):
    return tuple(zip(*batch))


def load_model(model_path, class_names, device):
    checkpoint = torch.load(model_path, map_location=device, weights_only=False)
    checkpoint_classes = checkpoint.get("class_names") if isinstance(checkpoint, dict) else None
    if checkpoint_classes and list(checkpoint_classes) != list(class_names):
        raise RuntimeError(
            "Class order mismatch between checkpoint and evaluator: "
            f"checkpoint={checkpoint_classes}, evaluator={class_names}"
        )
    state_dict = checkpoint.get("state_dict", checkpoint) if isinstance(checkpoint, dict) else checkpoint
    model = fasterrcnn_mobilenet_v3_large_320_fpn(
        weights=None, weights_backbone=None, num_classes=len(class_names) + 1
    ).to(device)
    model.load_state_dict(state_dict)
    model.eval()
    return model, checkpoint


def evaluate(model, loader, class_names, device, confidence_threshold, iou_threshold):
    per_class = {name: {"tp": 0, "fp": 0, "fn": 0} for name in class_names}
    latencies = []
    evaluated_images = 0
    with torch.no_grad():
        for images, targets in loader:
            batch = [image.to(device) for image in images]
            started = time.perf_counter()
            predictions = model(batch)
            latencies.append((time.perf_counter() - started) * 1000.0 / max(1, len(batch)))
            for prediction, target in zip(predictions, targets):
                evaluated_images += 1
                matched = set()
                for box, label, score in zip(
                    prediction["boxes"].cpu().tolist(),
                    prediction["labels"].cpu().tolist(),
                    prediction["scores"].cpu().tolist(),
                ):
                    if score < confidence_threshold or not 1 <= label <= len(class_names):
                        continue
                    name = class_names[label - 1]
                    candidates = [
                        (index, iou(box, truth_box))
                        for index, (truth_box, truth_label) in enumerate(
                            zip(target["boxes"].tolist(), target["labels"].tolist())
                        )
                        if truth_label == label and index not in matched
                    ]
                    best = max(candidates, key=lambda item: item[1], default=(-1, 0.0))
                    if best[1] >= iou_threshold:
                        matched.add(best[0])
                        per_class[name]["tp"] += 1
                    else:
                        per_class[name]["fp"] += 1
                for index, label in enumerate(target["labels"].tolist()):
                    if index not in matched and 1 <= label <= len(class_names):
                        per_class[class_names[label - 1]]["fn"] += 1

    metrics, precisions, recalls = {}, [], []
    for name, values in per_class.items():
        precision = values["tp"] / max(1, values["tp"] + values["fp"])
        recall = values["tp"] / max(1, values["tp"] + values["fn"])
        metrics[name] = {**values, "precision": precision, "recall": recall}
        precisions.append(precision)
        recalls.append(recall)
    return {
        "evaluated_images": evaluated_images,
        "macro_precision": sum(precisions) / max(1, len(precisions)),
        "macro_recall": sum(recalls) / max(1, len(recalls)),
        "inference_latency_ms": sum(latencies) / max(1, len(latencies)),
        "per_class": metrics,
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default=os.environ.get("DATASET_DIR", "/data/perception/datasets/curated-v6"))
    parser.add_argument("--split", default=os.environ.get("EVAL_SPLIT", "test"), choices=("validation", "test"))
    parser.add_argument("--model", default=os.environ.get("MODEL_PATH", "/data/perception/models/v7/detector.pt"))
    parser.add_argument("--report", default=os.environ.get("REPORT_PATH", "/data/perception/evaluations/v7-held-out/report.json"))
    parser.add_argument("--model-version", default=os.environ.get("MODEL_VERSION", ""))
    parser.add_argument("--classes", default=os.environ.get("CLASS_NAMES", DEFAULT_CLASSES))
    parser.add_argument("--confidence-threshold", type=float, default=float(os.environ.get("CONFIDENCE_THRESHOLD", "0.35")))
    parser.add_argument("--iou-threshold", type=float, default=float(os.environ.get("IOU_THRESHOLD", "0.5")))
    parser.add_argument("--min-macro-precision", type=float, default=float(os.environ.get("MIN_MACRO_PRECISION", "0.80")))
    parser.add_argument("--min-macro-recall", type=float, default=float(os.environ.get("MIN_MACRO_RECALL", "0.80")))
    parser.add_argument("--min-class-recall", type=float, default=float(os.environ.get("MIN_CLASS_RECALL", "0.60")))
    parser.add_argument("--enforce-thresholds", action="store_true", default=os.environ.get("ENFORCE_THRESHOLDS", "false").lower() == "true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.split == "train":
        raise SystemExit("Refusing to evaluate the training split; use validation or test.")
    class_names = [item.strip() for item in args.classes.split(",") if item.strip()]
    class_to_id = {name: index + 1 for index, name in enumerate(class_names)}
    device = torch.device(os.environ.get("DEVICE", "cuda" if torch.cuda.is_available() else "cpu"))
    model, checkpoint = load_model(args.model, class_names, device)
    dataset = HeldOutDataset(args.dataset, args.split, class_to_id)
    loader = DataLoader(dataset, batch_size=1, shuffle=False, num_workers=0, collate_fn=collate)
    metrics = evaluate(model, loader, class_names, device, args.confidence_threshold, args.iou_threshold)
    model_version = args.model_version or (checkpoint.get("model_version", "unknown") if isinstance(checkpoint, dict) else "unknown")
    minimum_class_recall = min(item["recall"] for item in metrics["per_class"].values())
    gate = {
        "min_macro_precision": args.min_macro_precision,
        "min_macro_recall": args.min_macro_recall,
        "min_class_recall": args.min_class_recall,
        "passed": (
            metrics["macro_precision"] >= args.min_macro_precision
            and metrics["macro_recall"] >= args.min_macro_recall
            and minimum_class_recall >= args.min_class_recall
        ),
        "minimum_observed_class_recall": minimum_class_recall,
    }
    report = {
        "schema_version": "held-out-evaluation-report-1",
        "model_version": model_version,
        "model_path": str(Path(args.model)),
        "dataset": str(Path(args.dataset)),
        "split": args.split,
        "held_out": args.split == "test",
        "device": str(device),
        "cuda_enabled": device.type == "cuda",
        "class_names": class_names,
        "confidence_threshold": args.confidence_threshold,
        "iou_threshold": args.iou_threshold,
        "promotion_gate": gate,
        **metrics,
    }
    report_path = Path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)
    if args.enforce_thresholds and not gate["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
