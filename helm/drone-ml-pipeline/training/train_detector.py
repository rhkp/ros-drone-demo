#!/usr/bin/env python3
"""Train a small, free torchvision detector from a curated dataset."""

import json
import os
import random
import time
from pathlib import Path

import torch
from PIL import Image, ImageOps
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn


DATASET_DIR = Path(os.environ.get("DATASET_DIR", "/data/perception/datasets/curated-v2"))
MODEL_DIR = Path(os.environ.get("MODEL_DIR", "/data/perception/models/v1"))
EVAL_DIR = Path(os.environ.get("EVAL_DIR", "/data/perception/evaluations/v1"))
EPOCHS = int(os.environ.get("EPOCHS", "12"))
BATCH_SIZE = int(os.environ.get("BATCH_SIZE", "4"))
SEED = int(os.environ.get("SEED", "20260914"))
CLASS_NAMES = [item.strip() for item in os.environ.get(
    "CLASS_NAMES", "well,storage_silo,cattle_shed,produce_barn,crop_field,greenhouse,water_tank"
).split(",") if item.strip()]
CLASS_TO_ID = {name: index + 1 for index, name in enumerate(CLASS_NAMES)}


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


class FarmDataset(Dataset):
    def __init__(self, root, split, training=False):
        self.image_dir = Path(root) / "images" / split
        self.label_dir = Path(root) / "labels" / split
        self.training = training
        self.images = sorted(self.image_dir.glob("*.png"))
        if not self.images:
            raise RuntimeError(f"No PNG images found in {self.image_dir}")
        self.image_classes = []
        frequencies = {name: 0 for name in CLASS_NAMES}
        for image_path in self.images:
            payload = read_json(self.label_dir / f"{image_path.stem}.json")
            names = {annotation.get("class_name") for annotation in payload.get("annotations", [])}
            names.discard(None)
            self.image_classes.append(names)
            for name in names:
                if name in frequencies:
                    frequencies[name] += 1
        self.class_frequencies = frequencies

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        image_path = self.images[index]
        image = Image.open(image_path).convert("RGB")
        width, height = image.size
        payload = read_json(self.label_dir / f"{image_path.stem}.json")
        boxes, labels = [], []
        for annotation in payload.get("annotations", []):
            class_id = CLASS_TO_ID.get(annotation.get("class_name"))
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
        if self.training and random.random() < 0.5:
            image = ImageOps.mirror(image)
            boxes = [[width - x2, y1, width - x1, y2] for x1, y1, x2, y2 in boxes]
        if self.training and random.random() < 0.5:
            image = ImageOps.flip(image)
            boxes = [[x1, height - y2, x2, height - y1] for x1, y1, x2, y2 in boxes]
        # Import numpy lazily; the PyTorch image includes it and this keeps startup simple.
        pixels = torch.from_numpy(__import__("numpy").array(image)).permute(2, 0, 1).float() / 255.0
        target = {
            "boxes": torch.tensor(boxes, dtype=torch.float32).reshape(-1, 4),
            "labels": torch.tensor(labels, dtype=torch.int64),
            "image_id": torch.tensor([index]),
            "area": torch.tensor([(b[2] - b[0]) * (b[3] - b[1]) for b in boxes], dtype=torch.float32),
            "iscrowd": torch.zeros(len(boxes), dtype=torch.int64),
            "path": str(image_path),
        }
        return pixels, target

    def sampling_weights(self):
        nonzero = [value for value in self.class_frequencies.values() if value]
        most_common = max(nonzero, default=1)
        weights = []
        for names in self.image_classes:
            if not names:
                weights.append(1.0)
                continue
            # Oversample minority classes, but use sqrt so tiny classes do not
            # completely crowd out the common classes.
            weights.append(max(1.0, max((most_common / self.class_frequencies[name]) ** 0.5 for name in names)))
        return weights


def collate(batch):
    return tuple(zip(*batch))


def iou(left, right):
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union else 0.0


def evaluate(model, loader, device):
    model.eval()
    per_class = {name: {"tp": 0, "fp": 0, "fn": 0} for name in CLASS_NAMES}
    latencies = []
    with torch.no_grad():
        for images, targets in loader:
            batch = [image.to(device) for image in images]
            started = time.perf_counter()
            predictions = model(batch)
            latencies.append((time.perf_counter() - started) * 1000.0 / max(1, len(batch)))
            for prediction, target in zip(predictions, targets):
                matched = set()
                for box, label, score in zip(
                    prediction["boxes"].cpu().tolist(),
                    prediction["labels"].cpu().tolist(),
                    prediction["scores"].cpu().tolist(),
                ):
                    if score < 0.35 or not 1 <= label <= len(CLASS_NAMES):
                        continue
                    name = CLASS_NAMES[label - 1]
                    candidates = [(idx, iou(box, truth_box)) for idx, (truth_box, truth_label) in enumerate(
                        zip(target["boxes"].tolist(), target["labels"].tolist())
                    ) if truth_label == label and idx not in matched]
                    best = max(candidates, key=lambda item: item[1], default=(-1, 0.0))
                    if best[1] >= 0.5:
                        matched.add(best[0])
                        per_class[name]["tp"] += 1
                    else:
                        per_class[name]["fp"] += 1
                for idx, label in enumerate(target["labels"].tolist()):
                    if idx not in matched and 1 <= label <= len(CLASS_NAMES):
                        per_class[CLASS_NAMES[label - 1]]["fn"] += 1
    metrics, precisions, recalls = {}, [], []
    for name, values in per_class.items():
        precision = values["tp"] / max(1, values["tp"] + values["fp"])
        recall = values["tp"] / max(1, values["tp"] + values["fn"])
        metrics[name] = {**values, "precision": precision, "recall": recall}
        precisions.append(precision)
        recalls.append(recall)
    return {
        "macro_precision": sum(precisions) / len(precisions),
        "macro_recall": sum(recalls) / len(recalls),
        "inference_latency_ms": sum(latencies) / max(1, len(latencies)),
        "per_class": metrics,
        "validation_images": len(loader.dataset),
    }


def main():
    random.seed(SEED)
    torch.manual_seed(SEED)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(json.dumps({"event": "training_start", "device": str(device), "cuda": torch.cuda.is_available(), "dataset": str(DATASET_DIR)}), flush=True)
    train = FarmDataset(DATASET_DIR, "train", training=True)
    validation = FarmDataset(DATASET_DIR, "validation")
    sampler = WeightedRandomSampler(train.sampling_weights(), num_samples=len(train), replacement=True)
    train_loader = DataLoader(train, batch_size=BATCH_SIZE, sampler=sampler, num_workers=0, collate_fn=collate)
    validation_loader = DataLoader(validation, batch_size=1, shuffle=False, num_workers=0, collate_fn=collate)
    model = fasterrcnn_mobilenet_v3_large_320_fpn(weights=None, weights_backbone=None, num_classes=len(CLASS_NAMES) + 1).to(device)
    optimizer = torch.optim.SGD(
        [parameter for parameter in model.parameters() if parameter.requires_grad],
        lr=0.005, momentum=0.9, weight_decay=0.0005,
    )
    history = []
    for epoch in range(EPOCHS):
        model.train()
        losses = []
        for images, targets in train_loader:
            batch = [image.to(device) for image in images]
            gpu_targets = [{key: value.to(device) for key, value in target.items() if isinstance(value, torch.Tensor)} for target in targets]
            loss = sum(model(batch, gpu_targets).values())
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(float(loss.detach().cpu()))
        result = {"epoch": epoch + 1, "mean_loss": sum(losses) / max(1, len(losses))}
        history.append(result)
        print(json.dumps(result), flush=True)

    MODEL_DIR.mkdir(parents=True, exist_ok=False)
    EVAL_DIR.mkdir(parents=True, exist_ok=False)
    model_version = MODEL_DIR.name
    checkpoint_path = MODEL_DIR / "detector.pt"
    torch.save({"model_version": model_version, "class_names": CLASS_NAMES, "state_dict": model.state_dict(), "training": {"epochs": EPOCHS, "batch_size": BATCH_SIZE, "seed": SEED}}, checkpoint_path)
    metrics = evaluate(model, validation_loader, device)
    report = {"schema_version": "evaluation-report-1", "model_version": model_version, "dataset": str(DATASET_DIR), "device": str(device), "class_names": CLASS_NAMES, "training_history": history, **metrics}
    (MODEL_DIR / "model_manifest.json").write_text(json.dumps({"schema_version": "model-manifest-1", "model_version": model_version, "format": "pytorch-state-dict", "artifact": str(checkpoint_path), "class_names": CLASS_NAMES, "dataset": str(DATASET_DIR), "device": str(device)}, indent=2) + "\n", encoding="utf-8")
    (EVAL_DIR / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"event": "training_complete", "model": str(checkpoint_path), "report": str(EVAL_DIR / "report.json"), **metrics}, indent=2), flush=True)


if __name__ == "__main__":
    main()
