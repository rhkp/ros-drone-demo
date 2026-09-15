#!/usr/bin/env python3
"""Export the versioned torchvision detector checkpoint to ONNX.

This is an artifact conversion step only. It does not read simulator truth and
does not change any dataset labels.
"""

import json
import os
from pathlib import Path

import torch
from torch import nn
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn


MODEL_DIR = Path(os.environ.get("MODEL_DIR", "/data/perception/models/v6"))
CHECKPOINT = Path(os.environ.get("CHECKPOINT", str(MODEL_DIR / "detector.pt")))
OUTPUT = Path(os.environ.get("OUTPUT", str(MODEL_DIR / "detector.onnx")))
WIDTH = int(os.environ.get("IMAGE_WIDTH", "320"))
HEIGHT = int(os.environ.get("IMAGE_HEIGHT", "240"))


class DetectionWrapper(nn.Module):
    """Adapt torchvision's list-of-dicts output to ONNX tensor outputs."""

    def __init__(self, detector):
        super().__init__()
        self.detector = detector

    def forward(self, image):
        result = self.detector([image])[0]
        return result["boxes"], result["labels"], result["scores"]


def main():
    checkpoint = torch.load(CHECKPOINT, map_location="cpu", weights_only=False)
    class_names = list(checkpoint["class_names"])
    detector = fasterrcnn_mobilenet_v3_large_320_fpn(
        weights=None, weights_backbone=None, num_classes=len(class_names) + 1,
    )
    detector.load_state_dict(checkpoint["state_dict"])
    detector.eval()
    wrapper = DetectionWrapper(detector).eval()
    dummy = torch.zeros((3, HEIGHT, WIDTH), dtype=torch.float32)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        wrapper,
        (dummy,),
        str(OUTPUT),
        input_names=["images"],
        output_names=["boxes", "labels", "scores"],
        opset_version=int(os.environ.get("ONNX_OPSET", "17")),
        dynamic_axes={
            "boxes": {0: "detections"},
            "labels": {0: "detections"},
            "scores": {0: "detections"},
        },
        do_constant_folding=True,
        dynamo=False,
    )
    manifest = {
        "schema_version": "model-manifest-2",
        "model_version": checkpoint["model_version"],
        "format": "onnx",
        "artifact": str(OUTPUT),
        "class_names": class_names,
        "input": {"name": "images", "width": WIDTH, "height": HEIGHT, "channels": 3},
        "outputs": {"boxes": "xyxy_pixels", "labels": "one_based_class_ids", "scores": "confidence"},
        "source_checkpoint": str(CHECKPOINT),
    }
    (MODEL_DIR / "onnx_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"event": "onnx_export_complete", **manifest}, indent=2), flush=True)


if __name__ == "__main__":
    main()
