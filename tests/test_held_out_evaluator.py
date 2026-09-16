import importlib.util
import json
from pathlib import Path

from PIL import Image


MODULE_PATH = Path(__file__).parents[1] / "helm" / "drone-ml-pipeline" / "verification" / "evaluate_detector.py"
SPEC = importlib.util.spec_from_file_location("evaluate_detector", MODULE_PATH)
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


def test_iou_matches_expected_overlap():
    assert EVALUATOR.iou([0, 0, 10, 10], [5, 5, 15, 15]) == 25 / 175


def test_held_out_dataset_reads_json_boxes(tmp_path):
    image_dir = tmp_path / "images" / "test"
    label_dir = tmp_path / "labels" / "test"
    image_dir.mkdir(parents=True)
    label_dir.mkdir(parents=True)
    Image.new("RGB", (32, 24), (0, 0, 0)).save(image_dir / "frame-000001.png")
    (label_dir / "frame-000001.json").write_text(json.dumps({
        "annotations": [{
            "class_name": "well",
            "bbox_pixels": {"x_min": 2, "y_min": 3, "x_max": 12, "y_max": 14},
        }],
    }), encoding="utf-8")

    dataset = EVALUATOR.HeldOutDataset(tmp_path, "test", {"well": 1})
    image, target = dataset[0]

    assert tuple(image.shape) == (3, 24, 32)
    assert target["boxes"].tolist() == [[2.0, 3.0, 12.0, 14.0]]
    assert target["labels"].tolist() == [1]
