#!/usr/bin/env python3
"""Create a reproducible, non-destructive, episode-aware curation.

Raw flight directories are never changed. Every labeled frame is retained, and
only a deterministic sample of no-label frames is copied into the derived set.
Whole source episodes are assigned to splits so adjacent frames do not leak
between train and validation.
"""

import hashlib
import json
import os
import shutil
from itertools import product
from pathlib import Path


DATA_ROOT = Path(os.environ.get("DATA_ROOT", "/data/perception")).resolve()
SOURCE_NAMES = [item.strip() for item in os.environ.get("SOURCE_DATASETS", "flight-v3").split(",") if item.strip()]
OUTPUT_DIR = Path(os.environ.get("OUTPUT_DIR", str(DATA_ROOT / "datasets" / "curated-v6"))).resolve()
NEGATIVE_RATIO = float(os.environ.get("NEGATIVE_RATIO", "0.35"))
SPLIT_MAP = json.loads(os.environ.get("SPLIT_MAP", "{}"))
CAMERA_OFFSET_Z = float(os.environ.get("CAMERA_OFFSET_Z", "0.36"))
IMAGE_X_AXIS = os.environ.get("IMAGE_X_AXIS", "y")
IMAGE_X_SIGN = float(os.environ.get("IMAGE_X_SIGN", "-1"))
IMAGE_Y_AXIS = os.environ.get("IMAGE_Y_AXIS", "x")
IMAGE_Y_SIGN = float(os.environ.get("IMAGE_Y_SIGN", "-1"))


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def project_box(center, dimensions, pose, width, height, camera):
    drone_x, drone_y, drone_z = (float(value) for value in pose)
    target_x = float(center["x"])
    target_y = float(center["y"])
    target_z = float(center.get("z", 0.0))
    camera_z = drone_z - CAMERA_OFFSET_Z
    points = []
    for offset_x, offset_y, offset_z in product((-0.5, 0.5), repeat=3):
        world_x = target_x + offset_x * float(dimensions["x"])
        world_y = target_y + offset_y * float(dimensions["y"])
        world_z = target_z + offset_z * float(dimensions["z"])
        depth = camera_z - world_z
        if depth <= 0.01:
            continue
        deltas = {"x": world_x - drone_x, "y": world_y - drone_y}
        points.append((
            float(camera["cx"]) + IMAGE_X_SIGN * float(camera["fx"]) * deltas[IMAGE_X_AXIS] / depth,
            float(camera["cy"]) + IMAGE_Y_SIGN * float(camera["fy"]) * deltas[IMAGE_Y_AXIS] / depth,
        ))
    if not points:
        return None
    x_min = max(0.0, min(point[0] for point in points))
    y_min = max(0.0, min(point[1] for point in points))
    x_max = min(float(width), max(point[0] for point in points))
    y_max = min(float(height), max(point[1] for point in points))
    if x_max - x_min < 1.0 or y_max - y_min < 1.0:
        return None
    return {
        "x_min": round(x_min, 3), "y_min": round(y_min, 3),
        "x_max": round(x_max, 3), "y_max": round(y_max, 3),
        "width": round(x_max - x_min, 3), "height": round(y_max - y_min, 3),
        "center_x": round((x_min + x_max) / 2.0, 3),
        "center_y": round((y_min + y_max) / 2.0, 3),
    }


def corrected_label_payload(source, stem):
    label_path = source / "labels" / f"{stem}.json"
    payload = read_json(label_path)
    metadata = read_json(source / "metadata" / f"{stem}.json")
    pose = metadata.get("drone_pose_farm_map")
    camera = metadata.get("camera") or {}
    if not pose or not all(key in camera for key in ("fx", "fy", "cx", "cy")):
        return payload
    annotations = []
    for annotation in payload.get("annotations", []):
        box = project_box(
            annotation.get("center_farm_map", {}),
            annotation.get("dimensions_m", {}),
            pose,
            payload.get("image_width", 320), payload.get("image_height", 240), camera,
        )
        if box is None:
            continue
        corrected = dict(annotation)
        corrected["bbox_pixels"] = box
        corrected["bbox_normalized"] = {
            "center_x": round(box["center_x"] / float(payload.get("image_width", 320)), 6),
            "center_y": round(box["center_y"] / float(payload.get("image_height", 240)), 6),
            "width": round(box["width"] / float(payload.get("image_width", 320)), 6),
            "height": round(box["height"] / float(payload.get("image_height", 240)), 6),
        }
        corrected["label_projection"] = "gazebo_camera_pitch_90_corrected"
        annotations.append(corrected)
    return {**payload, "annotations": annotations, "label_projection": "gazebo_camera_pitch_90_corrected"}


def copy_frame(source, output, stem, split, label_payload):
    names = {
        "images": f"{source.name}__{stem}.png",
        "labels": f"{source.name}__{stem}.json",
        "metadata": f"{source.name}__{stem}.json",
    }
    for directory in ("images", "metadata"):
        source_path = source / directory / f"{stem}.{'png' if directory == 'images' else 'json'}"
        if source_path.is_file():
            target_dir = output / directory / split
            target_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_path, target_dir / names[directory])
    target_dir = output / "labels" / split
    target_dir.mkdir(parents=True, exist_ok=True)
    label_json = target_dir / names["labels"]
    label_json.write_text(json.dumps(label_payload, indent=2) + "\n", encoding="utf-8")
    lines = []
    for annotation in label_payload.get("annotations", []):
        normalized = annotation.get("bbox_normalized") or {}
        lines.append(
            f"{annotation.get('class_id', 0)} {normalized.get('center_x', 0):.6f} "
            f"{normalized.get('center_y', 0):.6f} {normalized.get('width', 0):.6f} "
            f"{normalized.get('height', 0):.6f}"
        )
    (target_dir / f"{source.name}__{stem}.txt").write_text(
        "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8"
    )


def main():
    if OUTPUT_DIR.exists():
        raise RuntimeError(f"Refusing to overwrite existing curated dataset: {OUTPUT_DIR}")
    OUTPUT_DIR.mkdir(parents=True)
    for directory in ("images", "labels", "metadata"):
        (OUTPUT_DIR / directory).mkdir()

    all_frames = []
    classes = {}
    source_stats = {}
    split_counts = {}
    for source_name in SOURCE_NAMES:
        source = DATA_ROOT / "datasets" / source_name
        if not source.is_dir():
            raise RuntimeError(f"Source dataset does not exist: {source}")
        split = SPLIT_MAP.get(source_name, "train")
        if split not in {"train", "validation", "test"}:
            raise RuntimeError(f"Invalid split {split!r} for {source_name}")
        source_stats[source_name] = {"split": split, "images": 0, "labeled": 0, "negative": 0}
        for image in sorted((source / "images").glob("*.png")):
            source_stats[source_name]["images"] += 1
            label_path = source / "labels" / f"{image.stem}.json"
            label_payload = read_json(label_path)
            label_payload = corrected_label_payload(source, image.stem)
            annotations = label_payload.get("annotations", [])
            for annotation in annotations:
                name = annotation.get("class_name", "unknown")
                classes[name] = classes.get(name, 0) + 1
            all_frames.append({
                "source": source,
                "source_name": source_name,
                "split": split,
                "stem": image.stem,
                "annotations": annotations,
                "label_payload": label_payload,
            })
            source_stats[source_name]["labeled" if annotations else "negative"] += 1

    selected = []
    for source_name in SOURCE_NAMES:
        source_frames = [frame for frame in all_frames if frame["source_name"] == source_name]
        labeled = [frame for frame in source_frames if frame["annotations"]]
        negatives = [frame for frame in source_frames if not frame["annotations"]]
        negative_budget = min(len(negatives), max(1, int(len(labeled) * NEGATIVE_RATIO))) if negatives else 0
        selected_negatives = sorted(
            negatives,
            key=lambda frame: hashlib.sha256(
                f"{frame['source_name']}/{frame['stem']}".encode()
            ).hexdigest(),
        )[:negative_budget]
        selected.extend(labeled + selected_negatives)

    selected.sort(key=lambda frame: (frame["split"], frame["source_name"], frame["stem"]))
    for frame in selected:
        copy_frame(frame["source"], OUTPUT_DIR, frame["stem"], frame["split"], frame["label_payload"])
        split = split_counts.setdefault(frame["split"], {"frames": 0, "labeled": 0, "negative": 0})
        split["frames"] += 1
        split["labeled" if frame["annotations"] else "negative"] += 1

    manifest = {
        "schema_version": "curated-dataset-manifest-3",
        "dataset_version": OUTPUT_DIR.name,
        "raw_data_preserved": True,
        "source_datasets": SOURCE_NAMES,
        "source_stats": source_stats,
        "split_map": {name: SPLIT_MAP.get(name, "train") for name in SOURCE_NAMES},
        "split_counts": split_counts,
        "frames_before_curation": len(all_frames),
        "frames_after_curation": len(selected),
        "labeled_frames_after_curation": sum(1 for frame in selected if frame["annotations"]),
        "negative_frames_after_curation": sum(1 for frame in selected if not frame["annotations"]),
        "negative_policy": {
            "ratio_to_labeled_frames_per_episode": NEGATIVE_RATIO,
            "selection": "deterministic_sha256_sample",
        },
        "label_projection": "gazebo_camera_pitch_90_corrected",
        "class_counts": classes,
        "split_status": "provisional_episode_split",
        "split_warning": "A final test split needs another held-out episode with coverage of every class.",
        "layout": {
            "images": "images/{train,validation,test}",
            "labels": "labels/{train,validation,test} (JSON audit plus YOLO txt)",
            "metadata": "metadata/{train,validation,test}",
        },
    }
    (OUTPUT_DIR / "dataset_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2), flush=True)


if __name__ == "__main__":
    main()
