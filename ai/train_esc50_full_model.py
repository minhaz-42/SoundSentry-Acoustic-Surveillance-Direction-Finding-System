#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import pathlib
from dataclasses import dataclass

import numpy as np

import train_starter_model as base

ROOT = pathlib.Path(__file__).resolve().parent
MANIFEST = ROOT / "esc50_2k_manifest.csv"
DATA_DIR = ROOT / "data" / "esc50_2k"
ARTIFACT_DIR = ROOT / "artifacts"

WINDOW_SECONDS = (0.18, 0.35, 0.70, 1.20, 2.00)
FULL_GRID = [
    {"n_estimators": 9, "max_depth": 4, "max_features": 4},
    {"n_estimators": 11, "max_depth": 4, "max_features": 4},
]


@dataclass(frozen=True)
class Example:
    filename: str
    category: str
    fold: int
    label: int
    label_name: str
    window: str
    features: np.ndarray


def loud_region_center(samples: np.ndarray, sample_rate: int) -> int:
    frame = max(256, int(sample_rate * 0.025))
    hop = max(128, frame // 2)

    if samples.size <= frame:
        return samples.size // 2

    best_energy = -1.0
    best_center = samples.size // 2
    for start in range(0, samples.size - frame + 1, hop):
        chunk = samples[start : start + frame]
        energy = float(np.mean(chunk * chunk))
        if energy > best_energy:
            best_energy = energy
            best_center = start + (frame // 2)

    return best_center


def window_slice(samples: np.ndarray, center: int, length: int) -> np.ndarray:
    if length >= samples.size:
        return samples

    start = max(0, center - (length // 2))
    end = start + length
    if end > samples.size:
        end = samples.size
        start = max(0, end - length)

    return samples[start:end]


def extract_examples(row: dict[str, str]) -> list[Example]:
    wav_path = DATA_DIR / row["filename"]
    if not wav_path.exists():
        raise FileNotFoundError(f"missing audio file: {wav_path}")

    samples, sample_rate = base.load_wav(wav_path)
    center = loud_region_center(samples, sample_rate)
    examples: list[Example] = []

    for seconds in WINDOW_SECONDS:
        segment = window_slice(samples, center, max(64, int(sample_rate * seconds)))
        examples.append(
            Example(
                filename=row["filename"],
                category=row["category"],
                fold=int(row["fold"]),
                label=int(row["label"]),
                label_name=row["label_name"],
                window=f"{seconds:.2f}s",
                features=base.extract_features(segment, sample_rate),
            )
        )

    examples.append(
        Example(
            filename=row["filename"],
            category=row["category"],
            fold=int(row["fold"]),
            label=int(row["label"]),
            label_name=row["label_name"],
            window="full",
            features=base.extract_features(samples, sample_rate),
        )
    )
    return examples


def normalize(train_x: np.ndarray, eval_x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    mean = train_x.mean(axis=0)
    std = train_x.std(axis=0)
    std[std < 1e-9] = 1.0
    return (train_x - mean) / std, (eval_x - mean) / std, mean, std


def evaluate_by_fold(
    x: np.ndarray,
    y: np.ndarray,
    folds: np.ndarray,
    class_count: int,
    params: base.RandomForestParams,
    seed_offset: int,
) -> tuple[float, np.ndarray]:
    predictions = np.zeros_like(y)

    for fold in sorted(set(folds.tolist())):
        train_mask = folds != fold
        test_mask = folds == fold
        x_train_n, x_test_n, _, _ = normalize(x[train_mask], x[test_mask])
        trainer = base.RandomForestTrainer(
            params=params,
            class_count=class_count,
            seed=base.FOREST_SEED + seed_offset + fold,
        )
        trees = trainer.fit(x_train_n, y[train_mask])
        predictions[test_mask] = base.predict_forest(trees, x_test_n, class_count)

    return base.accuracy(predictions, y), predictions


def select_params(
    x: np.ndarray,
    y: np.ndarray,
    folds: np.ndarray,
    class_count: int,
) -> tuple[base.RandomForestParams, float, np.ndarray]:
    best_params: base.RandomForestParams | None = None
    best_accuracy = -1.0
    best_predictions = np.zeros_like(y)
    best_complexity: tuple[int, int, int] | None = None

    for index, config in enumerate(FULL_GRID):
        params = base.RandomForestParams(**config)
        fold_accuracy, predictions = evaluate_by_fold(
            x=x,
            y=y,
            folds=folds,
            class_count=class_count,
            params=params,
            seed_offset=index * 1000,
        )
        complexity = (
            params.n_estimators * ((2 ** (params.max_depth + 1)) - 1),
            params.n_estimators,
            params.max_features,
        )
        print(
            "candidate: "
            + f"trees={params.n_estimators}, "
            + f"depth={params.max_depth}, "
            + f"features={params.max_features}, "
            + f"fold_acc={fold_accuracy:.3f}"
        )

        if (
            fold_accuracy > best_accuracy + 1e-12
            or (
                abs(fold_accuracy - best_accuracy) <= 1e-12
                and (best_complexity is None or complexity < best_complexity)
            )
        ):
            best_params = params
            best_accuracy = fold_accuracy
            best_predictions = predictions
            best_complexity = complexity

    if best_params is None:
        raise RuntimeError("failed to select model parameters")

    return best_params, best_accuracy, best_predictions


def load_dataset() -> tuple[list[Example], list[str]]:
    if not MANIFEST.exists():
        raise FileNotFoundError(f"{MANIFEST} is missing; run python3 ai/download_esc50_2k.py first")

    rows = list(csv.DictReader(MANIFEST.open(newline="")))
    missing = [row["filename"] for row in rows if not (DATA_DIR / row["filename"]).exists()]
    if missing:
        shown = ", ".join(missing[:8])
        raise FileNotFoundError(
            f"{len(missing)} audio files are missing in {DATA_DIR}; "
            + f"first missing: {shown}. Run python3 ai/download_esc50_2k.py"
        )

    examples: list[Example] = []
    for row in rows:
        examples.extend(extract_examples(row))

    label_map = {int(row["label"]): row["label_name"] for row in rows}
    label_names = [label_map[index] for index in sorted(label_map)]
    return examples, label_names


def train_model(examples: list[Example], label_names: list[str]) -> base.RandomForestModel:
    x = np.vstack([example.features for example in examples])
    y = np.array([example.label for example in examples], dtype=np.int64)
    folds = np.array([example.fold for example in examples], dtype=np.int64)
    class_count = len(label_names)

    params, fold_accuracy, fold_predictions = select_params(x, y, folds, class_count)
    x_n, _, mean, std = normalize(x, x)
    trainer = base.RandomForestTrainer(params=params, class_count=class_count, seed=base.FOREST_SEED)
    trees = trainer.fit(x_n, y)
    train_predictions = base.predict_forest(trees, x_n, class_count)

    confusion = np.zeros((class_count, class_count), dtype=np.int64)
    for expected, predicted in zip(y, fold_predictions):
        confusion[expected, predicted] += 1

    return base.RandomForestModel(
        params=params,
        feature_mean=mean,
        feature_std=std,
        label_names=label_names,
        trees=trees,
        train_accuracy=base.accuracy(train_predictions, y),
        test_accuracy=fold_accuracy,
        confusion_test=confusion,
        cv_accuracy=fold_accuracy,
    )


def write_feature_dump(examples: list[Example]) -> None:
    with (ARTIFACT_DIR / "feature_dump.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["filename", "category", "fold", "label", "label_name", "window", *base.FEATURE_NAMES])
        for example in examples:
            writer.writerow(
                [
                    example.filename,
                    example.category,
                    example.fold,
                    example.label,
                    example.label_name,
                    example.window,
                    *example.features.tolist(),
                ]
            )


def main() -> int:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    examples, label_names = load_dataset()
    model = train_model(examples, label_names)
    model_dict = base.build_model_dict(model)
    model_dict["dataset"] = "ESC-50 full 2000-clip / 50-class model with peak-window augmentation"
    model_dict["example_count"] = len(examples)
    model_dict["source_clip_count"] = len({example.filename for example in examples})
    model_dict["fold_accuracy"] = model.cv_accuracy

    (ARTIFACT_DIR / "starter_model.json").write_text(json.dumps(model_dict, indent=2))
    base.write_c_header(model_dict)
    write_feature_dump(examples)

    print(f"fold accuracy:  {model.cv_accuracy:.3f}")
    print(f"train accuracy: {model.train_accuracy:.3f}")
    print(f"examples:       {len(examples)}")
    print(f"source clips:   {model_dict['source_clip_count']}")
    print("label names:    " + ", ".join(model.label_names))
    print(
        "forest params:  "
        + f"trees={model.params.n_estimators}, "
        + f"depth={model.params.max_depth}, "
        + f"features/split={model.params.max_features}"
    )
    print(f"model json:     {ARTIFACT_DIR / 'starter_model.json'}")
    print(f"c header:       {ARTIFACT_DIR / 'starter_model.h'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
