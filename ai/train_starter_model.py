#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import pathlib
import wave
from dataclasses import dataclass

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
MANIFEST = ROOT / "esc50_subset_manifest.csv"
DATA_DIR = ROOT / "data" / "esc50_subset"
ARTIFACT_DIR = ROOT / "artifacts"

FEATURE_NAMES = [
    "rms",
    "abs_mean",
    "peak",
    "zcr",
    "crest_factor",
    "spec_centroid_norm",
    "spec_rolloff_norm",
    "spec_bandwidth_norm",
    "spec_flatness",
    "low_band_share",
    "high_band_share",
]

FOREST_SEED = 331
HYPERPARAM_GRID = [
    {"n_estimators": 9, "max_depth": 3, "max_features": 3},
    {"n_estimators": 9, "max_depth": 4, "max_features": 3},
    {"n_estimators": 9, "max_depth": 5, "max_features": 4},
    {"n_estimators": 15, "max_depth": 3, "max_features": 3},
    {"n_estimators": 15, "max_depth": 4, "max_features": 3},
    {"n_estimators": 15, "max_depth": 5, "max_features": 4},
    {"n_estimators": 21, "max_depth": 4, "max_features": 3},
    {"n_estimators": 21, "max_depth": 5, "max_features": 4},
]


@dataclass(frozen=True)
class RandomForestParams:
    n_estimators: int
    max_depth: int
    max_features: int
    min_samples_split: int = 2
    min_samples_leaf: int = 1


@dataclass(frozen=True)
class TreeNode:
    feature_index: int
    threshold: float
    left_index: int
    right_index: int
    predicted_class: int


@dataclass
class RandomForestModel:
    params: RandomForestParams
    feature_mean: np.ndarray
    feature_std: np.ndarray
    label_names: list[str]
    trees: list[list[TreeNode]]
    train_accuracy: float
    test_accuracy: float
    confusion_test: np.ndarray
    cv_accuracy: float


def load_wav(path: pathlib.Path) -> tuple[np.ndarray, int]:
    with wave.open(str(path), "rb") as wf:
        n_channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        n_frames = wf.getnframes()
        raw = wf.readframes(n_frames)

    if sample_width != 2:
        raise ValueError(f"unsupported sample width in {path}: {sample_width}")

    data = np.frombuffer(raw, dtype=np.int16).astype(np.float64)
    if n_channels > 1:
        data = data.reshape(-1, n_channels).mean(axis=1)

    data /= 32768.0
    return data, sample_rate


def extract_features(samples: np.ndarray, sample_rate: int) -> np.ndarray:
    del sample_rate

    if samples.size == 0:
        return np.zeros(len(FEATURE_NAMES), dtype=np.float64)

    abs_samples = np.abs(samples)
    rms = math.sqrt(float(np.mean(samples * samples)))
    abs_mean = float(np.mean(abs_samples))
    peak = float(np.max(abs_samples))

    signs = np.signbit(samples)
    zcr = float(np.mean(signs[1:] != signs[:-1])) if samples.size > 1 else 0.0
    crest_factor = peak / (rms + 1e-6)

    window = samples[:128]
    spec_centroid_norm = 0.0
    spec_rolloff_norm = 0.0
    spec_bandwidth_norm = 0.0
    spec_flatness = 0.0
    low_band_share = 0.0
    high_band_share = 0.0

    if window.size >= 4:
        lp = float(window[0])
        prev_x = lp
        prev_hp = 0.0
        low_energy = 0.0
        high_energy = 0.0
        abs_sum = 0.0
        hp_abs_sum = 0.0
        diff_sq_sum = 0.0
        hp_zero_cross = 0.0

        for i, x in enumerate(window):
            x = float(x)
            lp = (0.78 * lp) + (0.22 * x)
            hp = x - lp

            low_energy += lp * lp
            high_energy += hp * hp
            abs_sum += abs(x)
            hp_abs_sum += abs(hp)

            if i > 0:
                diff = x - prev_x
                diff_sq_sum += diff * diff
                if (hp < 0.0 and prev_hp >= 0.0) or (hp >= 0.0 and prev_hp < 0.0):
                    hp_zero_cross += 1.0

            prev_x = x
            prev_hp = hp

        total_energy = low_energy + high_energy + 1e-9
        hp_zcr_norm = hp_zero_cross / float(window.size - 1)

        low_band_share = low_energy / total_energy
        high_band_share = high_energy / total_energy
        spec_rolloff_norm = high_band_share
        spec_centroid_norm = hp_abs_sum / (abs_sum + 1e-6)
        spec_bandwidth_norm = math.sqrt(diff_sq_sum / float(window.size - 1))
        spec_flatness = (0.60 * high_band_share) + (0.40 * hp_zcr_norm)

        spec_centroid_norm = min(spec_centroid_norm, 1.0)
        spec_rolloff_norm = min(spec_rolloff_norm, 1.0)
        spec_bandwidth_norm = min(spec_bandwidth_norm, 1.0)
        spec_flatness = min(spec_flatness, 1.0)

    return np.array(
        [
            rms,
            abs_mean,
            peak,
            zcr,
            crest_factor,
            spec_centroid_norm,
            spec_rolloff_norm,
            spec_bandwidth_norm,
            spec_flatness,
            low_band_share,
            high_band_share,
        ],
        dtype=np.float64,
    )


def gini_from_counts(counts: np.ndarray) -> float:
    total = float(np.sum(counts))
    if total <= 0.0:
        return 0.0
    probs = counts / total
    return 1.0 - float(np.sum(probs * probs))


def majority_class(y: np.ndarray, class_count: int) -> int:
    counts = np.bincount(y, minlength=class_count)
    return int(np.argmax(counts))


def flatten_tree(tree: list[TreeNode]) -> list[dict[str, int | float]]:
    return [
        {
            "feature_index": node.feature_index,
            "threshold": node.threshold,
            "left_index": node.left_index,
            "right_index": node.right_index,
            "predicted_class": node.predicted_class,
        }
        for node in tree
    ]


class RandomForestTrainer:
    def __init__(self, params: RandomForestParams, class_count: int, seed: int) -> None:
        self.params = params
        self.class_count = class_count
        self.seed = seed

    def fit(self, x: np.ndarray, y: np.ndarray) -> list[list[TreeNode]]:
        rng = np.random.default_rng(self.seed)
        trees: list[list[TreeNode]] = []

        for _ in range(self.params.n_estimators):
            bootstrap_idx = rng.integers(0, x.shape[0], size=x.shape[0])
            tree_rng = np.random.default_rng(int(rng.integers(0, 2**31 - 1)))
            nodes: list[TreeNode] = []
            self._build_tree(x[bootstrap_idx], y[bootstrap_idx], 0, tree_rng, nodes)
            trees.append(nodes)

        return trees

    def _build_tree(
        self,
        x: np.ndarray,
        y: np.ndarray,
        depth: int,
        rng: np.random.Generator,
        nodes: list[TreeNode],
    ) -> int:
        predicted_class = majority_class(y, self.class_count)
        node_index = len(nodes)
        nodes.append(TreeNode(-1, 0.0, -1, -1, predicted_class))

        if (
            depth >= self.params.max_depth
            or x.shape[0] < self.params.min_samples_split
            or np.unique(y).size <= 1
        ):
            return node_index

        split = self._best_split(x, y, rng)
        if split is None:
            return node_index

        feature_index, threshold = split
        left_mask = x[:, feature_index] <= threshold
        right_mask = ~left_mask
        if not np.any(left_mask) or not np.any(right_mask):
            return node_index

        left_index = self._build_tree(x[left_mask], y[left_mask], depth + 1, rng, nodes)
        right_index = self._build_tree(x[right_mask], y[right_mask], depth + 1, rng, nodes)
        nodes[node_index] = TreeNode(feature_index, threshold, left_index, right_index, predicted_class)
        return node_index

    def _best_split(self, x: np.ndarray, y: np.ndarray, rng: np.random.Generator) -> tuple[int, float] | None:
        sample_count, feature_count = x.shape
        candidate_count = min(self.params.max_features, feature_count)
        candidate_features = rng.choice(feature_count, size=candidate_count, replace=False)
        parent_counts = np.bincount(y, minlength=self.class_count).astype(np.float64)
        parent_impurity = gini_from_counts(parent_counts)
        best_gain = 0.0
        best_split: tuple[int, float] | None = None

        for feature_index in candidate_features:
            order = np.argsort(x[:, feature_index], kind="mergesort")
            feature_values = x[order, feature_index]
            labels = y[order]

            if feature_values[0] == feature_values[-1]:
                continue

            left_counts = np.zeros(self.class_count, dtype=np.int64)
            right_counts = np.bincount(labels, minlength=self.class_count).astype(np.int64)

            for i in range(sample_count - 1):
                label = labels[i]
                left_counts[label] += 1
                right_counts[label] -= 1

                if feature_values[i] == feature_values[i + 1]:
                    continue

                left_size = i + 1
                right_size = sample_count - left_size
                if left_size < self.params.min_samples_leaf or right_size < self.params.min_samples_leaf:
                    continue

                gain = parent_impurity
                gain -= (left_size / sample_count) * gini_from_counts(left_counts)
                gain -= (right_size / sample_count) * gini_from_counts(right_counts)

                if gain > (best_gain + 1e-12):
                    threshold = float((feature_values[i] + feature_values[i + 1]) * 0.5)
                    best_gain = gain
                    best_split = (int(feature_index), threshold)

        return best_split


def predict_tree(tree: list[TreeNode], features: np.ndarray) -> int:
    index = 0
    while True:
        node = tree[index]
        if node.feature_index < 0:
            return node.predicted_class
        if features[node.feature_index] <= node.threshold:
            index = node.left_index
        else:
            index = node.right_index


def predict_forest(trees: list[list[TreeNode]], x: np.ndarray, class_count: int) -> np.ndarray:
    predictions = np.zeros(x.shape[0], dtype=np.int64)
    votes = np.zeros(class_count, dtype=np.int64)

    for i, row in enumerate(x):
        votes.fill(0)
        for tree in trees:
            votes[predict_tree(tree, row)] += 1
        predictions[i] = int(np.argmax(votes))

    return predictions


def accuracy(pred: np.ndarray, target: np.ndarray) -> float:
    return float(np.mean(pred == target))


def cross_validate_params(x_train: np.ndarray, y_train: np.ndarray, class_count: int) -> tuple[RandomForestParams, float]:
    best_params: RandomForestParams | None = None
    best_accuracy = -1.0
    best_complexity: tuple[int, int, int] | None = None

    for offset, config in enumerate(HYPERPARAM_GRID):
        params = RandomForestParams(**config)
        fold_predictions = np.zeros_like(y_train)

        for holdout_idx in range(y_train.shape[0]):
            train_mask = np.ones(y_train.shape[0], dtype=bool)
            train_mask[holdout_idx] = False

            trainer = RandomForestTrainer(
                params=params,
                class_count=class_count,
                seed=FOREST_SEED + (offset * 1000) + holdout_idx,
            )
            trees = trainer.fit(x_train[train_mask], y_train[train_mask])
            fold_predictions[holdout_idx] = predict_forest(trees, x_train[~train_mask], class_count)[0]

        fold_accuracy = accuracy(fold_predictions, y_train)
        complexity = (params.n_estimators * (params.max_depth + 1), params.n_estimators, params.max_features)

        if (
            fold_accuracy > best_accuracy + 1e-12
            or (
                abs(fold_accuracy - best_accuracy) <= 1e-12
                and (best_complexity is None or complexity < best_complexity)
            )
        ):
            best_params = params
            best_accuracy = fold_accuracy
            best_complexity = complexity

    if best_params is None:
        raise RuntimeError("failed to select random forest hyperparameters")

    return best_params, best_accuracy


def train_model(
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_test: np.ndarray,
    y_test: np.ndarray,
    label_names: list[str],
) -> RandomForestModel:
    mean = x_train.mean(axis=0)
    std = x_train.std(axis=0)
    std[std < 1e-9] = 1.0

    x_train_n = (x_train - mean) / std
    x_test_n = (x_test - mean) / std

    params, cv_accuracy = cross_validate_params(x_train_n, y_train, len(label_names))
    trainer = RandomForestTrainer(params=params, class_count=len(label_names), seed=FOREST_SEED)
    trees = trainer.fit(x_train_n, y_train)

    train_pred = predict_forest(trees, x_train_n, len(label_names))
    test_pred = predict_forest(trees, x_test_n, len(label_names))
    confusion = np.zeros((len(label_names), len(label_names)), dtype=np.int64)

    for expected, predicted in zip(y_test, test_pred):
        confusion[expected, predicted] += 1

    return RandomForestModel(
        params=params,
        feature_mean=mean,
        feature_std=std,
        label_names=label_names,
        trees=trees,
        train_accuracy=accuracy(train_pred, y_train),
        test_accuracy=accuracy(test_pred, y_test),
        confusion_test=confusion,
        cv_accuracy=cv_accuracy,
    )


def build_model_dict(model: RandomForestModel) -> dict:
    return {
        "feature_names": FEATURE_NAMES,
        "feature_mean": model.feature_mean.tolist(),
        "feature_std": model.feature_std.tolist(),
        "label_names": model.label_names,
        "train_accuracy": model.train_accuracy,
        "test_accuracy": model.test_accuracy,
        "cv_accuracy": model.cv_accuracy,
        "confusion_test": model.confusion_test.tolist(),
        "dataset": "ESC-50 subset multiclass",
        "classifier": "RandomForest",
        "params": {
            "n_estimators": model.params.n_estimators,
            "max_depth": model.params.max_depth,
            "max_features": model.params.max_features,
            "min_samples_split": model.params.min_samples_split,
            "min_samples_leaf": model.params.min_samples_leaf,
        },
        "tree_node_counts": [len(tree) for tree in model.trees],
        "trees": [flatten_tree(tree) for tree in model.trees],
    }


def write_c_header(model: dict) -> None:
    header_path = ARTIFACT_DIR / "starter_model.h"
    means = ", ".join(f"{m:.8f}f" for m in model["feature_mean"])
    stds = ", ".join(f"{s:.8f}f" for s in model["feature_std"])
    label_names = ", ".join(f"\"{name}\"" for name in model["label_names"])
    tree_count = len(model["trees"])

    tree_blocks = []
    tree_sizes = []
    tree_refs = []
    for tree_index, tree in enumerate(model["trees"]):
        node_lines = []
        for node in tree:
            node_lines.append(
                "    "
                + "{"
                + f"{node['left_index']}, {node['right_index']}, {node['feature_index']}, "
                + f"{node['predicted_class']}, {node['threshold']:.8f}f"
                + "}"
            )
        tree_blocks.append(
            f"static const starter_model_node_t starter_model_tree_{tree_index}[{len(tree)}] = {{\n"
            + ",\n".join(node_lines)
            + "\n};"
        )
        tree_sizes.append(str(len(tree)))
        tree_refs.append(f"starter_model_tree_{tree_index}")

    text = f"""#ifndef STARTER_MODEL_H
#define STARTER_MODEL_H

#include <stdint.h>

#define STARTER_MODEL_FEATURE_COUNT {len(FEATURE_NAMES)}
#define STARTER_MODEL_CLASS_COUNT {len(model["label_names"])}
#define STARTER_MODEL_TREE_COUNT {tree_count}

typedef struct {{
    int16_t left_index;
    int16_t right_index;
    int8_t feature_index;
    uint8_t predicted_class;
    float threshold;
}} starter_model_node_t;

static const float starter_model_feature_mean[STARTER_MODEL_FEATURE_COUNT] = {{{means}}};
static const float starter_model_feature_std[STARTER_MODEL_FEATURE_COUNT] = {{{stds}}};
static const char *const starter_model_label_names[STARTER_MODEL_CLASS_COUNT] = {{{label_names}}};

{chr(10).join(tree_blocks)}

static const uint16_t starter_model_tree_sizes[STARTER_MODEL_TREE_COUNT] = {{{", ".join(tree_sizes)}}};
static const starter_model_node_t *const starter_model_trees[STARTER_MODEL_TREE_COUNT] = {{{", ".join(tree_refs)}}};

static int starter_model_tree_predict(const starter_model_node_t *tree, uint16_t node_count, const float *x)
{{
    int16_t index = 0;

    while ((index >= 0) && (index < (int16_t)node_count)) {{
        const starter_model_node_t *node = &tree[index];
        if (node->feature_index < 0) {{
            return (int)node->predicted_class;
        }}
        if (x[node->feature_index] <= node->threshold) {{
            index = node->left_index;
        }} else {{
            index = node->right_index;
        }}
    }}

    return 0;
}}

static int starter_model_predict_class(const float *features)
{{
    float x[STARTER_MODEL_FEATURE_COUNT];
    uint8_t votes[STARTER_MODEL_CLASS_COUNT] = {{0}};
    int best_class = 0;

    for (int i = 0; i < STARTER_MODEL_FEATURE_COUNT; ++i) {{
        x[i] = (features[i] - starter_model_feature_mean[i]) / starter_model_feature_std[i];
    }}

    for (int tree_index = 0; tree_index < STARTER_MODEL_TREE_COUNT; ++tree_index) {{
        int predicted_class = starter_model_tree_predict(
            starter_model_trees[tree_index],
            starter_model_tree_sizes[tree_index],
            x
        );
        if ((predicted_class >= 0) && (predicted_class < STARTER_MODEL_CLASS_COUNT)) {{
            votes[predicted_class]++;
        }}
    }}

    for (int class_index = 1; class_index < STARTER_MODEL_CLASS_COUNT; ++class_index) {{
        if (votes[class_index] > votes[best_class]) {{
            best_class = class_index;
        }}
    }}

    return best_class;
}}

static const char *starter_model_class_name(int class_id)
{{
    if ((class_id < 0) || (class_id >= STARTER_MODEL_CLASS_COUNT)) {{
        return "UNK";
    }}
    return starter_model_label_names[class_id];
}}

#endif
"""
    header_path.write_text(text)


def main() -> int:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)

    rows = list(csv.DictReader(MANIFEST.open(newline="")))
    dataset = []

    for row in rows:
        wav_path = DATA_DIR / row["filename"]
        if not wav_path.exists():
            raise FileNotFoundError(f"missing audio file: {wav_path}")
        samples, sample_rate = load_wav(wav_path)
        feats = extract_features(samples, sample_rate)
        dataset.append(
            {
                "filename": row["filename"],
                "category": row["category"],
                "label": int(row["label"]),
                "label_name": row["label_name"],
                "split": row["split"],
                "features": feats,
            }
        )

    labels = sorted({d["label"] for d in dataset})
    label_name_map = {d["label"]: d["label_name"] for d in dataset}
    label_names = [label_name_map[i] for i in labels]

    x_train = np.vstack([d["features"] for d in dataset if d["split"] == "train"])
    y_train = np.array([d["label"] for d in dataset if d["split"] == "train"], dtype=np.int64)
    x_test = np.vstack([d["features"] for d in dataset if d["split"] == "test"])
    y_test = np.array([d["label"] for d in dataset if d["split"] == "test"], dtype=np.int64)

    model = train_model(x_train, y_train, x_test, y_test, label_names)
    model_dict = build_model_dict(model)

    (ARTIFACT_DIR / "starter_model.json").write_text(json.dumps(model_dict, indent=2))
    write_c_header(model_dict)

    with (ARTIFACT_DIR / "feature_dump.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["filename", "category", "label", "label_name", "split", *FEATURE_NAMES])
        for item in dataset:
            writer.writerow(
                [
                    item["filename"],
                    item["category"],
                    item["label"],
                    item["label_name"],
                    item["split"],
                    *item["features"].tolist(),
                ]
            )

    print(f"cv accuracy:    {model.cv_accuracy:.3f}")
    print(f"train accuracy: {model.train_accuracy:.3f}")
    print(f"test accuracy:  {model.test_accuracy:.3f}")
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
