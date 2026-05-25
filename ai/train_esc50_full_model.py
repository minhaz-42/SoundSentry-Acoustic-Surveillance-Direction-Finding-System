#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import pathlib
import wave
from dataclasses import dataclass
from typing import Any

import joblib
import numpy as np
from scipy import fftpack, signal
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.metrics import accuracy_score

ROOT = pathlib.Path(__file__).resolve().parent
MANIFEST = ROOT / "esc50_2k_manifest.csv"
DATA_DIR = ROOT / "data" / "esc50_2k"
ARTIFACT_DIR = ROOT / "artifacts"

FEATURE_CACHE = ARTIFACT_DIR / "esc50_2k_logmel_features.npz"
MODEL_PATH = ARTIFACT_DIR / "esc50_full_model.joblib"
SUMMARY_PATH = ARTIFACT_DIR / "starter_model.json"
FEATURE_DUMP_PATH = ARTIFACT_DIR / "feature_dump.csv"

FEATURE_VERSION = 2
TARGET_ACCURACY = 0.55
TARGET_SAMPLE_RATE = 16000
N_FFT = 512
HOP_LENGTH = 256
N_MELS = 80
N_MFCC = 24
TEMPORAL_PARTS = 5

EXTRA_TREES_PARAMS: dict[str, Any] = {
    "n_estimators": 1200,
    "max_features": 0.25,
    "criterion": "gini",
    "class_weight": "balanced",
    "random_state": 331,
    "n_jobs": -1,
}

STAT_NAMES = ["mean", "std", "min", "max", "p10", "p50", "p90"]
LOW_LEVEL_NAMES = ["rms", "zcr", "centroid", "low_share", "mid_share", "high_share"]


@dataclass(frozen=True)
class Dataset:
    x: np.ndarray
    y: np.ndarray
    folds: np.ndarray
    filenames: list[str]
    categories: list[str]
    label_names: list[str]
    feature_names: list[str]


def hz_to_mel(hz: float | np.ndarray) -> float | np.ndarray:
    return 2595.0 * np.log10(1.0 + (np.asarray(hz) / 700.0))


def mel_to_hz(mel: float | np.ndarray) -> float | np.ndarray:
    return 700.0 * ((10.0 ** (np.asarray(mel) / 2595.0)) - 1.0)


def build_mel_filterbank() -> np.ndarray:
    mel_edges = np.linspace(hz_to_mel(50.0), hz_to_mel(TARGET_SAMPLE_RATE / 2.0), N_MELS + 2)
    hz_edges = mel_to_hz(mel_edges)
    bins = np.floor(((N_FFT + 1) * hz_edges) / TARGET_SAMPLE_RATE).astype(int)
    weights = np.zeros((N_MELS, (N_FFT // 2) + 1), dtype=np.float32)

    for band in range(1, N_MELS + 1):
        left = int(bins[band - 1])
        center = int(bins[band])
        right = int(bins[band + 1])

        if center <= left:
            center = left + 1
        if right <= center:
            right = center + 1

        for index in range(left, min(center, weights.shape[1])):
            weights[band - 1, index] = (index - left) / float(center - left)
        for index in range(center, min(right, weights.shape[1])):
            weights[band - 1, index] = (right - index) / float(right - center)

    enorm = 2.0 / (hz_edges[2 : N_MELS + 2] - hz_edges[:N_MELS])
    weights *= enorm[:, np.newaxis]
    return weights


MEL_FILTERBANK = build_mel_filterbank()


def load_wav(path: pathlib.Path) -> np.ndarray:
    with wave.open(str(path), "rb") as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        frame_count = wf.getnframes()
        raw = wf.readframes(frame_count)

    if sample_width != 2:
        raise ValueError(f"unsupported sample width in {path}: {sample_width}")

    samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
    if channels > 1:
        samples = samples.reshape(-1, channels).mean(axis=1)
    samples /= 32768.0

    if sample_rate != TARGET_SAMPLE_RATE:
        gcd = math.gcd(sample_rate, TARGET_SAMPLE_RATE)
        samples = signal.resample_poly(
            samples,
            TARGET_SAMPLE_RATE // gcd,
            sample_rate // gcd,
        ).astype(np.float32)

    return samples


def frame_signal(samples: np.ndarray) -> np.ndarray:
    if samples.size < N_FFT:
        samples = np.pad(samples, (0, N_FFT - samples.size))

    frame_count = 1 + max(0, (samples.size - N_FFT) // HOP_LENGTH)
    shape = (frame_count, N_FFT)
    strides = (samples.strides[0] * HOP_LENGTH, samples.strides[0])
    frames = np.lib.stride_tricks.as_strided(samples, shape=shape, strides=strides).copy()
    frames *= np.hanning(N_FFT).astype(np.float32)
    return frames


def stat_features(matrix: np.ndarray) -> np.ndarray:
    return np.concatenate(
        [
            matrix.mean(axis=1),
            matrix.std(axis=1),
            matrix.min(axis=1),
            matrix.max(axis=1),
            np.percentile(matrix, 10, axis=1),
            np.percentile(matrix, 50, axis=1),
            np.percentile(matrix, 90, axis=1),
        ]
    )


def temporal_means(matrix: np.ndarray) -> np.ndarray:
    chunks = np.array_split(matrix, TEMPORAL_PARTS, axis=1)
    values = []
    for chunk in chunks:
        if chunk.size:
            values.append(chunk.mean(axis=1))
        else:
            values.append(np.zeros(matrix.shape[0], dtype=np.float32))
    return np.concatenate(values)


def extract_audio_features(path: pathlib.Path) -> np.ndarray:
    samples = load_wav(path)
    peak = float(np.max(np.abs(samples))) if samples.size else 0.0
    if peak > 1e-4:
        samples = samples / peak

    frames = frame_signal(samples)
    spectrum = np.abs(np.fft.rfft(frames, n=N_FFT, axis=1)) ** 2
    mel_power = np.maximum(MEL_FILTERBANK @ spectrum.T, 1e-10)
    log_mel = np.log(mel_power).astype(np.float32)
    mfcc = fftpack.dct(log_mel, axis=0, type=2, norm="ortho")[:N_MFCC]
    delta = np.diff(mfcc, axis=1, prepend=mfcc[:, :1])

    rms = np.sqrt(np.mean(frames * frames, axis=1, keepdims=True)).T
    zcr = np.mean(
        np.signbit(frames[:, 1:]) != np.signbit(frames[:, :-1]),
        axis=1,
        keepdims=True,
    ).T

    freq_bins = np.arange(spectrum.shape[1], dtype=np.float32)[:, np.newaxis]
    spectrum_t = spectrum.T
    centroid = (
        ((freq_bins * spectrum_t).sum(axis=0) / (spectrum_t.sum(axis=0) + 1e-10))[np.newaxis, :]
        / float(N_FFT // 2)
    )
    mel_total = mel_power.sum(axis=0, keepdims=True) + 1e-10
    low_share = mel_power[:20].sum(axis=0, keepdims=True) / mel_total
    mid_share = mel_power[20:50].sum(axis=0, keepdims=True) / mel_total
    high_share = mel_power[50:].sum(axis=0, keepdims=True) / mel_total

    return np.concatenate(
        [
            stat_features(log_mel),
            temporal_means(log_mel),
            stat_features(mfcc),
            stat_features(delta),
            stat_features(np.vstack([rms, zcr, centroid, low_share, mid_share, high_share])),
        ]
    ).astype(np.float32)


def build_feature_names() -> list[str]:
    names: list[str] = []

    for stat in STAT_NAMES:
        names.extend(f"logmel_{band:02d}_{stat}" for band in range(N_MELS))

    for part in range(TEMPORAL_PARTS):
        names.extend(f"logmel_{band:02d}_part{part}_mean" for band in range(N_MELS))

    for stat in STAT_NAMES:
        names.extend(f"mfcc_{index:02d}_{stat}" for index in range(N_MFCC))

    for stat in STAT_NAMES:
        names.extend(f"delta_mfcc_{index:02d}_{stat}" for index in range(N_MFCC))

    for stat in STAT_NAMES:
        names.extend(f"{name}_{stat}" for name in LOW_LEVEL_NAMES)

    return names


def read_manifest() -> list[dict[str, str]]:
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
    return rows


def cache_matches(rows: list[dict[str, str]], cache: np.lib.npyio.NpzFile) -> bool:
    if "feature_version" not in cache.files:
        return False
    if int(cache["feature_version"]) != FEATURE_VERSION:
        return False
    filenames = cache["filenames"].astype(str).tolist()
    return filenames == [row["filename"] for row in rows]


def load_dataset() -> Dataset:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    rows = read_manifest()
    feature_names = build_feature_names()

    if FEATURE_CACHE.exists():
        cache = np.load(FEATURE_CACHE, allow_pickle=True)
        if cache_matches(rows, cache):
            print(f"loaded feature cache: {FEATURE_CACHE}")
            x = cache["x"]
            y = cache["y"]
            folds = cache["folds"]
            filenames = cache["filenames"].astype(str).tolist()
            categories = cache["categories"].astype(str).tolist()
            label_names = cache["label_names"].astype(str).tolist()
            return Dataset(x, y, folds, filenames, categories, label_names, feature_names)

    features = []
    for index, row in enumerate(rows, start=1):
        features.append(extract_audio_features(DATA_DIR / row["filename"]))
        if (index % 200) == 0 or index == len(rows):
            print(f"features: {index}/{len(rows)}")

    x = np.vstack(features)
    y = np.array([int(row["label"]) for row in rows], dtype=np.int64)
    folds = np.array([int(row["fold"]) for row in rows], dtype=np.int64)
    filenames = [row["filename"] for row in rows]
    categories = [row["category"] for row in rows]
    label_map = {int(row["label"]): row["label_name"] for row in rows}
    label_names = [label_map[index] for index in sorted(label_map)]

    np.savez_compressed(
        FEATURE_CACHE,
        feature_version=np.array(FEATURE_VERSION),
        x=x,
        y=y,
        folds=folds,
        filenames=np.array(filenames),
        categories=np.array(categories),
        label_names=np.array(label_names),
    )
    print(f"saved feature cache: {FEATURE_CACHE}")
    return Dataset(x, y, folds, filenames, categories, label_names, feature_names)


def make_model(random_state: int = 331) -> ExtraTreesClassifier:
    params = dict(EXTRA_TREES_PARAMS)
    params["random_state"] = random_state
    return ExtraTreesClassifier(**params)


def evaluate_by_fold(dataset: Dataset) -> tuple[float, list[float], np.ndarray, np.ndarray]:
    predictions = np.zeros_like(dataset.y)
    fold_accuracies: list[float] = []

    for fold in sorted(set(dataset.folds.tolist())):
        train_mask = dataset.folds != fold
        test_mask = dataset.folds == fold
        model = make_model(random_state=331 + fold)
        model.fit(dataset.x[train_mask], dataset.y[train_mask])
        fold_predictions = model.predict(dataset.x[test_mask])
        predictions[test_mask] = fold_predictions
        fold_accuracy = accuracy_score(dataset.y[test_mask], fold_predictions)
        fold_accuracies.append(float(fold_accuracy))
        print(f"fold {fold}: {fold_accuracy:.3f}")

    class_count = len(dataset.label_names)
    confusion = np.zeros((class_count, class_count), dtype=np.int64)
    for expected, predicted in zip(dataset.y, predictions):
        confusion[int(expected), int(predicted)] += 1

    return float(accuracy_score(dataset.y, predictions)), fold_accuracies, predictions, confusion


def train_final_model(dataset: Dataset) -> tuple[ExtraTreesClassifier, float]:
    model = make_model()
    model.fit(dataset.x, dataset.y)
    train_predictions = model.predict(dataset.x)
    return model, float(accuracy_score(dataset.y, train_predictions))


def write_feature_dump(dataset: Dataset) -> None:
    with FEATURE_DUMP_PATH.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["filename", "category", "fold", "label", "label_name", *dataset.feature_names])
        for filename, category, fold, label, row in zip(
            dataset.filenames,
            dataset.categories,
            dataset.folds,
            dataset.y,
            dataset.x,
        ):
            writer.writerow(
                [
                    filename,
                    category,
                    int(fold),
                    int(label),
                    dataset.label_names[int(label)],
                    *row.tolist(),
                ]
            )


def write_summary(
    dataset: Dataset,
    fold_accuracy: float,
    fold_accuracies: list[float],
    train_accuracy: float,
    confusion: np.ndarray,
) -> None:
    summary = {
        "dataset": "ESC-50 full 2000-clip / 50-class log-mel model",
        "classifier": "ExtraTreesClassifier",
        "source_clip_count": len(dataset.filenames),
        "example_count": len(dataset.filenames),
        "class_count": len(dataset.label_names),
        "label_names": dataset.label_names,
        "feature_version": FEATURE_VERSION,
        "feature_count": len(dataset.feature_names),
        "feature_names": dataset.feature_names,
        "audio_feature_config": {
            "target_sample_rate": TARGET_SAMPLE_RATE,
            "n_fft": N_FFT,
            "hop_length": HOP_LENGTH,
            "n_mels": N_MELS,
            "n_mfcc": N_MFCC,
            "temporal_parts": TEMPORAL_PARTS,
            "peak_normalized": True,
        },
        "params": EXTRA_TREES_PARAMS,
        "fold_accuracy": fold_accuracy,
        "cv_accuracy": fold_accuracy,
        "fold_accuracies": fold_accuracies,
        "train_accuracy": train_accuracy,
        "target_accuracy": TARGET_ACCURACY,
        "target_met": fold_accuracy >= TARGET_ACCURACY,
        "confusion_test": confusion.tolist(),
        "model_artifact": str(MODEL_PATH),
        "feature_cache": str(FEATURE_CACHE),
        "firmware_compatible": False,
        "firmware_note": (
            "This 55%+ model uses log-mel/MFCC desktop features and is stored as a "
            "joblib artifact. The STM32 firmware header is not regenerated by this script "
            "because the firmware currently computes only the compact 11-feature vector."
        ),
    }
    SUMMARY_PATH.write_text(json.dumps(summary, indent=2))


def main() -> int:
    dataset = load_dataset()
    print(f"clips:          {len(dataset.filenames)}")
    print(f"classes:        {len(dataset.label_names)}")
    print(f"features/clip:  {dataset.x.shape[1]}")

    fold_accuracy, fold_accuracies, _, confusion = evaluate_by_fold(dataset)
    model, train_accuracy = train_final_model(dataset)
    joblib.dump(
        {
            "model": model,
            "label_names": dataset.label_names,
            "feature_names": dataset.feature_names,
            "feature_config": {
                "feature_version": FEATURE_VERSION,
                "target_sample_rate": TARGET_SAMPLE_RATE,
                "n_fft": N_FFT,
                "hop_length": HOP_LENGTH,
                "n_mels": N_MELS,
                "n_mfcc": N_MFCC,
                "temporal_parts": TEMPORAL_PARTS,
                "peak_normalized": True,
            },
        },
        MODEL_PATH,
        compress=3,
    )
    write_summary(dataset, fold_accuracy, fold_accuracies, train_accuracy, confusion)
    write_feature_dump(dataset)

    print(f"fold accuracy:  {fold_accuracy:.3f}")
    print(f"train accuracy: {train_accuracy:.3f}")
    print(f"target met:     {fold_accuracy >= TARGET_ACCURACY}")
    print(f"model joblib:   {MODEL_PATH}")
    print(f"model summary:  {SUMMARY_PATH}")
    print(f"feature dump:   {FEATURE_DUMP_PATH}")
    return 0 if fold_accuracy >= TARGET_ACCURACY else 2


if __name__ == "__main__":
    raise SystemExit(main())
