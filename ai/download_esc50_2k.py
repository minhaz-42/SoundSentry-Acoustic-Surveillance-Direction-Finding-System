#!/usr/bin/env python3
from __future__ import annotations

import csv
import pathlib
import sys
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed

ROOT = pathlib.Path(__file__).resolve().parent
BASE_URL = "https://raw.githubusercontent.com/karolpiczak/ESC-50/master"
META_URL = f"{BASE_URL}/meta/esc50.csv"
AUDIO_URL = f"{BASE_URL}/audio"
MANIFEST = ROOT / "esc50_2k_manifest.csv"
OUT_DIR = ROOT / "data" / "esc50_2k"
META_DIR = ROOT / "data" / "esc50_meta"
META_FILE = META_DIR / "esc50.csv"
DEFAULT_WORKERS = 8

TARGETS = {
    "clapping": (0, "CLAP"),
    "door_wood_knock": (1, "KNOCK"),
    "glass_breaking": (2, "GLASS"),
    "coughing": (3, "COUGH"),
    "laughing": (4, "LAUGH"),
    "clock_tick": (5, "TICK"),
}

SHORT_NAMES = {
    "airplane": "AIRPLANE",
    "breathing": "BREATH",
    "brushing_teeth": "BRUSH_TEETH",
    "can_opening": "CAN_OPEN",
    "car_horn": "CAR_HORN",
    "cat": "CAT",
    "chainsaw": "CHAINSAW",
    "chirping_birds": "BIRDS",
    "church_bells": "BELLS",
    "clock_alarm": "ALARM",
    "cow": "COW",
    "crackling_fire": "FIRE",
    "crickets": "CRICKETS",
    "crow": "CROW",
    "crying_baby": "BABY_CRY",
    "dog": "DOG",
    "door_wood_creaks": "DOOR_CREAK",
    "drinking_sipping": "DRINKING",
    "engine": "ENGINE",
    "fireworks": "FIREWORKS",
    "footsteps": "FOOTSTEPS",
    "frog": "FROG",
    "hand_saw": "HAND_SAW",
    "helicopter": "HELICOPTER",
    "hen": "HEN",
    "insects": "INSECTS",
    "keyboard_typing": "KEYBOARD",
    "mouse_click": "MOUSECLICK",
    "pig": "PIG",
    "pouring_water": "WATER_POUR",
    "rain": "RAIN",
    "rooster": "ROOSTER",
    "sea_waves": "SEA_WAVES",
    "sheep": "SHEEP",
    "siren": "SIREN",
    "sneezing": "SNEEZE",
    "snoring": "SNORE",
    "thunderstorm": "THUNDER",
    "toilet_flush": "TOILET",
    "train": "TRAIN",
    "vacuum_cleaner": "VACUUM",
    "washing_machine": "WASHER",
    "water_drops": "WATERDROP",
    "wind": "WIND",
}


def build_label_map(rows: list[dict[str, str]]) -> dict[str, tuple[int, str]]:
    label_map = dict(TARGETS)
    next_label = max(label for label, _ in TARGETS.values()) + 1

    for row in sorted(rows, key=lambda item: int(item["target"])):
        category = row["category"]
        if category in label_map:
            continue
        label_map[category] = (next_label, SHORT_NAMES.get(category, category.upper()))
        next_label += 1

    return label_map


def download(url: str, target: pathlib.Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + ".tmp")
    if tmp.exists():
        tmp.unlink()
    urllib.request.urlretrieve(url, tmp)
    tmp.replace(target)


def parse_workers() -> int:
    for arg in sys.argv[1:]:
        if arg.startswith("--workers="):
            value = int(arg.split("=", 1)[1])
            return max(1, value)
    return DEFAULT_WORKERS


def fetch_audio(row: dict[str, str]) -> tuple[str, str]:
    filename = row["filename"]
    target = OUT_DIR / filename
    if target.exists() and target.stat().st_size > 0:
        return filename, "skip"

    download(f"{AUDIO_URL}/{filename}", target)
    return filename, "get"


def main() -> int:
    manifest_only = "--manifest-only" in sys.argv[1:]
    workers = parse_workers()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    META_DIR.mkdir(parents=True, exist_ok=True)

    if not META_FILE.exists():
        print(f"get metadata {META_URL}")
        download(META_URL, META_FILE)

    with META_FILE.open(newline="") as f:
        rows = list(csv.DictReader(f))

    if len(rows) < 2000:
        print(f"metadata has only {len(rows)} rows; expected ESC-50 2000 rows", file=sys.stderr)
        return 1

    label_map = build_label_map(rows)
    manifest_rows = []
    for row in rows:
        category = row["category"]
        label, label_name = label_map[category]
        manifest_rows.append(
            {
                "filename": row["filename"],
                "fold": row["fold"],
                "category": category,
                "esc50_target": row["target"],
                "label": str(label),
                "label_name": label_name,
            }
        )

    with MANIFEST.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "filename",
                "fold",
                "category",
                "esc50_target",
                "label",
                "label_name",
            ],
        )
        writer.writeheader()
        writer.writerows(manifest_rows)

    if manifest_only:
        print(f"manifest: {MANIFEST}")
        print(f"classes:  {len(label_map)}")
        print(f"clips:    {len(manifest_rows)}")
        print("audio download skipped because --manifest-only was used")
        return 0

    total = len(manifest_rows)
    done = 0
    downloaded = 0
    skipped = 0
    print(f"downloading {total} clips with {workers} workers")
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = [executor.submit(fetch_audio, row) for row in manifest_rows]
        for future in as_completed(futures):
            filename, status = future.result()
            done += 1
            if status == "get":
                downloaded += 1
            else:
                skipped += 1

            if (done % 25) == 0 or done == total:
                print(f"[{done}/{total}] downloaded={downloaded} skipped={skipped} last={filename}")

    print(f"manifest: {MANIFEST}")
    print(f"audio:    {OUT_DIR}")
    print(f"classes:  {len(label_map)}")
    print(f"ready:    {total} clips, {downloaded} downloaded this run, {skipped} skipped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
