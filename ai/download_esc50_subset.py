#!/usr/bin/env python3
from __future__ import annotations

import csv
import pathlib
import sys
import urllib.request

BASE_URL = "https://raw.githubusercontent.com/karolpiczak/ESC-50/master/audio"
ROOT = pathlib.Path(__file__).resolve().parent
MANIFEST = ROOT / "esc50_subset_manifest.csv"
OUT_DIR = ROOT / "data" / "esc50_subset"


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    with MANIFEST.open(newline="") as f:
        rows = list(csv.DictReader(f))

    total = len(rows)
    downloaded = 0

    for idx, row in enumerate(rows, start=1):
        filename = row["filename"]
        target = OUT_DIR / filename
        if target.exists():
            print(f"[{idx}/{total}] skip {filename}")
            continue

        url = f"{BASE_URL}/{filename}"
        print(f"[{idx}/{total}] get  {filename}")
        try:
            urllib.request.urlretrieve(url, target)
        except Exception as exc:
            print(f"failed {filename}: {exc}", file=sys.stderr)
            return 1
        downloaded += 1

    print(f"done: {total} files ready, {downloaded} downloaded this run")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
