# ESC-50 AI Pipeline

This folder builds the compact event classifier used by the STM32 firmware.
The final pipeline can train on the full ESC-50 dataset: 2000 clips across all
50 ESC-50 classes, while keeping the exported model small enough for the Blue
Pill firmware.

The first six labels are kept stable for the project UI/ring behavior:
- `CLAP`: `clapping`
- `KNOCK`: `door_wood_knock`
- `GLASS`: `glass_breaking`
- `COUGH`: `coughing`
- `LAUGH`: `laughing`
- `TICK`: `clock_tick`

The remaining 44 ESC-50 categories are added after those labels.

This is not a final production model. It is a bootstrap model so you can move
forward without collecting manual samples right now.

## Files

- `download_esc50_target.py`
  Downloads the official ESC-50 metadata and all clips for the six target
  categories.
- `train_esc50_target_model.py`
  Extracts firmware-aligned peak-window features, performs ESC-50 fold
  validation, trains the final model on all target clips, and exports the C
  header used by STM32.
- `download_esc50_2k.py`
  Downloads all 2000 official ESC-50 clips and writes a 50-class manifest.
- `train_esc50_full_model.py`
  Trains the full 50-class model on all 2000 ESC-50 clips with log-mel/MFCC
  features, writes `artifacts/esc50_full_model.joblib`, and records fold
  validation in `artifacts/starter_model.json`.
- `download_esc50_subset.py` and `train_starter_model.py`
  Legacy starter pipeline for the tiny subset. Keep these only for quick tests.

## Expected outputs

After training, these files appear in `artifacts/`:

- `starter_model.json`
- `esc50_full_model.joblib`
- `esc50_2k_logmel_features.npz`
- `feature_dump.csv`

`starter_model.h` is still produced by the compact firmware-oriented training
scripts, not by the 55%+ full-data trainer.

## How to use

1. Download the full ESC-50 data:

```bash
python3 ai/download_esc50_2k.py
```

2. Train the final 50-class high-accuracy model:

```bash
python3 ai/train_esc50_full_model.py
```

3. Use `artifacts/starter_model.json` to inspect fold accuracy and
   `artifacts/esc50_full_model.joblib` for desktop-side inference.

The high-accuracy full model is not exported to `starter_model.h`, because it
uses 1338 log-mel/MFCC features while the STM32 firmware currently computes the
compact 11-feature vector. Use `train_starter_model.py` only when regenerating
that compact firmware header.

## Important limitation

No public dataset will be perfect for your exact soldered microphones, room,
gain, and event thresholds. This model is a much stronger baseline than the
starter subset, but the best final accuracy will still come from adding your
own logged clips from the actual hardware environment.
