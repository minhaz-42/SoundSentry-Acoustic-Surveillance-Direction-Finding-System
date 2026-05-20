# ESC-50 Starter AI Pipeline

This folder builds a tiny starter classifier for the `SoundSentry` project using a
small subset of the official ESC-50 dataset.

Current starter labels:
- `CLAP`: `clapping`
- `KNOCK`: `door_wood_knock`
- `GLASS`: `glass_breaking`
- `COUGH`: `coughing`
- `LAUGH`: `laughing`
- `TICK`: `clock_tick`

This is not a final production model. It is a bootstrap model so you can move
forward without collecting manual samples right now.

## Files

- `esc50_subset_manifest.csv`
  Small labeled subset definition.
- `download_esc50_subset.py`
  Downloads the WAV files listed in the manifest from the official ESC-50 repo.
- `train_starter_model.py`
  Extracts firmware-aligned audio features with `numpy`, trains a compact random forest in pure Python, and exports a C header for STM32.

## Expected outputs

After training, these files appear in `artifacts/`:

- `starter_model.json`
- `starter_model.h`
- `feature_dump.csv`

## How to use

1. Download the subset:

```bash
python3 ai/download_esc50_subset.py
```

2. Train the starter model:

```bash
python3 ai/train_starter_model.py
```

3. Rebuild the STM32 firmware so the regenerated `starter_model.h` is compiled into flash.

## Important limitation

The ESC-50 clips are full WAV recordings, while your STM32 firmware currently
works on event-level summaries. The trainer now mirrors the lightweight
feature approximation used on-device, but live hardware behavior can still
shift because microphone gain, noise, and timing differ from the dataset.
