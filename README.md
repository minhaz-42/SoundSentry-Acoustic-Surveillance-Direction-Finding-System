# SoundSentry

SoundSentry is an STM32-based acoustic surveillance and direction-finding system that detects sound events, estimates where they came from, classifies them with a lightweight embedded AI model, and records the result for later review.

## Overview

The firmware samples a 3-microphone array with ADC + DMA, tracks left/center/right direction, distinguishes short impulse events from longer continuous activity, drives a servo toward the detected direction, shows live status on an SSD1306 OLED, timestamps events with a DS3231 RTC, and stores event logs on an SD card through FatFs.

## Features

- 3-microphone acoustic sensing on STM32F103
- Left, center, and right direction estimation
- Event detection with simple threat heuristics
- On-device sound classification using a compact exported C model
- SSD1306 OLED status display
- Servo-based directional response
- DS3231 real-time clock timestamping
- SD card logging over SPI with FatFs
- Python training pipeline for generating the starter model

## Starter AI Classes

The current starter model is trained on a small ESC-50 subset and recognizes:

- `CLAP`
- `KNOCK`
- `GLASS`
- `COUGH`
- `LAUGH`
- `TICK`

This starter model is meant to bootstrap development. Real-world performance will depend on microphone quality, placement, gain, background noise, and how closely live events match the training data.

## Repository Layout

- `Core/` - main firmware source and headers
- `Drivers/` - STM32 HAL and CMSIS support files
- `FATFS/` - FatFs app and target integration
- `Middlewares/` - third-party FatFs source
- `ai/` - dataset helpers, training script, and exported model artifacts
- `Debug/` - generated build output currently included in the repo
- `331project.ioc` - STM32CubeMX project configuration

## Build And Run

1. Open or import the project in STM32CubeIDE.
2. Verify your hardware wiring for the microphones, OLED, RTC, servo, and SD card.
3. Build the firmware project.
4. Flash the target STM32 board.
5. Power the system and verify OLED status, RTC reads, and SD logging.

## Regenerating The AI Model

If you want to rebuild the starter classifier:

```bash
python3 ai/download_esc50_subset.py
python3 ai/train_starter_model.py
```

After training, rebuild the firmware so the regenerated `ai/artifacts/starter_model.h` is compiled into flash.

## Notes

- The main firmware implementation lives in `Core/Src/main.c`.
- The AI training notes and dataset details are in `ai/README.md`.
- The repository currently includes generated binaries and artifacts because they were part of the initial project snapshot.
