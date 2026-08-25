---
publishDate: 2026-08-24T00:00:00Z
title: Parkinsons Disease Hand Tremor Diagnosis System
excerpt: A wrist-worn, on-device prototype that quantifies Parkinsonian hand tremor from IMU signals and screens it with an on-chip XGBoost model — no cloud, validated on 469 real subjects.
image: /assets/images/parkinsons-hand-tremor-screening/cover.jpg
tags:
 - parkinsons
 - tremor
 - esp32
 - edge-ai
 - wearable
---

> Quantify hand tremor anywhere, on the device itself — an accessible screening tool for Parkinson's (a research prototype, not a substitute for professional medical diagnosis).

---

## Acknowledgements

Built on the **MYOSA Mini Kit** for **MYOSA Event 6.0 (IEEE Sensors Council)**.
Team: **National Ilan University** — Hsu Luo-Jie (firmware & hardware) and Tu Che-Yang / Louis (signal processing, system integration, validation).
Clinical validation uses the public **PADS smartwatch dataset** (PhysioNet, 469 subjects). Thanks to the MYOSA organizers and the IEEE Sensors community.

> **Important:** This is a **research and educational prototype** for tremor screening and quantification. It is **not a certified medical device** and is **not a substitute for professional medical diagnosis.**

---

## Overview

One of the earliest signs of Parkinson's disease is a **resting hand tremor at about 3–7 Hz**. It is easy to miss until a clinic visit, and it is clinically hard to tell apart from **Essential Tremor (ET)**. Yet there is no cheap, always-available tool to *quantify* it at home or in a clinic follow-up.

We built a **wrist-worn device on the MYOSA kit** that measures hand acceleration, extracts tremor features with an **on-device FFT**, and screens the signal with **two complementary judgments running on the chip itself** — a transparent rule-based threshold and a machine-learning model (**XGBoost**) ported into firmware. Everything runs **on-device in real time — no internet, no cloud upload**.

**Key features:**

* **On-device screening in ~10 s** — 200 Hz sampling, FFT, feature extraction, and inference all on the MCU.
* **Two judgments, one device** — an interpretable rule-based threshold *plus* an XGBoost model exported to a compact C header.
* **Four gesture-switched modes** — Tremor, Finger-Tap, Continuous Monitor, and a guided Comprehensive Exam.
* **Self-hosted live web dashboard** — the device serves its own web page over WiFi; view live waveform / FFT / results with no app install.
* **Clinically grounded** — the same features are validated on **469 real subjects** (PADS) with subject-wise cross-validation.

---

## Demo / Examples

### Images

<p align="center">
  <img src="/assets/images/parkinsons-hand-tremor-screening/system-architecture.png" width="800"><br/>
  <i>System architecture — MYOSA (ESP32-WROOM) + XIAO ESP32-S3 dual-board on-device screening, with offline clinical validation.</i>
</p>

<p align="center">
  <img src="/assets/images/parkinsons-hand-tremor-screening/hardware-components.jpg" width="460"><br/>
  <i>The actual wrist-worn build — MYOSA main board + XIAO ESP32-S3 co-processor, 0.96" OLED and APDS-9960 gesture sensor on elastic bands.</i>
</p>

<p align="center">
  <img src="/assets/images/parkinsons-hand-tremor-screening/algorithm-flowchart.png" width="800"><br/>
  <i>Algorithm flow — gesture trigger → sampling → quality check → FFT/features → rule-based + XGBoost judgment → output.</i>
</p>

<p align="center">
  <img src="/assets/images/parkinsons-hand-tremor-screening/software-stack.png" width="800"><br/>
  <i>Software stack — the on-device firmware (application → hardware) and the offline Python training pipeline that exports the on-board model.</i>
</p>

<p align="center">
  <img src="/assets/images/parkinsons-hand-tremor-screening/web-dashboard.png" width="420"><br/>
  <i>Live web dashboard (self-hosted by the ESP32): tremor / finger-tap / monitor modes and a guided exam with a 0–100 composite score.</i>
</p>

<p align="center">
  <img src="/assets/images/parkinsons-hand-tremor-screening/pads-validation.png" width="800"><br/>
  <i>Clinical validation on the PADS dataset (469 subjects, subject-wise cross-validation).</i>
</p>

### Videos

<video controls width="100%">
  <source src="/demo.mp4" type="video/mp4">
</video>

---

## Features (Detailed)

### 1. On-device tremor measurement
The device samples acceleration at a fixed **200 Hz for 10 s** (µs-scheduled loop), removes gravity with a high-pass filter, band-passes the signal, and runs an **FFT**. From the spectrum it computes peak frequency, `band_ratio` (3–7 Hz energy share), RMS amplitude, frequency stability, smoothness and left/right asymmetry.

### 2. Interpretable rule-based judgment (neutral)
A "PD-type tremor" pattern requires **three conditions together**: peak frequency **3–7 Hz**, `band_ratio ≥ 0.50`, and amplitude **≥ 13 mg** (milli-g, i.e. thousandths of gravity). The device outputs only **`signal_level`** (BELOW / LOW / MEDIUM / HIGH at 13 / 50 / 193 mg) and **`motor_pattern`** — a neutral description of the signal, never a disease label. Thresholds can be self-calibrated and stored in flash (NVS).

### 3. On-chip XGBoost (machine learning)
The rule path runs alongside an **XGBoost** model exported to a compact C header (`.h`) that runs directly on the MCU. On the device's strongest single task (LiftHold) it reaches **AUC ≈ 0.845** for PD vs. healthy (subject-wise 5-fold). A guided three-task model (rest + posture + tapping) replaces hand-tuned weighting.

### 4. Dual-board co-processing (XIAO + MYOSA)
The MYOSA main board (ESP32-WROOM, no PSRAM) cannot hold the heavier feature DSP, so a **XIAO ESP32-S3 (8 MB PSRAM)** co-processor computes the rich features and runs inference, then returns results to MYOSA over a small serial protocol. MYOSA keeps the model as a `const` array in flash, so on-board RAM stays free.

### 5. Four gesture-switched modes
Switch modes by **waving over the APDS gesture sensor** (no physical button): **Tremor** (frequency/amplitude/pattern), **Finger-Tap** (bradykinesia grade G0–G4, inspired by the MDS-UPDRS finger-tapping item), **Continuous Monitor** (background tremor-episode detection with false-alarm guarding), and **Comprehensive Exam** (guided rest → tap → posture → a 0–100 composite index with pre/post-medication comparison).

### 6. Self-hosted live dashboard
The ESP32 hosts its own web page over WiFi (WebSocket). A phone or laptop on the same hotspot sees the **live waveform, FFT spectrum, results, and CSV export** — no app, no cloud.

---

## Usage Instructions

1. **Wear** the device on the wrist and power it on (the OLED shows the current mode).
2. **Select a mode** by waving left / right / up / down over the APDS sensor.
3. **Hold your hand steady**; the device confirms posture, then measures for ~10 s.
4. **Read the result** on the OLED, or connect a phone/laptop to the device's WiFi and open the dashboard for live waveform, FFT and history.
5. For a full assessment, choose **Comprehensive Exam** and follow the guided rest → tap → posture steps.

Serial commands (for setup/testing): `c` self-calibrate · `m` tremor · `t` finger-tap · `o` monitor · `g` guided exam · `x` reset thresholds.

---

## Tech Stack

* **MYOSA main board — ESP32-WROOM-32E** (on-device measurement, rule judgment, WiFi dashboard)
* **XIAO ESP32-S3 (8 MB PSRAM)** — co-processor for heavy feature DSP + XGBoost inference
* **Sensors** — MPU6050 (6-axis IMU), APDS-9960 (gesture/proximity), SSD1306 OLED, buzzer (all I2C)
* **Firmware** — Arduino / C++ (ESP32 core), self-drawn WebSocket dashboard (`index_html.h`)
* **Machine learning** — Python, scikit-learn, **XGBoost** (trained offline, exported to a C header for the MCU)
* **Dataset** — PADS smartwatch dataset (PhysioNet, 469 subjects)

---

## Requirements / Installation

**The main codebase lives in the `0817/` folder** (see File Structure below). Two firmware sketches are flashed:

* `0817/xiao_tremor_compute/xiao_tremor_compute.ino` → **XIAO ESP32-S3**
* `0817/phase4_myosa_xiao/phase4_myosa_xiao.ino` → **MYOSA (ESP32-WROOM-32E)**

**Flash the firmware (Arduino IDE):**

```bash
# 1. Install the ESP32 board package in Arduino IDE (Boards Manager → esp32 by Espressif)
# 2. Install libraries: MPU6050, APDS-9960, Adafruit SSD1306, ArduinoWebsockets
# 3. Open each .ino, select the matching board, and Upload:
#      - xiao_tremor_compute.ino  → board: "XIAO_ESP32S3"
#      - phase4_myosa_xiao.ino     → board: "ESP32 Dev Module" (MYOSA WROOM-32E)
```

**Reproduce the model training (Python):**

```bash
pip install numpy pandas scipy scikit-learn xgboost
# training / export scripts live under 0817/pads_xgb*, which emit the C-header models (.h)
```

---

## File Structure

**`0817/` is the main codebase.** The two `.ino` files above are what you flash; the `pads_xgb*` folders train and export the on-board models.

```
0817/
├─ xiao_tremor_compute/          # XIAO ESP32-S3 firmware (MAIN) — feature DSP + XGBoost (LiftHold)
│  ├─ xiao_tremor_compute.ino
│  ├─ pads_xgb_lifthold_model.h  # on-board XGBoost model (C header)
│  └─ pads_sensor_features.h
├─ phase4_myosa_xiao/            # MYOSA ESP32-WROOM firmware (MAIN) — main board + live dashboard
│  ├─ phase4_myosa_xiao.ino
│  ├─ index_html.h               # self-hosted WebSocket dashboard
│  ├─ pads_xgb_guided3_model.h   # guided 3-task model (flash)
│  └─ TremorLinkProtocol.h       # dual-board (XIAO ↔ MYOSA) protocol
├─ pads_xgb_lifthold/            # LiftHold XGBoost training + export → .h
├─ pads_xgb_guided3/             # guided 3-task XGBoost training + export
├─ pads_xgb/                     # 66-dim multi-task XGBoost (library)
└─ pads_overlap/                 # overlapping-window analysis (rigorous validation)
```

---

## License

MIT License. The project is original work; the PADS dataset is used under its own open license (PhysioNet).

---

## Contribution Notes

Issues and pull requests are welcome. This is a screening/quantification prototype for research and education, and it is **not** a substitute for professional medical evaluation.
