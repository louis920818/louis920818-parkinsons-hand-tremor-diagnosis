# PADS sensor-only XGBoost — XIAO ESP32-S3 port

Using the **sensor-only algorithm** of `pads_sensor_only.py` as the reference, the model is changed from Random Forest to **XGBoost** and ported to pure C that flashes onto the XIAO ESP32-S3. **For research screening, not a diagnosis (needs external clinical validation).**

## Validation (strict subject-wise repeated CV, 469 subjects, passes the label-shuffle leakage test)

| Task | Random Forest | **XGBoost** | Leakage test (shuffled labels) |
|---|---|---|---|
| PD vs healthy | AUC 0.864 | **0.866** | 0.504 (≈ 0.5 → no leakage ✓) |
| PD vs essential tremor | AUC 0.906 | **0.890** | 0.537 (✓) |

XGBoost nearly ties RF, and it exports to smaller C that fits the MCU better. The C model matches Python XGBoost **sample-by-sample** (probability error ~2e-7, retrained with `tree_method=exact`).

## Files

**On-board (C / Arduino):**
| File | Description |
|---|---|
| `pads_xgb_model.h` | Pure C for the two XGBoost models (300 trees/model; `pads_predict_PD_vs_HC/ET(feat[66])` returns the PD probability) |
| `pads_sensor_features.h` | C version of the feature-extraction DSP (`pads_rec_feats` / `pads_rep_feats`, direct DFT aligned to numpy) |
| `pads_infer.h` | Assembles the per-task results into the 66-dim vector + calls the model (`pads_screen_PD_vs_HC/ET`) |
| `example_infer.ino` | Inference example skeleton |

**Reproduce/retrain (Python, in `train_and_export/`):**
`pads_sensor_only.py` (the original algorithm), `extract_on_device.py` (feature extraction), `train_xgb.py` (RF↔XGB validation + training the final model), `xgb_to_c.py` (export the C header), `sensor_feats.csv` (469×66 feature table), `xgb_HC.json`/`xgb_ET.json` (the final models).

## Usage

```c
#include "pads_infer.h"
PadsCapture cap; memset(&cap,0,sizeof(cap));
cap.relaxed     = pads_rec_feats(acc, gyr, N);   // rest
cap.stretchhold = pads_rec_feats(acc2,gyr2,N2);  // posture
cap.pointfinger = pads_rep_feats(acc3, N3);       // tapping
// ... other tasks; leave undone ones as .valid=0 (auto-imputed with the training median)
float prob = pads_screen_PD_vs_HC(&cap);          // 0-1
```

## 66-dim feature order
See the header comment at the top of `pads_xgb_model.h` (index 0..65). Split into: 12 rich features each for Relaxed/StretchHold + symmetry, HoldWeight/DrinkGlas, rest-vs-posture contrast, PointFinger/TouchNose tapping, and the other 5 sensor tasks.

## ⚠️ Deployment limitations (please understand)
1. **Multi-task capture**: 0.86/0.91 come from 11 action tasks. The device must capture them one by one via a guided flow (rest, posture, hold-weight, drink, tap…); un-captured tasks are auto-imputed with the median, but **the fewer tasks you do, the lower the accuracy**.
2. **A single watch has no symmetry features**: the original algorithm uses left/right-wrist `asym_bpow` and cross-wrist maxima; a single device has only one hand, so symmetry features are set to NaN and imputed, giving slightly lower accuracy than the paper.
3. **Sampling rate**: training used 100 Hz; if the device samples at 200 Hz, the capture must be **resampled to 100 Hz** before computing features, or the frequency axis won't line up.
4. **Domain gap / recommendation**: the most rigorous approach is to **re-validate** with this C feature definition on the device's own measured data (or retrain after recomputing with the C features), confirming the device matches PADS.
5. **Positioning**: research screening, not a diagnosis.

## Performance
One direct DFT per task (N≈1000) is a few tens of milliseconds; a whole screening on the ESP32-S3 is on the order of a few hundred milliseconds — enough for a one-shot readout. Model inference (600 trees) is pure bitwise comparison, microsecond-level.
