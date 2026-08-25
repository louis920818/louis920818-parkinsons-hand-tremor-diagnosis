# Guided three-task XGBoost — replacing the hand-weighted CMPI (rest + posture + tap)

Per the requirement, the Guided comprehensive exam's **hand-weighted 0.40×rest + 0.40×tap + 0.20×posture** is replaced by a data-driven XGBoost that fuses the tasks.
**Research screening, not a diagnosis. Trained on the PADS smartwatch dataset; not re-validated on this device's own measured data.**

## Accuracy (PADS, 469 subjects, subject-wise 5-fold)
| Task | AUC | Leakage (6× shuffle) |
|---|---|---|
| PD vs healthy | 0.794 | 0.549 |
| PD vs essential tremor | 0.817 | 0.494 |

⚠️ This is **lower than the deployed LiftHold single-task XGBoost (0.845)** — because LiftHold happens to be this device's strongest single task; rest/tap individually are weaker, and even combined they don't catch up. The reason to keep the three-task model is "a multi-cue guided flow + removing arbitrary weights," not higher accuracy.

## Important limitation: tap features are currently imputed
The device's finger-tap test uses a "count the taps" protocol (it only stores `tapTimes`/`tapAmplitudes`) and has no continuous acceleration to compute PADS's 4 spectral tap features. So `feat[24..27]` are left as NaN on-device and imputed by the model with the training median — effectively equivalent to "rest + posture" (~0.78). To make tapping actually contribute, the tap protocol would need to record a stretch of continuous acceleration (a separate engineering effort).

## Files changed (phase4_myosa_xiao/)
- Added `pads_sensor_features.h` (feature DSP) and `pads_xgb_guided3_model.h` (model).
- `phase4_myosa_xiao.ino`:
  - At the end of the rest/posture tasks, `computeGuidedTaskPadsRec()` (200→100 Hz decimation, gyro deg/s→rad/s) computes and stores the PADS features.
  - `completeGuidedPostureTest()` assembles the 28-dim vector → `pads_g3_PD_vs_HC()`, outputting `pd_probability` (serial + WebSocket), coexisting with the original CMPI.

## Reproduce
`train_and_export/train_guided3.py` (trains on the rest/posture/tap columns of `pads_xgb/train_and_export/sensor_feats.csv` and exports the C header), `guided3_report.json`.

## Data source
- PADS smartwatch dataset (469 subjects: HC 79 / PD 276 / DD 114).
- Feature definitions: `pads_sensor_features.h` (i.e. the rec features of `pads_sensor_only.py`).
- Baselines: deployed LiftHold XGBoost (0.845), original hand-weighted CMPI.
