# LiftHold single-task XGBoost — direct device-model swap (the current on-board solution)

**This is the option that can be flashed straight onto the XIAO within the time budget.** It changes neither the firmware's capture nor its features; it only replaces the logistic regression (`pads_v3_lifthold_model.h`) inside `runPadsModel()` with XGBoost (`pads_xgb_lifthold_model.h`), using the **same 15 features** the firmware already computes for the LiftHold task. After flashing, the device just runs one LiftHold measurement as usual to output a result.

**Research screening, not a diagnosis. Trained on the PADS smartwatch dataset; not yet re-validated on this device's (MYOSA + MPU6050) own measured data.**

## Validation (subject-wise 5-fold, 469 subjects × both wrists = 938 samples; passes the label-shuffle leakage test)
| Task | Current logistic regression | XGBoost (adopted) | Leakage (shuffle) |
|---|---|---|---|
| PD vs healthy (device main output) | AUC 0.839 | **0.845** | 0.483 (≈ 0.5 ✓) |
| PD vs other movement disorders (reference) | AUC 0.481 | **0.656** | 0.558 |

- On the main task (PD vs healthy) XGBoost **slightly beats** the current LR, and it improves greatly on PD vs other (where LR has almost no discriminative power).
- The C version matches Python XGBoost sample-by-sample (prob_err 3e-7; C-vs-XGB 1.2e-7).
- Hyperparameters: max_depth = 2, eta = 0.05, 400 trees, min_child_weight = 5 (a small, MCU-friendly model; tree_method = exact for bit-consistent export).

## Why LiftHold single-task (not the 66-dim multi-task)
The `pads_xgb/` (66-dim) version is more accurate (PD vs healthy 0.866), but it requires guiding the subject through 11 actions and writing a new capture flow — not feasible within the time budget. This solution reuses the device's **existing** LiftHold path, going on-board with zero capture changes — the pragmatic choice under time constraints. The slightly lower accuracy is an inevitable trade-off.

## Files changed
- `xiao_tremor_compute/pads_xgb_lifthold_model.h` (added): pure C for the two XGBoost models.
- `xiao_tremor_compute/xiao_tremor_compute.ino` (changed): added the include; the end of `runPadsModel()` changed from LR scoring to `return pads_lh_PD_vs_HC(feature);`. The 15-feature computation is untouched.

## Reproduce (train_and_export/)
- `lh_feats.py`: a Python copy of the firmware's 15-feature DSP (fs = 200 resampling, gyro rad/s→deg/s, aligned to the device). Verified against the deployed model's scaler mean to confirm the definitions are correct.
- `extract_lh.py`: batch extraction → `lh_sensor15_feats.csv` (938×15).
- `train_lh_xgb.py` / `final_lh.py`: subject-wise CV (vs LR + leakage test) and the final training/C export.
- `lh_xgb_report.json`: the numbers.

## Deployment limitations
1. **Single task**: uses only LiftHold; accuracy is lower than the multi-task version (0.845 vs the 66-dim's 0.866).
2. **Domain gap not validated**: the PADS watch ≠ this device's sensor; on-device re-validation was not done within the time budget (skipped per your instruction). It is recommended after launch.
3. **Sampling rate**: the training features were resampled to 200 Hz to align with the device's `SAMPLE_HZ = 200`; if the sampling rate changes later, they must be recomputed.
4. Research screening, not a diagnosis.

## Data source
- PADS smartwatch dataset (469 subjects: HC 79 / PD 276 / DD 114), LiftHold raw signal from movement/timeseries.
- Feature definitions: `analyzeWindow`/`runPadsModel` (15 features) in `xiao_tremor_compute.ino`.
- Baseline: `pads_v3_lifthold_model.h` (the existing L1 logistic regression).
