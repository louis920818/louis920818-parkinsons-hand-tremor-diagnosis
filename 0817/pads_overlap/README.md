# Overlapping (sliding) window analysis — correct vs inflated

The requirement was to "add overlapping windows." Overlapping windows are a standard technique for time-series signals (slicing one recording into overlapping segments to get more samples),
**but the key is how you group the cross-validation folds**: grouped correctly it is legitimate data augmentation; grouped wrongly it is data leakage and inflated numbers.

## Tunable parameters (at the top of `extract_windows.py`)
- `WIN_SEC`  length of each overlapping window (seconds), default 2.56 s (= 256 points @ 100 Hz)
- `OVERLAP`  overlap ratio 0–0.95, default 0.50 (50% overlap)
- others: FS = 100 Hz, TRIM_SEC = 0.5 (trim the start-up shake)

## Three evaluations (LiftHold, 8 features/window, 469 subjects, all "subject-level AUC")
| Method | PD vs healthy | PD vs other disorders | Reportable? |
|---|---|---|---|
| A No overlapping windows (one row per segment, subject-wise) | 0.772 | 0.637 | baseline |
| **B Overlapping windows, "correct"** (subject-wise grouping + averaging each subject's window probabilities) | **0.787** | **0.690** | ✅ yes |
| C Overlapping windows, "inflated" (windows split randomly, one subject's windows span train/test) | 0.813 | 0.756 | ⚠ invalid |

## Conclusion (for the advisor / report)
- **Done correctly (B), overlapping windows give a small genuine gain**: 0.772 → 0.787 (+0.015); PD vs other 0.637 → 0.690.
  Mechanism: more training samples + averaging a subject's multiple window probabilities at inference → more stable, slightly higher.
- **Done wrong (C) it leaks and inflates**: when windows are split randomly, a person's adjacent overlapping windows appear in both train and test, so the model has "seen this person" → AUC spuriously +0.026 more (up to 0.813). That number must not be written or claimed.
- **Correct method**: `StratifiedGroupKFold(groups=subject)`, so all of one person's windows stay in the same fold; aggregate per subject at inference.

## Output files
- `extract_windows.py` — overlapping-window extraction (parameters above; outputs `pads_overlap_windows.csv`, 5628 windows).
- `train_overlap2.py` — the A/B/C evaluations (subject-wise vs leakage).
- `pads_overlap_windows.csv` — window-level feature table.
- `overlap_compare.png` — comparison chart.

## Data source
- PADS dataset (469 subjects: HC 79 / PD 276 / DD 114), LiftHold raw signal resampled to 100 Hz.
- Numbers from `train_overlap2.py`, subject-wise 5-fold (averaged over 5 seeds).
