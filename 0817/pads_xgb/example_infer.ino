/*
 * PADS sensor-only PD screening -- XIAO ESP32-S3 inference example (skeleton)
 * For research screening, not a diagnosis. fs=100Hz; record one segment per task (recommend <=2048 samples).
 * Flow: guide the subject through each task -> compute PadsRec/PadsRep per segment -> assemble the 66-dim vector -> XGBoost inference.
 * A single device (one watch) cannot capture the left-right asymmetry features; they are auto-imputed with the training median (slightly lower accuracy than the two-watch version).
 */
#include "pads_infer.h"

// For each task, pass in that segment's acc[N][3], gyr[N][3] (after removing the first 0.5s of shake).
// Dummy data is used here for illustration; in practice it is captured from the MPU6050.
static float acc_buf[2048 * 3], gyr_buf[2048 * 3];

void run_screening() {
  PadsCapture cap;
  memset(&cap, 0, sizeof(cap));

  // ---- example: rest task ----
  int n_rest = capture_task_into(acc_buf, gyr_buf);   // <-- your capture function, returns sample count N
  cap.relaxed = pads_rec_feats(acc_buf, gyr_buf, n_rest);

  // ---- example: posture (StretchHold) ----
  int n_post = capture_task_into(acc_buf, gyr_buf);
  cap.stretchhold = pads_rec_feats(acc_buf, gyr_buf, n_post);

  // ---- example: finger-tap (PointFinger) ----
  int n_tap = capture_task_into(acc_buf, gyr_buf);
  cap.pointfinger = pads_rep_feats(acc_buf, n_tap);

  // ... other tasks (HoldWeight / DrinkGlas / TouchNose / RelaxedTask / LiftHold /
  //     CrossArms / TouchIndex / Entrainment) fill in as above; leave .valid=0 for tasks not done.

  float p_hc = pads_screen_PD_vs_HC(&cap);   // PD vs healthy (the ~0.86 AUC model)
  float p_et = pads_screen_PD_vs_ET(&cap);   // PD vs essential tremor (~0.90 AUC)

  Serial.printf("# PADS_XGB,pd_vs_hc=%.3f,pd_vs_et=%.3f,not_diagnosis=1\n", p_hc, p_et);
}

// Placeholder: implement your MPU6050 capture (if sampling at 200Hz, resample to 100Hz to align with training).
int capture_task_into(float* acc, float* gyr) { /* TODO fill acc/gyr, return N */ return 0; }

void setup() { Serial.begin(115200); }
void loop() { /* trigger run_screening() by gesture/button */ }
