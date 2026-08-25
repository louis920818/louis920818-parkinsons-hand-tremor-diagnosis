#pragma once

/*
 * PADS v3 LiftHold-only research model
 * ------------------------------------
 * Source: pads_v3_limit_deployment.json
 * Spec: L1 Logistic Regression, C=0.2, Top 15; the last coefficient happens to be 0.
 * Limitation: trained on PADS wearable sensor data; external validation on MYOSA MPU6050 is not yet done.
 * Outputs a research score only; must not be used as a diagnosis, medical decision, or claim of clinical accuracy.
 */

#define PADS_MODEL_FEATURE_COUNT 15

static const char *const PADS_MODEL_FEATURE_NAMES[PADS_MODEL_FEATURE_COUNT] = {
  "acc_x__power_2_12_log",
  "acc_x__spectral_centroid",
  "acc_y__spectral_centroid",
  "gyro_y__mad",
  "acc_z__spectral_centroid",
  "gyro_x__frequency_stability",
  "acc_z__peak_prominence",
  "acc_x__ratio_3_7_to_2_12",
  "gyro_z__mad",
  "gyro_mag__iqr",
  "acc_x__amplitude_variability",
  "acc_z__spectral_entropy",
  "acc_y__ratio_3_7_to_2_12",
  "gyro_y__ratio_3_7_to_2_12",
  "gyro_x__spectral_centroid"
};

static const float PADS_MODEL_IMPUTER[PADS_MODEL_FEATURE_COUNT] = {
  -4.62141360f, 6.53754891f, 6.57489417f, 3.47378789f, 5.33342413f,
  1.92700076f, 10.70669121f, 0.42265262f, 1.61511598f, 6.97304457f,
  0.52685386f, 0.79626068f, 0.37192050f, 0.46940372f, 6.72866256f
};

static const float PADS_MODEL_SCALER_MEAN[PADS_MODEL_FEATURE_COUNT] = {
  -4.54808854f, 6.51319652f, 6.63282883f, 4.17313814f, 5.44142152f,
  1.91264826f, 11.10853522f, 0.43243705f, 2.80963513f, 9.54913797f,
  0.56913327f, 0.78009637f, 0.38149490f, 0.48299823f, 6.76812285f
};

static const float PADS_MODEL_SCALER_SCALE[PADS_MODEL_FEATURE_COUNT] = {
  0.51171628f, 0.98672092f, 1.02833769f, 2.89668348f, 1.02491733f,
  0.89523369f, 3.02504858f, 0.15230501f, 2.64201516f, 8.82250134f,
  0.31036363f, 0.08120282f, 0.15282394f, 0.15276103f, 0.95826636f
};

static const float PADS_MODEL_COEFFICIENTS[PADS_MODEL_FEATURE_COUNT] = {
  0.85802874f, 0.60724724f, -0.29184700f, 0.45949733f, 0.44483262f,
  -0.27173627f, -0.07002648f, -0.14449949f, 0.19245119f, 0.45778122f,
  -0.28629953f, 0.15603928f, 0.30312676f, 0.52407445f, 0.0f
};

static const float PADS_MODEL_INTERCEPT = 0.88270409f;
static const float PADS_MODEL_THRESHOLD = 0.54066564f;
static const char *const PADS_MODEL_VERSION = "pads-v3-limit-lifthold-lr-20260814";
