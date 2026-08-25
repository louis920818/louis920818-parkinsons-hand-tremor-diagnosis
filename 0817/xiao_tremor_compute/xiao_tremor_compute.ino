/*
 * XIAO ESP32S3 Tremor Compute Coprocessor
 * ----------------------------------------
 * Handles only UART data reception and IMU computation; does not start Wi-Fi / BLE.
 * MYOSA: TX GPIO26 -> XIAO D7(RX), RX GPIO27 <- XIAO D6(TX), common GND.
 * Arduino IDE: XIAO_ESP32S3, 8MB Flash, OPI PSRAM, USB CDC On Boot=Enabled.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include <stdlib.h>
#include <esp_heap_caps.h>
#include "TremorLinkProtocol.h"
#include "pads_v3_lifthold_model.h"
#include "pads_xgb_lifthold_model.h"   // XGBoost LiftHold model (replaces the LR scoring on the line above; features unchanged)
#include "pads_sensor_features.h"      // for the guided three-task: PADS rec feature DSP (XIAO has PSRAM, put the heavy buffers here)

using namespace TremorLink;

#define LINK_RX_PIN D7
#define LINK_TX_PIN D6
#define SAMPLE_HZ 200
#define FFT_SIZE 512
#define FFT_HOP_SIZE 256
#define MAX_SESSION_SAMPLES 4096
#define MAX_WINDOWS 15
#define STORED_AXES 6
#define FEATURE_CHANNELS 8

#define BAND_LO_HZ 2.0f
#define BAND_HI_HZ 12.0f
#define TREMOR_LO_HZ 3.0f
#define TREMOR_HI_HZ 7.0f
#define MIN_PEAK_PROM_DB 3.0f
#define QUALITY_FFT_MIN_PEAK_RMS_RATIO 0.75f   // matches MYOSA: peak/RMS below this = no clear periodic peak (noise), skip the drift re-measure

#define QUALITY_REASON_SAMPLING_RATE (1UL << 0)
#define QUALITY_REASON_READ_FAILURES (1UL << 1)
#define QUALITY_REASON_CLIPPED       (1UL << 2)
#define QUALITY_REASON_JITTER        (1UL << 3)
#define QUALITY_REASON_WINDOWS       (1UL << 4)
#define QUALITY_REASON_FFT_FREQ      (1UL << 5)
#define QUALITY_REASON_FFT_MAG       (1UL << 6)
#define QUALITY_REASON_FFT_RATIO     (1UL << 7)
#define QUALITY_REASON_UART          (1UL << 8)

enum StoredAxis : uint8_t { AX_ACC_X, AX_ACC_Y, AX_ACC_Z, AX_GYRO_X, AX_GYRO_Y, AX_GYRO_Z };
enum FeatureChannel : uint8_t { CH_ACC_X, CH_ACC_Y, CH_ACC_Z, CH_ACC_MAG, CH_GYRO_X, CH_GYRO_Y, CH_GYRO_Z, CH_GYRO_MAG };

struct WindowFeature {
  float dominantFrequency;
  float power3To7;
  float power2To12;
  float ratio3To7Over2To12;
  float spectralCentroid;
  float spectralEntropy;
  float peakProminenceDb;
  float amplitudeRms;
  float peakMagnitude;
};

HardwareSerial MyosaSerial(1);
FrameDecoder decoder;

float *channels[STORED_AXES] = {};
float fftReal[FFT_SIZE], fftImag[FFT_SIZE], fftPsd[FFT_SIZE / 2], hannWindow[FFT_SIZE];
float sessionAggregatePsd[FFT_SIZE / 2], windowAggregatePsd[FFT_SIZE / 2];
float sortScratch[MAX_SESSION_SAMPLES];
WindowFeature windowFeatures[FEATURE_CHANNELS][MAX_WINDOWS];

bool buffersReady = false;
bool sessionActive = false;
bool gravityInitialized = false;
bool sequenceError = false;
uint16_t currentSessionId = 0;
uint16_t expectedBlockSequence = 0;
uint16_t txSequence = 0;
uint16_t sampleCount = 0;
uint16_t scheduledCount = 0;
uint16_t readFailures = 0;
uint16_t clippedSamples = 0;
uint32_t firstTimestampUs = 0, lastTimestampUs = 0;
uint32_t lastHelloMs = 0, lastFrameMs = 0;
int32_t sourceMaxJitterUs = 0;
float sourceEffectiveFs = 0;
float gravity[3] = {}, gyroBias[3] = {};
float previousAccX = 0;
bool previousAccValid = false;
SessionBeginPayload sessionConfig = {};
TapSummaryPayload lastTapSummary = {};
float savedCalRestMedianMg = NAN;
float savedCalRestMaxMg = NAN;
int16_t savedGuidedRestScore = -1;

bool allocateBuffers() {
  bool ok = true;
  for (uint8_t axis = 0; axis < STORED_AXES; ++axis) {
    channels[axis] = static_cast<float *>(ps_malloc(MAX_SESSION_SAMPLES * sizeof(float)));
    if (!channels[axis]) channels[axis] = static_cast<float *>(malloc(MAX_SESSION_SAMPLES * sizeof(float)));
    if (!channels[axis]) ok = false;
  }
  return ok;
}

void fft(float *real, float *imag, int n) {
  for (int i = 1, j = 0; i < n; ++i) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float t = real[i]; real[i] = real[j]; real[j] = t;
      t = imag[i]; imag[i] = imag[j]; imag[j] = t;
    }
  }
  for (int length = 2; length <= n; length <<= 1) {
    const float angle = -2.0f * PI / length;
    const float baseReal = cosf(angle), baseImag = sinf(angle);
    for (int i = 0; i < n; i += length) {
      float wr = 1, wi = 0;
      for (int k = 0; k < length / 2; ++k) {
        const float ur = real[i + k], ui = imag[i + k];
        const float vr = real[i + k + length / 2] * wr - imag[i + k + length / 2] * wi;
        const float vi = real[i + k + length / 2] * wi + imag[i + k + length / 2] * wr;
        real[i + k] = ur + vr; imag[i + k] = ui + vi;
        real[i + k + length / 2] = ur - vr; imag[i + k + length / 2] = ui - vi;
        const float nextWr = wr * baseReal - wi * baseImag;
        wi = wr * baseImag + wi * baseReal; wr = nextWr;
      }
    }
  }
}

float channelValue(uint8_t channel, int index) {
  if (channel <= CH_ACC_Z) return channels[channel][index];
  if (channel == CH_ACC_MAG) {
    const float x = channels[AX_ACC_X][index], y = channels[AX_ACC_Y][index], z = channels[AX_ACC_Z][index];
    return sqrtf(x * x + y * y + z * z);
  }
  if (channel <= CH_GYRO_Z) return channels[channel - CH_GYRO_X + AX_GYRO_X][index];
  const float x = channels[AX_GYRO_X][index], y = channels[AX_GYRO_Y][index], z = channels[AX_GYRO_Z][index];
  return sqrtf(x * x + y * y + z * z);
}

void computePsd(uint8_t channel, int start) {
  double meanSum = 0, hannPower = 0;
  for (int i = 0; i < FFT_SIZE; ++i) meanSum += channelValue(channel, start + i);
  const float mean = static_cast<float>(meanSum / FFT_SIZE);
  for (int i = 0; i < FFT_SIZE; ++i) {
    fftReal[i] = (channelValue(channel, start + i) - mean) * hannWindow[i];
    fftImag[i] = 0;
    hannPower += hannWindow[i] * hannWindow[i];
  }
  fft(fftReal, fftImag, FFT_SIZE);
  const float scale = SAMPLE_HZ * static_cast<float>(hannPower);
  for (int i = 0; i < FFT_SIZE / 2; ++i) {
    float density = (fftReal[i] * fftReal[i] + fftImag[i] * fftImag[i]) / (scale > 0 ? scale : 1.0f);
    if (i > 0) density *= 2.0f;
    fftPsd[i] = density;
  }
}

float integratePower(float lowHz, float highHz) {
  const float binHz = static_cast<float>(SAMPLE_HZ) / FFT_SIZE;
  int low = max(1, static_cast<int>(ceilf(lowHz / binHz)));
  int high = min(FFT_SIZE / 2 - 1, static_cast<int>(floorf(highHz / binHz)));
  double sum = 0;
  for (int i = low; i <= high; ++i) sum += fftPsd[i];
  return static_cast<float>(sum * binHz);
}

float aggregateBandRatio(const float *spectrum) {
  const float binHz = static_cast<float>(SAMPLE_HZ) / FFT_SIZE;
  const int analysisLow = max(1, static_cast<int>(ceilf(BAND_LO_HZ / binHz)));
  const int analysisHigh = min(FFT_SIZE / 2 - 1, static_cast<int>(floorf(BAND_HI_HZ / binHz)));
  const int tremorLow = max(analysisLow, static_cast<int>(ceilf(TREMOR_LO_HZ / binHz)));
  const int tremorHigh = min(analysisHigh, static_cast<int>(floorf(TREMOR_HI_HZ / binHz)));
  double analysisPower = 0, tremorPower = 0;
  for (int bin = analysisLow; bin <= analysisHigh; ++bin) {
    analysisPower += spectrum[bin];
    if (bin >= tremorLow && bin <= tremorHigh) tremorPower += spectrum[bin];
  }
  return analysisPower > 0 ? static_cast<float>(tremorPower / analysisPower) : 0.0f;
}

float aggregatePeakFrequency(const float *spectrum) {
  const float binHz = static_cast<float>(SAMPLE_HZ) / FFT_SIZE;
  const int low = max(1, static_cast<int>(ceilf(BAND_LO_HZ / binHz)));
  const int high = min(FFT_SIZE / 2 - 2, static_cast<int>(floorf(BAND_HI_HZ / binHz)));
  int peak = low;
  for (int bin = low + 1; bin <= high; ++bin) if (spectrum[bin] > spectrum[peak]) peak = bin;
  const float y0 = spectrum[peak - 1], y1 = spectrum[peak], y2 = spectrum[peak + 1];
  const float denominator = y0 - 2.0f * y1 + y2;
  const float delta = denominator != 0 ? 0.5f * (y0 - y2) / denominator : 0.0f;
  return (peak + delta) * binHz;
}

int compareFloat(const void *left, const void *right) {
  const float a = *static_cast<const float *>(left), b = *static_cast<const float *>(right);
  return (a > b) - (a < b);
}

float quantileSorted(const float *values, int count, float quantile) {
  if (count <= 0) return 0;
  const float position = quantile * (count - 1);
  const int lower = static_cast<int>(floorf(position)), upper = static_cast<int>(ceilf(position));
  const float fraction = position - lower;
  return values[lower] * (1.0f - fraction) + values[upper] * fraction;
}

float medianArray(float *values, int count) {
  qsort(values, count, sizeof(float), compareFloat);
  return quantileSorted(values, count, 0.5f);
}

WindowFeature analyzeWindow(uint8_t channel, int start) {
  WindowFeature feature = {};
  computePsd(channel, start);
  const float binHz = static_cast<float>(SAMPLE_HZ) / FFT_SIZE;
  const int low = max(1, static_cast<int>(ceilf(BAND_LO_HZ / binHz)));
  const int high = min(FFT_SIZE / 2 - 2, static_cast<int>(floorf(BAND_HI_HZ / binHz)));
  int peak = low, backgroundCount = 0;
  double powerSum = 0, weightedSum = 0, sumSq = 0;
  for (int i = low; i <= high; ++i) {
    if (fftPsd[i] > fftPsd[peak]) peak = i;
    powerSum += fftPsd[i];
    weightedSum += fftPsd[i] * i * binHz;
    sortScratch[backgroundCount++] = fftPsd[i];
  }
  feature.dominantFrequency = peak * binHz;
  feature.power3To7 = integratePower(TREMOR_LO_HZ, TREMOR_HI_HZ);
  feature.power2To12 = integratePower(BAND_LO_HZ, BAND_HI_HZ);
  feature.ratio3To7Over2To12 = feature.power2To12 > 0 ? feature.power3To7 / feature.power2To12 : 0;
  feature.spectralCentroid = powerSum > 0 ? static_cast<float>(weightedSum / powerSum) : 0;
  double entropy = 0;
  for (int i = low; i <= high; ++i) {
    const float probability = powerSum > 0 ? static_cast<float>(fftPsd[i] / powerSum) : 0;
    if (probability > 0) entropy -= probability * logf(probability);
  }
  feature.spectralEntropy = backgroundCount > 1 ? static_cast<float>(entropy / logf(backgroundCount)) : 0;
  const float background = medianArray(sortScratch, backgroundCount);
  feature.peakProminenceDb = 10.0f * log10f((fftPsd[peak] + 1e-18f) / (background + 1e-18f));
  feature.peakMagnitude = sqrtf(max(0.0f, fftPsd[peak]));
  for (int i = 0; i < FFT_SIZE; ++i) {
    const float value = channelValue(channel, start + i);
    sumSq += static_cast<double>(value) * value;
  }
  feature.amplitudeRms = sqrtf(static_cast<float>(sumSq / FFT_SIZE));
  return feature;
}

float medianWindowField(uint8_t channel, uint8_t field, int windowCount) {
  for (int i = 0; i < windowCount; ++i) {
    const WindowFeature &f = windowFeatures[channel][i];
    switch (field) {
      case 0: sortScratch[i] = f.dominantFrequency; break;
      case 1: sortScratch[i] = log10f(f.power2To12 + 1e-18f); break;
      case 2: sortScratch[i] = f.ratio3To7Over2To12; break;
      case 3: sortScratch[i] = f.spectralCentroid; break;
      case 4: sortScratch[i] = f.spectralEntropy; break;
      default: sortScratch[i] = f.peakProminenceDb; break;
    }
  }
  return medianArray(sortScratch, windowCount);
}

float channelMad(uint8_t channel) {
  for (int i = 0; i < sampleCount; ++i) sortScratch[i] = channelValue(channel, i);
  const float median = medianArray(sortScratch, sampleCount);
  for (int i = 0; i < sampleCount; ++i) sortScratch[i] = fabsf(channelValue(channel, i) - median);
  return medianArray(sortScratch, sampleCount);
}

float channelIqr(uint8_t channel) {
  for (int i = 0; i < sampleCount; ++i) sortScratch[i] = channelValue(channel, i);
  qsort(sortScratch, sampleCount, sizeof(float), compareFloat);
  return quantileSorted(sortScratch, sampleCount, 0.75f) - quantileSorted(sortScratch, sampleCount, 0.25f);
}

float frequencyStability(uint8_t channel, int windowCount) {
  int valid = 0;
  double mean = 0;
  for (int i = 0; i < windowCount; ++i) {
    if (windowFeatures[channel][i].peakProminenceDb >= MIN_PEAK_PROM_DB) {
      sortScratch[valid++] = windowFeatures[channel][i].dominantFrequency;
      mean += windowFeatures[channel][i].dominantFrequency;
    }
  }
  if (valid < 2) return NAN;
  mean /= valid;
  double variance = 0;
  for (int i = 0; i < valid; ++i) { const float d = sortScratch[i] - mean; variance += d * d; }
  return sqrtf(static_cast<float>(variance / valid));
}

float amplitudeVariability(uint8_t channel, int windowCount) {
  double mean = 0;
  for (int i = 0; i < windowCount; ++i) mean += windowFeatures[channel][i].amplitudeRms;
  mean /= max(1, windowCount);
  if (mean <= 0) return 0;
  double variance = 0;
  for (int i = 0; i < windowCount; ++i) {
    const float d = windowFeatures[channel][i].amplitudeRms - mean;
    variance += d * d;
  }
  return sqrtf(static_cast<float>(variance / max(1, windowCount))) / mean;
}

void prepareResearchSignals() {
  if (sampleCount < 2) return;
  for (uint8_t axis = AX_ACC_X; axis <= AX_ACC_Z; ++axis) {
    double mean = 0;
    for (int i = 0; i < sampleCount; ++i) mean += channels[axis][i];
    mean /= sampleCount;
    for (int i = 0; i < sampleCount; ++i) channels[axis][i] -= mean;
  }
  for (uint8_t axis = AX_GYRO_X; axis <= AX_GYRO_Z; ++axis) {
    double sumY = 0, sumXY = 0, sumXX = 0;
    for (int i = 0; i < sampleCount; ++i) {
      const float x = -1.0f + 2.0f * i / static_cast<float>(sampleCount - 1);
      sumY += channels[axis][i]; sumXY += x * channels[axis][i]; sumXX += x * x;
    }
    const float intercept = static_cast<float>(sumY / sampleCount);
    const float slope = sumXX > 0 ? static_cast<float>(sumXY / sumXX) : 0;
    for (int i = 0; i < sampleCount; ++i) {
      const float x = -1.0f + 2.0f * i / static_cast<float>(sampleCount - 1);
      channels[axis][i] -= slope * x + intercept;
    }
  }
}

float runPadsModel(int windowCount) {
  float feature[PADS_MODEL_FEATURE_COUNT] = {};
  feature[0] = medianWindowField(CH_ACC_X, 1, windowCount);
  feature[1] = medianWindowField(CH_ACC_X, 3, windowCount);
  feature[2] = medianWindowField(CH_ACC_Y, 3, windowCount);
  feature[3] = channelMad(CH_GYRO_Y);
  feature[4] = medianWindowField(CH_ACC_Z, 3, windowCount);
  feature[5] = frequencyStability(CH_GYRO_X, windowCount);
  feature[6] = medianWindowField(CH_ACC_Z, 5, windowCount);
  feature[7] = medianWindowField(CH_ACC_X, 2, windowCount);
  feature[8] = channelMad(CH_GYRO_Z);
  feature[9] = channelIqr(CH_GYRO_MAG);
  feature[10] = amplitudeVariability(CH_ACC_X, windowCount);
  feature[11] = medianWindowField(CH_ACC_Z, 4, windowCount);
  feature[12] = medianWindowField(CH_ACC_Y, 2, windowCount);
  feature[13] = medianWindowField(CH_GYRO_Y, 2, windowCount);
  feature[14] = medianWindowField(CH_GYRO_X, 3, windowCount);

  // Changed from logistic regression (pads_v3) to XGBoost (pads_xgb_lifthold_model.h). Reuses exactly the 15 features above,
  // NaNs are imputed internally by the model with the training median. Main output = PD vs healthy; for PD vs other movement disorders call pads_lh_PD_vs_DD(feature).
  return pads_lh_PD_vs_HC(feature);   // returns PD probability (0~1); research screening, not a diagnosis
}

void resetSession(const FrameHeader &header, const SessionBeginPayload &config) {
  currentSessionId = header.sessionId;
  sessionConfig = config;
  expectedBlockSequence = 0;
  sampleCount = scheduledCount = readFailures = clippedSamples = 0;
  firstTimestampUs = lastTimestampUs = 0;
  sourceMaxJitterUs = 0; sourceEffectiveFs = 0;
  sequenceError = false; previousAccValid = false;
  memcpy(gravity, config.gravity, sizeof(gravity));
  memcpy(gyroBias, config.gyroBias, sizeof(gyroBias));
  const float gravityMagnitude = sqrtf(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
  gravityInitialized = gravityMagnitude > 0.2f;
  sessionActive = buffersReady;
}

void addSample(const RawSample &sample) {
  ++scheduledCount;
  if (!(sample.flags & SAMPLE_VALID)) { ++readFailures; return; }
  if (sampleCount >= MAX_SESSION_SAMPLES) return;
  const float ax = sample.accel[0] / 16384.0f;
  const float ay = sample.accel[1] / 16384.0f;
  const float az = sample.accel[2] / 16384.0f;
  if (!gravityInitialized) {
    gravity[0] = ax; gravity[1] = ay; gravity[2] = az;
    gravityInitialized = true;
  }
  const float alpha = constrain(sessionConfig.gravityAlpha, 0.0f, 0.99999f);
  gravity[0] = alpha * gravity[0] + (1.0f - alpha) * ax;
  gravity[1] = alpha * gravity[1] + (1.0f - alpha) * ay;
  gravity[2] = alpha * gravity[2] + (1.0f - alpha) * az;
  channels[AX_ACC_X][sampleCount] = ax - gravity[0];
  channels[AX_ACC_Y][sampleCount] = ay - gravity[1];
  channels[AX_ACC_Z][sampleCount] = az - gravity[2];
  channels[AX_GYRO_X][sampleCount] = sample.gyro[0] / 131.0f - gyroBias[0];
  channels[AX_GYRO_Y][sampleCount] = sample.gyro[1] / 131.0f - gyroBias[1];
  channels[AX_GYRO_Z][sampleCount] = sample.gyro[2] / 131.0f - gyroBias[2];
  if (sample.flags & SAMPLE_CLIPPED) ++clippedSamples;
  if (sampleCount == 0) firstTimestampUs = sample.timestampUs;
  lastTimestampUs = sample.timestampUs;
  ++sampleCount;
}

void copyText(char *destination, size_t capacity, const char *source) {
  if (!capacity) return;
  strncpy(destination, source, capacity - 1);
  destination[capacity - 1] = '\0';
}

void addQualityIssue(uint8_t &quality, uint32_t &reasons, uint8_t level, uint32_t reason) {
  quality = max(quality, level);
  reasons |= reason;
}

void analyzeSession(ResultPayload &result) {
  memset(&result, 0, sizeof(result));
  result.mode = sessionConfig.mode;
  result.flags = RESULT_LINK_COMPLETE;
  result.padsProbability = NAN;
  result.guidedCompositeScore = -1;
  result.guidedRestScore = -1;
  result.guidedTapScore = -1;
  result.guidedPostureScore = -1;
  if (sampleCount < FFT_SIZE) {
    result.quality = LINK_QUALITY_REPEAT_TEST;
    result.classificationSuppressed = 1;
    result.qualityReasons = QUALITY_REASON_WINDOWS;
    copyText(result.signalLevel, sizeof(result.signalLevel), "INVALID");
    copyText(result.motorPattern, sizeof(result.motorPattern), "RESULT_SUPPRESSED");
    return;
  }

  double accelSq = 0, gyroSq = 0, jerkSq = 0;
  for (int i = 0; i < sampleCount; ++i) {
    const float x = channels[AX_ACC_X][i], y = channels[AX_ACC_Y][i], z = channels[AX_ACC_Z][i];
    accelSq += static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z;
    const float gx = channels[AX_GYRO_X][i], gy = channels[AX_GYRO_Y][i], gz = channels[AX_GYRO_Z][i];
    gyroSq += static_cast<double>(gx) * gx + static_cast<double>(gy) * gy + static_cast<double>(gz) * gz;
    if (i > 0) { const float jerk = (x - previousAccX) * SAMPLE_HZ; jerkSq += static_cast<double>(jerk) * jerk; }
    previousAccX = x;
  }
  const float rmsG = sqrtf(static_cast<float>(accelSq / sampleCount));
  result.rmsMg = rmsG * 1000.0f;
  result.gyroRmsDps = sqrtf(static_cast<float>(gyroSq / sampleCount));
  result.jerkRmsGps = sampleCount > 1 ? sqrtf(static_cast<float>(jerkSq / (sampleCount - 1))) : 0;

  const int windowCount = min(MAX_WINDOWS, 1 + (sampleCount - FFT_SIZE) / FFT_HOP_SIZE);
  result.windowCount = windowCount;
  memset(sessionAggregatePsd, 0, sizeof(sessionAggregatePsd));
  for (int window = 0; window < windowCount; ++window) {
    const int start = window * FFT_HOP_SIZE;
    for (uint8_t channel = 0; channel < FEATURE_CHANNELS; ++channel)
      windowFeatures[channel][window] = analyzeWindow(channel, start);
    memset(windowAggregatePsd, 0, sizeof(windowAggregatePsd));
    for (uint8_t axis = CH_ACC_X; axis <= CH_ACC_Z; ++axis) {
      computePsd(axis, start);
      for (int bin = 0; bin < FFT_SIZE / 2; ++bin) {
        windowAggregatePsd[bin] += fftPsd[bin];
        sessionAggregatePsd[bin] += fftPsd[bin];
      }
    }
    WindowFeature &qualityFeature = windowFeatures[CH_ACC_MAG][window];
    qualityFeature.dominantFrequency = aggregatePeakFrequency(windowAggregatePsd);
    qualityFeature.ratio3To7Over2To12 = aggregateBandRatio(windowAggregatePsd);
    const int peakBin = constrain(static_cast<int>(roundf(qualityFeature.dominantFrequency * FFT_SIZE / SAMPLE_HZ)),
                                  1, FFT_SIZE / 2 - 2);
    qualityFeature.peakMagnitude = sqrtf(max(0.0f, windowAggregatePsd[peakBin]));
  }
  for (int window = 0; window < windowCount; ++window) {
    const WindowFeature &feature = windowFeatures[CH_ACC_MAG][window];
    const bool tremorWindow = feature.dominantFrequency >= TREMOR_LO_HZ &&
                              feature.dominantFrequency <= TREMOR_HI_HZ &&
                              feature.ratio3To7Over2To12 >= sessionConfig.bandRatioMin &&
                              feature.amplitudeRms >= sessionConfig.ampDetect;
    if (tremorWindow) ++result.tremorWindowCount;
  }
  result.persistencePct = windowCount > 0
      ? 100.0f * result.tremorWindowCount / windowCount : 0.0f;

  for (int i = 0; i < windowCount; ++i) {
    const WindowFeature &x = windowFeatures[CH_ACC_X][i];
    const WindowFeature &y = windowFeatures[CH_ACC_Y][i];
    const WindowFeature &z = windowFeatures[CH_ACC_Z][i];
    sortScratch[i] = x.power2To12 + y.power2To12 + z.power2To12;
  }
  result.peakFreqHz = aggregatePeakFrequency(sessionAggregatePsd);
  result.bandRatio = aggregateBandRatio(sessionAggregatePsd);

  uint8_t quality = LINK_QUALITY_PASS;
  uint32_t reasons = 0;
  float qualityFrequencyRange = 0.0f;
  bool qualityFftChecked = false;
  const float effectiveFs = sourceEffectiveFs > 0 ? sourceEffectiveFs
      : ((sampleCount > 1 && lastTimestampUs != firstTimestampUs)
          ? (sampleCount - 1) * 1000000.0f / static_cast<uint32_t>(lastTimestampUs - firstTimestampUs) : 0);
  if (effectiveFs < 195.0f || effectiveFs > 205.0f)
    addQualityIssue(quality, reasons, LINK_QUALITY_REPEAT_TEST, QUALITY_REASON_SAMPLING_RATE);
  else if (effectiveFs < 198.0f || effectiveFs > 202.0f)
    addQualityIssue(quality, reasons, LINK_QUALITY_WARNING, QUALITY_REASON_SAMPLING_RATE);
  if (readFailures > 2) addQualityIssue(quality, reasons, LINK_QUALITY_REPEAT_TEST, QUALITY_REASON_READ_FAILURES);
  else if (readFailures > 0) addQualityIssue(quality, reasons, LINK_QUALITY_WARNING, QUALITY_REASON_READ_FAILURES);
  if (clippedSamples > 5) addQualityIssue(quality, reasons, LINK_QUALITY_REPEAT_TEST, QUALITY_REASON_CLIPPED);
  else if (clippedSamples > 0) addQualityIssue(quality, reasons, LINK_QUALITY_WARNING, QUALITY_REASON_CLIPPED);
  if (sourceMaxJitterUs > 2500) addQualityIssue(quality, reasons, LINK_QUALITY_REPEAT_TEST, QUALITY_REASON_JITTER);
  else if (sourceMaxJitterUs > 1000) addQualityIssue(quality, reasons, LINK_QUALITY_WARNING, QUALITY_REASON_JITTER);
  if (sequenceError || scheduledCount != sessionConfig.expectedSamples)
    addQualityIssue(quality, reasons, LINK_QUALITY_REPEAT_TEST, QUALITY_REASON_UART | QUALITY_REASON_WINDOWS);

  if (windowCount >= 2 && rmsG >= 0.010f) {
    float minFreq = 1e9f, maxFreq = -1e9f, minRatio = 1e9f, maxRatio = -1e9f;
    double magnitudeMean = 0;
    for (int i = 0; i < windowCount; ++i) {
      const WindowFeature &f = windowFeatures[CH_ACC_MAG][i];
      minFreq = min(minFreq, f.dominantFrequency); maxFreq = max(maxFreq, f.dominantFrequency);
      minRatio = min(minRatio, f.ratio3To7Over2To12); maxRatio = max(maxRatio, f.ratio3To7Over2To12);
      magnitudeMean += f.peakMagnitude;
    }
    magnitudeMean /= windowCount;
    // matches MYOSA: when there is no clear periodic peak (peak/RMS too low = broadband noise), the FFT max peak is just noise,
    // so the frequency/amplitude/band drift re-measure is not applied, avoiding a healthy person's slight shake being misjudged as REPEAT TEST.
    const bool dominantPeak = !(magnitudeMean > 1e-6 &&
                                (magnitudeMean / rmsG) < QUALITY_FFT_MIN_PEAK_RMS_RATIO);
    if (dominantPeak) {
      qualityFftChecked = true;
      double magnitudeVariance = 0;
      for (int i = 0; i < windowCount; ++i) {
        const float d = windowFeatures[CH_ACC_MAG][i].peakMagnitude - magnitudeMean;
        magnitudeVariance += d * d;
      }
      const float magnitudeCv = magnitudeMean > 0 ? sqrtf(magnitudeVariance / windowCount) / magnitudeMean : 0;
      const float frequencyRange = maxFreq - minFreq, ratioRange = maxRatio - minRatio;
      qualityFrequencyRange = frequencyRange;
      if (frequencyRange > 1.5f) addQualityIssue(quality, reasons, LINK_QUALITY_REPEAT_TEST, QUALITY_REASON_FFT_FREQ);
      else if (frequencyRange > 0.8f) addQualityIssue(quality, reasons, LINK_QUALITY_WARNING, QUALITY_REASON_FFT_FREQ);
      if (magnitudeCv > 0.60f) addQualityIssue(quality, reasons, LINK_QUALITY_REPEAT_TEST, QUALITY_REASON_FFT_MAG);
      else if (magnitudeCv > 0.35f) addQualityIssue(quality, reasons, LINK_QUALITY_WARNING, QUALITY_REASON_FFT_MAG);
      if (ratioRange > 0.35f) addQualityIssue(quality, reasons, LINK_QUALITY_REPEAT_TEST, QUALITY_REASON_FFT_RATIO);
      else if (ratioRange > 0.20f) addQualityIssue(quality, reasons, LINK_QUALITY_WARNING, QUALITY_REASON_FFT_RATIO);
    }
  }

  result.quality = quality;
  result.qualityReasons = reasons;
  result.classificationSuppressed = quality == LINK_QUALITY_REPEAT_TEST;

  if (sessionConfig.mode == MODE_GUIDED_REST) {
    const bool inBand = result.peakFreqHz >= TREMOR_LO_HZ && result.peakFreqHz <= TREMOR_HI_HZ;
    if (!result.classificationSuppressed && rmsG >= sessionConfig.ampDetect && inBand &&
        result.bandRatio >= sessionConfig.bandRatioMin) {
      const float denominator = max(0.001f, 1.0f - sessionConfig.bandRatioMin);
      const float bandScore = constrain(100.0f * (result.bandRatio - sessionConfig.bandRatioMin) / denominator, 0.0f, 100.0f);
      const float consistencyScore = qualityFftChecked
          ? constrain(100.0f * (1.0f - qualityFrequencyRange / 1.5f), 0.0f, 100.0f) : 0.0f;
      savedGuidedRestScore = static_cast<int16_t>(roundf(0.65f * bandScore + 0.35f * consistencyScore));
    } else {
      savedGuidedRestScore = result.classificationSuppressed ? -1 : 0;
    }
    result.guidedRestScore = savedGuidedRestScore;
  } else if (sessionConfig.mode == MODE_GUIDED_POSTURE) {
    result.guidedRestScore = savedGuidedRestScore;
    result.guidedTapScore = (lastTapSummary.quality != LINK_QUALITY_REPEAT_TEST && lastTapSummary.score <= 4)
        ? static_cast<int16_t>(lastTapSummary.score * 25) : -1;
    result.guidedPostureScore = static_cast<int16_t>(roundf(constrain(result.persistencePct, 0.0f, 100.0f)));
    if (!result.classificationSuppressed && result.guidedRestScore >= 0 && result.guidedTapScore >= 0) {
      result.guidedCompositeScore = static_cast<int16_t>(roundf(
          0.40f * result.guidedRestScore + 0.40f * result.guidedTapScore + 0.20f * result.guidedPostureScore));
    }
  }
  if (result.classificationSuppressed) {
    copyText(result.signalLevel, sizeof(result.signalLevel), "INVALID");
    copyText(result.motorPattern, sizeof(result.motorPattern), "RESULT_SUPPRESSED");
    return;
  }
  const float rms = result.rmsMg / 1000.0f;
  if (rms >= sessionConfig.ampSevere) copyText(result.signalLevel, sizeof(result.signalLevel), "HIGH");
  else if (rms >= sessionConfig.ampModerate) copyText(result.signalLevel, sizeof(result.signalLevel), "MEDIUM");
  else if (rms >= sessionConfig.ampDetect) copyText(result.signalLevel, sizeof(result.signalLevel), "LOW");
  else copyText(result.signalLevel, sizeof(result.signalLevel), "BELOW_THRESHOLD");
  const bool inBand = result.peakFreqHz >= TREMOR_LO_HZ && result.peakFreqHz <= TREMOR_HI_HZ;
  if (rms < sessionConfig.ampDetect) copyText(result.motorPattern, sizeof(result.motorPattern), "NO_CHARACTERISTIC_TREMOR");
  else if (!inBand || result.bandRatio < sessionConfig.bandRatioMin)
    copyText(result.motorPattern, sizeof(result.motorPattern), "NON_RHYTHMIC_MOTION");
  else {
    copyText(result.motorPattern, sizeof(result.motorPattern),
             rms >= sessionConfig.ampSevere ? "STRONG_RHYTHMIC_TREMOR" : "RHYTHMIC_TREMOR");
    result.flags |= RESULT_TREMOR_NOW;
  }

  if (sessionConfig.mode == MODE_GUIDED_REST || sessionConfig.mode == MODE_GUIDED_POSTURE) {
    // Guided three-task XGBoost: decimate 200->100Hz, gyro deg/s->rad/s, compute PADS rec features and send to MYOSA for the model.
    // (channels are still raw here: prepareResearchSignals only changes them below in LIFT_HOLD)
    static float a100[1024 * 3], g100[1024 * 3];      // 100Hz scratch (max 1024 points = 10.24s@100Hz)
    const float D2R = (float)(M_PI / 180.0);           // gyro deg/s -> rad/s (align with PADS training)
    int gn = 0;
    for (int i = 0; i + 1 < sampleCount && gn < 1024; i += 2, ++gn) {   // decimate: 200->100Hz
      a100[gn*3+0]=channels[AX_ACC_X][i]; a100[gn*3+1]=channels[AX_ACC_Y][i]; a100[gn*3+2]=channels[AX_ACC_Z][i];
      g100[gn*3+0]=channels[AX_GYRO_X][i]*D2R; g100[gn*3+1]=channels[AX_GYRO_Y][i]*D2R; g100[gn*3+2]=channels[AX_GYRO_Z][i]*D2R;
    }
    PadsRec pr = pads_rec_feats(a100, g100, gn);
    if (pr.valid) {
      result.padsFeat[0]=pr.peak;result.padsFeat[1]=pr.bratio;result.padsFeat[2]=pr.bpow;result.padsFeat[3]=pr.rms;
      result.padsFeat[4]=pr.grms;result.padsFeat[5]=pr.spec_ent;result.padsFeat[6]=pr.peak_sharp;result.padsFeat[7]=pr.lohi;
      result.padsFeat[8]=pr.g_bratio;result.padsFeat[9]=pr.freq_stab;result.padsFeat[10]=pr.ac_reg;result.padsFeat[11]=pr.jerk;
      result.flags |= RESULT_G3_VALID;
    }
  }

  if (sessionConfig.mode == MODE_LIFT_HOLD) {
    prepareResearchSignals();
    for (int window = 0; window < windowCount; ++window) {
      const int start = window * FFT_HOP_SIZE;
      for (uint8_t channel = 0; channel < FEATURE_CHANNELS; ++channel)
        windowFeatures[channel][window] = analyzeWindow(channel, start);
    }
    result.padsProbability = runPadsModel(windowCount);
    if (isfinite(result.padsProbability)) result.flags |= RESULT_PADS_VALID;
  }
}

void sendError(uint8_t error, uint8_t mode, uint16_t detail) {
  ErrorPayload payload{error, mode, detail};
  sendFrame(MyosaSerial, MSG_ERROR, currentSessionId, txSequence++, &payload, sizeof(payload));
}

void finishCalibration(const SessionEndPayload &endPayload) {
  if (sequenceError || scheduledCount != sessionConfig.expectedSamples ||
      endPayload.scheduledSamples != sessionConfig.expectedSamples || readFailures > 0) {
    CalibrationResultPayload invalid{};
    invalid.quality = LINK_QUALITY_REPEAT_TEST;
    invalid.valid = 0;
    invalid.qualityReasons = static_cast<uint16_t>(QUALITY_REASON_UART | QUALITY_REASON_WINDOWS |
                                                    (readFailures > 0 ? QUALITY_REASON_READ_FAILURES : 0));
    sendFrame(MyosaSerial, MSG_CALIBRATION_RESULT, currentSessionId, txSequence++, &invalid, sizeof(invalid));
    return;
  }
  const int windowCount = sampleCount / FFT_SIZE;
  if (windowCount <= 0) { sendError(LINK_SAMPLE_COUNT, sessionConfig.mode, sampleCount); return; }
  for (int window = 0; window < windowCount; ++window) {
    double sumSq = 0;
    const int start = window * FFT_SIZE;
    for (int i = 0; i < FFT_SIZE; ++i) {
      const float x = channels[AX_ACC_X][start + i], y = channels[AX_ACC_Y][start + i], z = channels[AX_ACC_Z][start + i];
      sumSq += static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z;
    }
    sortScratch[window] = sqrtf(static_cast<float>(sumSq / FFT_SIZE)) * 1000.0f;
  }
  qsort(sortScratch, windowCount, sizeof(float), compareFloat);
  const float median = quantileSorted(sortScratch, windowCount, 0.5f);
  const float maximum = sortScratch[windowCount - 1];
  CalibrationResultPayload result = {};
  result.quality = LINK_QUALITY_PASS;
  result.valid = 1;
  if (sessionConfig.mode == MODE_CAL_REST) {
    savedCalRestMedianMg = median; savedCalRestMaxMg = maximum;
    result.restMedianMg = median;
  } else {
    result.restMedianMg = savedCalRestMedianMg;
    result.shakeMedianMg = median;
    if (!isfinite(savedCalRestMaxMg) || savedCalRestMaxMg > 60.0f || median < 25.0f || median < savedCalRestMaxMg * 2.0f ||
        endPayload.readFailures > 20 || endPayload.clippedSamples > 10) {
      result.quality = LINK_QUALITY_REPEAT_TEST; result.valid = 0;
    } else {
      result.ampDetectMg = max(10.0f, savedCalRestMaxMg * 1.5f);
      result.ampSevereMg = median;
      result.ampModerateMg = sqrtf(result.ampDetectMg * result.ampSevereMg);
      result.ampMildMg = result.ampDetectMg;
    }
  }
  sendFrame(MyosaSerial, MSG_CALIBRATION_RESULT, currentSessionId, txSequence++, &result, sizeof(result));
}

void finishSession(const SessionEndPayload &endPayload) {
  sourceMaxJitterUs = endPayload.maxJitterUs;
  sourceEffectiveFs = endPayload.effectiveFs;
  readFailures = max(readFailures, endPayload.readFailures);
  clippedSamples = max(clippedSamples, endPayload.clippedSamples);
  if (sessionConfig.mode == MODE_CAL_REST || sessionConfig.mode == MODE_CAL_SHAKE) {
    finishCalibration(endPayload);
  } else {
    ResultPayload result;
    analyzeSession(result);
    const MessageType type = sessionConfig.mode == MODE_MONITOR_WINDOW ? MSG_WINDOW_RESULT : MSG_SESSION_RESULT;
    sendFrame(MyosaSerial, type, currentSessionId, txSequence++, &result, sizeof(result));
    Serial.print("# XIAO_RESULT,session="); Serial.print(currentSessionId);
    Serial.print(",mode="); Serial.print(result.mode);
    Serial.print(",freq_Hz="); Serial.print(result.peakFreqHz, 2);
    Serial.print(",rms_mg="); Serial.print(result.rmsMg, 1);
    Serial.print(",quality="); Serial.println(result.quality);
  }
  sessionActive = false;
}

void processFrame() {
  const FrameHeader &header = decoder.header();
  lastFrameMs = millis();
  if (header.type == MSG_HELLO) {
    HelloPayload hello{0x0000000FUL, "xiao-compute-v1"};
    sendFrame(MyosaSerial, MSG_HELLO, 0, txSequence++, &hello, sizeof(hello));
  } else if (header.type == MSG_SESSION_BEGIN && decoder.payloadLength() == sizeof(SessionBeginPayload)) {
    SessionBeginPayload config;
    memcpy(&config, decoder.payload(), sizeof(config));
    resetSession(header, config);
  } else if (header.type == MSG_SAMPLE_BLOCK && decoder.payloadLength() == sizeof(SampleBlockPayload) && sessionActive) {
    SampleBlockPayload block;
    memcpy(&block, decoder.payload(), sizeof(block));
    if (header.sessionId != currentSessionId || header.sequence != expectedBlockSequence) sequenceError = true;
    expectedBlockSequence = header.sequence + 1;
    const uint8_t count = min(block.count, SAMPLES_PER_BLOCK);
    for (uint8_t i = 0; i < count; ++i) addSample(block.samples[i]);
  } else if (header.type == MSG_TAP_SUMMARY && decoder.payloadLength() == sizeof(TapSummaryPayload)) {
    memcpy(&lastTapSummary, decoder.payload(), sizeof(lastTapSummary));
  } else if (header.type == MSG_SESSION_END && decoder.payloadLength() == sizeof(SessionEndPayload) && sessionActive) {
    SessionEndPayload endPayload;
    memcpy(&endPayload, decoder.payload(), sizeof(endPayload));
    finishSession(endPayload);
  }
}

void sendHello() {
  HelloPayload hello{0x0000000FUL, "xiao-compute-v1"};
  sendFrame(MyosaSerial, MSG_HELLO, 0, txSequence++, &hello, sizeof(hello));
}

void setup() {
  Serial.begin(115200);
  MyosaSerial.setRxBufferSize(4096);
  MyosaSerial.begin(UART_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  for (int i = 0; i < FFT_SIZE; ++i) hannWindow[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));
  buffersReady = allocateBuffers();
  Serial.println("# XIAO Tremor Compute v1");
  Serial.print("# PSRAM bytes="); Serial.println(ESP.getPsramSize());
  Serial.print("# buffers="); Serial.println(buffersReady ? "OK" : "FAIL");
  sendHello();
  lastHelloMs = millis();
}

void loop() {
  while (MyosaSerial.available()) {
    if (decoder.push(static_cast<uint8_t>(MyosaSerial.read()))) processFrame();
  }
  if (millis() - lastHelloMs >= 1000) {
    sendHello();
    lastHelloMs = millis();
  }
  if (sessionActive && millis() - lastFrameMs > 3000) {
    sendError(LINK_RESULT_TIMEOUT, sessionConfig.mode, sampleCount);
    sessionActive = false;
  }
  delay(1);
}
