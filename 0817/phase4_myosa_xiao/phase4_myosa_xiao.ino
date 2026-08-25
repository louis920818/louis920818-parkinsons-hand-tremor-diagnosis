/*
 * ============================================================================
 *  Phase 4 (full proposal version) - Parkinson's hand-tremor screening and quantification prototype
 *  READY gesture selects mode / dwell to start -> measure or test -> rule-based screening output
 * ============================================================================
 *  Platform: MYOSA Mini Kit (ESP32-WROOM-32E + MPU6050 + APDS-9960 + SSD1306, all I2C)
 *  Dependencies: <Wire.h> + Adafruit_SSD1306, Adafruit_GFX (install via Library Manager).
 *
 *  * Current implementation focus:
 *    B2.2 gesture/start confirm -> left/right/up/down switch mode; Gesture FIFO starts after dwell reaches the set time
 *    B2.4 ambient light  -> records APDS clear-channel value; currently not used in compensation
 *    B8   multi-feature  -> besides frequency and RMS, adds: gyro RMS (stability), jerk RMS (smoothness),
 *                         peak, zero-crossing rate (ZCR)
 *    B6/B9 neutral output -> signal amplitude level + motion pattern (rule-based quantification, not ML, not a diagnosis)
 *
 *  ! This device is a "screening/quantification prototype", not a medical diagnosis. The three-level output is for reference only.
 *
 *  === READY === left=Monitor, right=Tap, up=Tremor, down=Guided; starts after dwell reaches MODE_START_HOLD_MS
 *  === commands ===  c=calibrate  x=clear calibration  m=tremor measure  l=LiftHold research model  t=Tap  o=Monitor  g=Guided  p=Proximity  r=raw CSV  z=gravity reset
 * ============================================================================
 */

#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <Preferences.h>
#include <Arduino_APDS9960.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "pads_v3_lifthold_model.h"
#include "pads_xgb_guided3_model.h"        // guided three-task XGBoost (model arrays only, in flash; features computed by XIAO and returned)
#include "TremorLinkProtocol.h"
#include "XiaoLinkClient.h"

// ==== WIFI/WS ADDON: connect to phone hotspot (STA) + serve web page ====
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>            // requires library: WebSockets (by Markus Sattler)
#include "index_html.h"                  // embedded index.html (same folder)
WebServer        httpServer(80);
WebSocketsServer webSocket = WebSocketsServer(81);
// WiFi credentials live in the private wifi_secrets.h; it still compiles if the file is missing, but WiFi won't connect.
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#define WIFI_STA_SSID "YOUR_WIFI_SSID"
#define WIFI_STA_PASS "YOUR_WIFI_PASSWORD"
#warning "wifi_secrets.h not found; using placeholder WiFi credentials"
#endif
const char* STA_SSID = WIFI_STA_SSID;
const char* STA_PASS = WIFI_STA_PASS;
// ==== WIFI/WS ADDON end ====

// ---------------- tunable parameters ----------------
#define BAUD            921600
#define MPU_ADDR        0x69
#define APDS_ADDR       0x39
#define OLED_ADDR       0x3C
#define OLED_W          128
#define OLED_H          64

#define SAMPLE_HZ       200
#define SAMPLE_US       (1000000UL / SAMPLE_HZ)
#define FFT_SIZE        512
#define FFT_HOP_SIZE    (FFT_SIZE / 2)  // 50% overlap: advance every 256 points
#define MEAS_TOTAL_SAMPLES 2048         // 10.24 s @ 200Hz
#define MEAS_WINDOWS    (1 + (MEAS_TOTAL_SAMPLES - FFT_SIZE) / FFT_HOP_SIZE) // 7 windows
#define CAL_WINDOWS     6
#define SETTLE_MS       1500
#define CAL_RESULT_MS   3000       // how long the calibration-done screen stays
#define GUIDED_REST_INSTRUCTION_MS 8000
#define GUIDED_STAGE_RESULT_MS      2500
#define GUIDED_TAP_INSTRUCTION_MS   8000
#define GUIDED_POSTURE_INSTRUCTION_MS 8000
#define GUIDED_POSTURE_WINDOWS         MEAS_WINDOWS  // also 2.56s / 50% overlap, 10.24s total
#define LIFT_HOLD_INSTRUCTION_MS       8000           // relax the arm first, then raise and extend it after the beep

#define BAND_LO_HZ      2.0f
#define BAND_HI_HZ      12.0f
#define TREMOR_LO_HZ    3.0f
#define TREMOR_HI_HZ    7.0f
#define MIN_PEAK_PROM_DB 3.0f
#define GRAV_ALPHA      0.978f
#define MEAS_LIVE_UPDATE 1         // update progress, frequency and amplitude after each FFT window

#define PROX_ON_THRESH  30

// The amplitude thresholds only describe the measured-signal level, not clinical severity.
// Can be updated by 'c' calibration and saved to NVS; 'x' clears the calibration and restores defaults.
const float AMP_DETECT_DEFAULT = 0.013f;
const float AMP_ELEVATED_DEFAULT = 0.050f;
const float AMP_STRONG_DEFAULT = 0.193f;
float ampDetect = AMP_DETECT_DEFAULT;
float ampElevated = AMP_ELEVATED_DEFAULT;
float ampStrong = AMP_STRONG_DEFAULT;
#define BAND_RATIO_MIN  0.50f      // band power ratio for 3-7 Hz / 2-12 Hz; confirm with real measurements

// Measurement Quality judges this measurement's reliability; REPEAT TEST blocks the pattern-classification output.
#define QUALITY_FS_PASS_MIN_HZ       198.0f
#define QUALITY_FS_PASS_MAX_HZ       202.0f
#define QUALITY_FS_WARN_MIN_HZ       195.0f
#define QUALITY_FS_WARN_MAX_HZ       205.0f
#define QUALITY_JITTER_PASS_MAX_US   1000
#define QUALITY_JITTER_WARN_MAX_US   2500
#define QUALITY_FFT_MIN_RMS_G        0.010f
#define QUALITY_FFT_MIN_PEAK_RMS_RATIO 0.75f
#define QUALITY_FREQ_PASS_RANGE_HZ   0.8f
#define QUALITY_FREQ_WARN_RANGE_HZ   1.5f
#define QUALITY_MAG_PASS_CV          0.35f
#define QUALITY_MAG_WARN_CV          0.60f
#define QUALITY_RATIO_PASS_RANGE     0.20f
#define QUALITY_RATIO_WARN_RANGE     0.35f

enum MeasurementQuality : uint8_t {
  QUALITY_PASS = 0,
  QUALITY_WARNING = 1,
  QUALITY_REPEAT_TEST = 2
};

enum QualityReason : uint16_t {
  QUALITY_REASON_SAMPLING_RATE = 1U << 0,
  QUALITY_REASON_READ_FAILURES = 1U << 1,
  QUALITY_REASON_CLIPPED       = 1U << 2,
  QUALITY_REASON_JITTER        = 1U << 3,
  QUALITY_REASON_WINDOWS       = 1U << 4,
  QUALITY_REASON_FFT_FREQ      = 1U << 5,
  QUALITY_REASON_FFT_MAG       = 1U << 6,
  QUALITY_REASON_FFT_RATIO     = 1U << 7
};

// calibration guards and NVS settings
#define CAL_REST_MAX_G          0.060f
#define CAL_TREMOR_MIN_G        0.025f
#define CAL_MIN_SEPARATION      2.0f
#define CAL_MAX_THRESHOLD_G     2.0f
#define CAL_MAX_READ_FAILURES   20
#define CAL_MAX_CLIPPED_SAMPLES 10
const char *CAL_NVS_NAMESPACE = "tremor_cal";
const char *CAL_NVS_KEY = "record";
const uint32_t CAL_NVS_MAGIC = 0x43414C31UL;  // "CAL1"
const uint32_t CAL_NVS_VERSION = 1;

struct CalibrationRecord {
  uint32_t magic;
  uint32_t version;
  float detect;
  float elevated;
  float strong;
  uint32_t checksum;
};

// buzzer (3-pin module GND/5V/SIG: SIG->GPIO25, 5V->VIN, GND->GND)
#define BUZZER_PIN      25
#define BUZZER_ACTIVE   1          // active=1 (sounds when powered); if it only "clicks" once after flashing it's passive, set to 0

// finger-tap screening: counts taps, speed, rhythm and interruptions; not a clinical MDS-UPDRS score
#define N_TAPS          10         // matches the 10 movements of the MDS-UPDRS finger-tapping task
#define TAP_POLL_MS       5        // fast tapping needs a higher poll rate; APDS wait time 2.78ms supports it
#define TAP_ON_MARGIN     4        // add this to the baseline max; balances sensitivity and noise immunity
#define TAP_RELEASE_RATIO 60       // OFF sits at 60% of baseline->ON, so the finger need not move fully away
#define TAP_REFRACT_MS   70        // minimum interval between taps; covers fast tapping and suppresses double-counting
#define TAPTEST_TIMEOUT_MS 25000
#define TAP_RELEASE_STABLE_MS 400  // how long it must stay stable after the start hand is removed
#define TAP_RELEASE_TIMEOUT_MS 5000
#define TAP_BASELINE_MS 1000

// APDS9960 gesture switching: left=Monitor(o), right=Tap(t), up=Tremor, down=Guided(g)
#define GESTURE_ENTER_THRESH 20     // verified on hardware by a standalone diagnostic program
#define GESTURE_EXIT_THRESH  10
#define GESTURE_SENSITIVITY  80
#define GESTURE_REPEAT_MS       200 // at least 0.2 s between two mode switches
#define GESTURE_FIFO_POLL_MS      15
#define GESTURE_CHANNEL_THRESH    30 // low-signal boundary of the official algorithm
#define GESTURE_DIRECTION_THRESH  20 // sensitivity=80 maps to internal threshold 20
#define MODE_START_HOLD_MS      3000 // how long to dwell in READY before starting the current mode
#define FIFO_HOLD_ON_THRESH        30 // when the FIFO four-direction average reaches this, a hand is present
#define FIFO_HOLD_OFF_THRESH       18 // hysteresis: only below this can the hand be considered gone
#define FIFO_HOLD_ON_COUNT          2 // two consecutive strong readings before dwell starts
#define FIFO_HOLD_LOST_MS         250 // how long low signal lasts before dwell is cancelled
#define MONITOR_EXIT_POLL_MS        75 // Monitor-exit proximity poll interval
#define MONITOR_EXIT_DRAW_MS       250 // OLED countdown update interval, to avoid disturbing MPU sampling
#define MONITOR_EXIT_ARM_MS        500 // after entering, move the hand away for 0.5 s before exit is allowed
#define MONITOR_EXIT_ARM_MAX        30 // below this the hand is considered removed
#define MONITOR_EXIT_PROX_ON       180 // getting this close starts the exit countdown
#define MONITOR_EXIT_PROX_OFF      100 // during countdown, below this it cancels
#define MONITOR_EXIT_HOLD_MS      3000 // stay close 3 s to exit Monitor
#define GESTURE_SWAP_LR  0          // if the module is mounted so left/right are reversed, set to 1
#define MODE_TREMOR     0
#define MODE_TAP        1
#define MODE_MONITOR    2
#define MODE_GUIDED     3

// ---------------- global buffers ----------------
float bufX[FFT_SIZE], bufY[FFT_SIZE], bufZ[FFT_SIZE];
float fReal[FFT_SIZE], fImag[FFT_SIZE];
float psd[FFT_SIZE / 2];
float qualityWindowPsd[FFT_SIZE / 2];
float hann[FFT_SIZE];
float hannPowerSum = 0;

// Eight channels matching the Python single-wrist model: Acc/Gyro XYZ + magnitude.
#define FEATURE_CHANNEL_COUNT 8
enum FeatureChannel : uint8_t {
  CH_ACC_X, CH_ACC_Y, CH_ACC_Z, CH_ACC_MAG,
  CH_GYRO_X, CH_GYRO_Y, CH_GYRO_Z, CH_GYRO_MAG
};
const char *FEATURE_CHANNEL_NAMES[FEATURE_CHANNEL_COUNT] = {
  "acc_x", "acc_y", "acc_z", "acc_mag",
  "gyro_x", "gyro_y", "gyro_z", "gyro_mag"
};
struct WindowSpectralFeatures {
  float dominantFrequency;
  float power3To7;
  float power2To12;
  float ratio3To7Over2To12;
  float ratio3To7Over7To12;
  float spectralCentroid;
  float spectralSpread;
  float spectralEntropy;
  float peakProminenceDb;
  float amplitudeRms;
};
WindowSpectralFeatures sessionWindowFeatures[FEATURE_CHANNEL_COUNT][MEAS_WINDOWS];
// Store only the six physical axes (48KB); the two magnitudes are computed on the fly to avoid WROOM-32E DRAM overflow.
#define STORED_AXIS_COUNT 6
enum StoredAxis : uint8_t { AX_ACC_X, AX_ACC_Y, AX_ACC_Z, AX_GYRO_X, AX_GYRO_Y, AX_GYRO_Z };
float sessionChannelSamples[STORED_AXIS_COUNT][MEAS_TOTAL_SAMPLES];
#define MAG_QUANTILE_SAMPLES 64          // reduced from 128 to 64 to free DRAM (2048 divisible; the magnitude quantile is insensitive to sample count, measurement length unchanged)
float magnitudeQuantileSamples[2][MAG_QUANTILE_SAMPLES];
bool analysisWindowPrimed = false;

int      bufIdx = 0;
float    gravX = 0, gravY = 0, gravZ = 0;
bool     gravInit = false;
bool     streamRaw = false;
uint32_t nextSampleUs = 0, windowStartUs = 0, sessionStartUs = 0;
int32_t  maxJitterUs = 0;
float    effectiveFs = 0;
int      mpuFailStreak = 0;
uint32_t sessionReadFailures = 0;
uint32_t sessionClippedSamples = 0;

// measurement accumulators
double   sessSumSq = 0;            // sum of squares of linear accel -> accel RMS
double   gyroSumSq = 0;            // sum of squares of angular velocity -> stability
double   jerkSumSq = 0;            // sum of squares of jerk -> smoothness
float    peakMag = 0;             // linear-acceleration peak
uint32_t zcrCount = 0;            // zero-crossing count (using lx)
float    prevLx = 0; bool prevLxValid = false; int prevSign = 0;
uint32_t sessSamples = 0;
int      sessWindows = 0;
float    qualityWindowFreq[MEAS_WINDOWS];
float    qualityWindowMagnitude[MEAS_WINDOWS];
float    qualityWindowBandRatio[MEAS_WINDOWS];
float    qualityWindowFs[MEAS_WINDOWS];
float    qualityFreqRangeHz = 0;
float    qualityMagnitudeCv = 0;
float    qualityBandRatioRange = 0;
float    qualityPeakToRmsRatio = 0;
bool     qualityFftChecked = false;
float    gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
double   gyroBiasSumX = 0, gyroBiasSumY = 0, gyroBiasSumZ = 0;
uint32_t gyroBiasSamples = 0;

// calibration
float    calRest[CAL_WINDOWS], calTremor[CAL_WINDOWS];
int      calIdx = 0;
uint32_t calReadFailures = 0;
uint32_t calClippedSamples = 0;

enum State { ST_IDLE, ST_SETTLE, ST_MEASURING, ST_RESULT, ST_CAL_REST, ST_CAL_TREMOR, ST_CAL_RESULT, ST_TAPTEST, ST_PROXMON, ST_MONITOR, ST_GUIDED_REST, ST_GUIDED_REST_DONE, ST_GUIDED_TAP, ST_GUIDED_TAP_DONE, ST_GUIDED_POSTURE, ST_GUIDED_POSTURE_SETTLE, ST_LIFT_HOLD_INSTRUCTION, ST_XIAO_WAIT };
State    state = ST_IDLE;
enum SessionTask : uint8_t { SESSION_TREMOR_QUANT, SESSION_REST, SESSION_LIFT_HOLD };
SessionTask sessionTask = SESSION_TREMOR_QUANT;
uint32_t settleStartMs = 0, lastPollMs = 0;
uint32_t calResultStartMs = 0;
uint32_t guidedRestInstructionStartMs = 0;
uint32_t guidedRestResultStartMs = 0;
uint32_t guidedTapInstructionStartMs = 0;
uint32_t guidedTapResultStartMs = 0;
uint32_t guidedPostureInstructionStartMs = 0;
uint32_t guidedPostureSettleStartMs = 0;
int8_t   guidedRestLastCountdown = -1;
int8_t   guidedTapLastCountdown = -1;
int8_t   guidedPostureLastCountdown = -1;
bool     guidedRestActive = false;
bool     guidedTapActive = false;
bool     guidedPostureActive = false;
bool     liftHoldActive = false;
uint32_t liftHoldInstructionStartMs = 0;
int8_t   liftHoldLastCountdown = -1;
uint8_t  guidedPostureWindows = 0, guidedPostureTremorWindows = 0;
float    guidedPosturePersistencePct = 0;
int      guidedTapPrototypeScore = -1;
float    g3RestFeat[12] = {}, g3PostFeat[12] = {};   // guided three-task: rest/posture PADS features returned by XIAO
bool     g3RestValid = false, g3PostValid = false;   // whether the corresponding features have been received
float    g3PdProbability = NAN;                        // PD probability from the guided three-task XGBoost (replaces the 0.40/0.40/0.20 manual weighting)
float    guidedPostureWindowFs[GUIDED_POSTURE_WINDOWS];
uint32_t guidedPostureReadFailures = 0, guidedPostureClippedSamples = 0;
MeasurementQuality guidedPostureQuality = QUALITY_PASS;
uint16_t guidedPostureQualityReasons = 0;
int      guidedRestScore = -1, guidedTapScore = -1, guidedPostureScore = -1, guidedCmpi = -1;
MeasurementQuality guidedOverallQuality = QUALITY_PASS;
// finger-tap test
enum TapPhase { TAP_WAIT_RELEASE, TAP_BASELINE, TAP_ACTIVE };
uint32_t tapTimes[N_TAPS]; int tapCount = 0; bool tapDown = false;
float    tapAmplitudes[N_TAPS];
uint16_t tapPeakProximity = 0;
uint32_t tapStartMs = 0, lastTapPollMs = 0;
uint32_t tapPhaseStartMs = 0, tapReleaseStableStartMs = 0, tapBaselineSum = 0;
uint16_t tapBaselineMax = 0, tapBaselineSamples = 0;
TapPhase tapPhase = TAP_WAIT_RELEASE;
uint16_t tapOn = 40, tapOff = 20;            // set dynamically from baseline calibration (adaptive threshold)
uint16_t tapBaseline = 0;
// mode selection + APDS gesture
int      selMode = MODE_TREMOR;
uint32_t lastGesturePollMs = 0, lastGestureSwitchMs = 0;
bool     gestureIn = false;
int      gestureDirectionX = 0, gestureDirectionY = 0;
int      gestureEntryX = 0, gestureEntryY = 0;
bool     handNear = false;
uint32_t holdStartMs = 0, fifoLastPresenceMs = 0;
uint16_t fifoPresenceX4 = 0;
uint8_t  fifoPresenceLevel = 0, fifoPresenceStreak = 0;
// continuous monitoring
double   monWinSumSq = 0; int monEpisodes = 0; int monTremorStreak = 0; bool monActive = false; uint32_t monStartMs = 0;
bool     monExitArmed = false;
uint32_t monExitAwayStartMs = 0, monExitHoldStartMs = 0, monExitLastPollMs = 0, monExitLastDrawMs = 0;
uint8_t  monExitProx = 0;
float    monLastFreq = 0, monLastRmsMg = 0;
// Tap/Tremor result stays on screen; exit is the same as Monitor: move the hand away first, then hold close 3 s.
bool     resultExitArmed = false, resultOledSaved = false;
uint32_t resultExitAwayStartMs = 0, resultExitHoldStartMs = 0;
uint32_t resultExitLastPollMs = 0, resultExitLastDrawMs = 0;
uint8_t  resultExitProx = 0;
uint8_t  resultOledBuffer[OLED_W * OLED_H / 8];
uint8_t  lastProx = 0;
bool     proxMonHeaderPrinted = false;
bool     apdsOk = false;

// most recent result
float    rFreq = 0, rRmsMg = 0, rBandRatio = 0, rGyro = 0, rJerk = 0, rPeak = 0, rZcr = 0;
uint16_t rAmbient = 0;
bool     rInBand = false;
const char *rSignalLevel = "--", *rMotorPattern = "--";
MeasurementQuality rQuality = QUALITY_PASS;
uint16_t rQualityReasons = 0;
float    rPadsResearchProbability = NAN;
bool     rPadsResearchValid = false;

// ---------------- XIAO compute co-processing ----------------
#define XIAO_UART_RX_PIN 27
#define XIAO_UART_TX_PIN 26
#define XIAO_RESULT_TIMEOUT_MS 2000
#define XIAO_WINDOW_TIMEOUT_MS 500

HardwareSerial xiaoSerial(2);
XiaoLinkClient xiaoLink(xiaoSerial);
TremorLink::RawSample xiaoLastRawSample{};
bool xiaoSessionOpen = false;
uint16_t xiaoSessionId = 0, xiaoAwaitSessionId = 0;
uint16_t xiaoScheduledSamples = 0, xiaoValidSamples = 0;
uint16_t xiaoReadFailures = 0, xiaoClippedSamples = 0;
TremorLink::SessionMode xiaoSessionMode = TremorLink::MODE_NONE;
bool xiaoAwaitingResult = false;
bool xiaoAwaitingGuidedPosture = false;
uint32_t xiaoResultDeadlineMs = 0;
uint32_t xiaoLastReportedOverruns = 0;

enum ComputeSource : uint8_t {
  COMPUTE_SOURCE_LOCAL,
  COMPUTE_SOURCE_XIAO,
  COMPUTE_SOURCE_LOCAL_FALLBACK
};
ComputeSource computeSource = COMPUTE_SOURCE_LOCAL;
char rSignalLevelStorage[16] = "--";
char rMotorPatternStorage[32] = "--";   // aligned with ResultPayload.motorPattern[32] to avoid truncating NO_CHARACTERISTIC_TREMOR

void broadcastCurrentResult();
void broadcastSessionWaveform();
void serviceXiaoCompute();
const char *computeSourceText();

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1, 400000UL, 400000UL);
bool     oledOk = false;

// ---------------- MPU6050 ----------------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
bool mpuBegin() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x01); delay(10);
  mpuWrite(0x1A, 0x03); mpuWrite(0x19, 0x04); mpuWrite(0x1B, 0x00); mpuWrite(0x1C, 0x00);
  delay(10); return true;
}
bool mpuRead(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, bool &clipped) {
  clipped = false;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return false;
  int16_t rax = (Wire.read()<<8)|Wire.read(), ray = (Wire.read()<<8)|Wire.read(), raz = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();
  int16_t rgx = (Wire.read()<<8)|Wire.read(), rgy = (Wire.read()<<8)|Wire.read(), rgz = (Wire.read()<<8)|Wire.read();
  clipped = rax <= -32760 || rax >= 32760 || ray <= -32760 || ray >= 32760 || raz <= -32760 || raz >= 32760 ||
            rgx <= -32760 || rgx >= 32760 || rgy <= -32760 || rgy >= 32760 || rgz <= -32760 || rgz >= 32760;
  ax = rax/16384.0f; ay = ray/16384.0f; az = raz/16384.0f;
  gx = rgx/131.0f;   gy = rgy/131.0f;   gz = rgz/131.0f;
  xiaoLastRawSample.timestampUs = micros();
  xiaoLastRawSample.accel[0] = rax; xiaoLastRawSample.accel[1] = ray; xiaoLastRawSample.accel[2] = raz;
  xiaoLastRawSample.gyro[0] = rgx; xiaoLastRawSample.gyro[1] = rgy; xiaoLastRawSample.gyro[2] = rgz;
  xiaoLastRawSample.flags = TremorLink::SAMPLE_VALID |
      (clipped ? TremorLink::SAMPLE_CLIPPED : 0);
  return true;
}

TremorLink::SessionMode xiaoModeForMeasurement() {
  if (sessionTask == SESSION_LIFT_HOLD) return TremorLink::MODE_LIFT_HOLD;
  if (guidedRestActive || sessionTask == SESSION_REST) return TremorLink::MODE_GUIDED_REST;
  return TremorLink::MODE_TREMOR_CAPTURE;
}

bool beginXiaoCompute(TremorLink::SessionMode mode, uint16_t expectedSamples) {
  if (!xiaoLink.online()) {
    xiaoSessionOpen = false;
    return false;
  }
  TremorLink::SessionBeginPayload config{};
  config.mode = mode;
  config.expectedSamples = expectedSamples;
  config.sampleHz = SAMPLE_HZ;
  config.gravityAlpha = GRAV_ALPHA;
  config.ampDetect = ampDetect;
  config.ampMild = ampDetect;
  config.ampModerate = ampElevated;
  config.ampSevere = ampStrong;
  config.bandRatioMin = BAND_RATIO_MIN;
  config.gyroBias[0] = gyroBiasX; config.gyroBias[1] = gyroBiasY; config.gyroBias[2] = gyroBiasZ;
  config.gravity[0] = gravX; config.gravity[1] = gravY; config.gravity[2] = gravZ;
  xiaoSessionId = xiaoLink.beginSession(config);
  xiaoSessionOpen = xiaoSessionId != 0;
  xiaoSessionMode = mode;
  xiaoScheduledSamples = xiaoValidSamples = xiaoReadFailures = xiaoClippedSamples = 0;
  if (xiaoSessionOpen) {
    Serial.print("# XIAO SESSION BEGIN,id="); Serial.print(xiaoSessionId);
    Serial.print(",mode="); Serial.println(static_cast<int>(mode));
  }
  return xiaoSessionOpen;
}

uint16_t endXiaoCompute(float measuredFs) {
  if (!xiaoSessionOpen) return 0;
  TremorLink::SessionEndPayload end{};
  end.mode = xiaoSessionMode;
  end.scheduledSamples = xiaoScheduledSamples;
  end.validSamples = xiaoValidSamples;
  end.readFailures = xiaoReadFailures;
  end.clippedSamples = xiaoClippedSamples;
  end.maxJitterUs = maxJitterUs;
  end.effectiveFs = measuredFs;
  const uint16_t endedId = xiaoSessionId;
  const bool queued = xiaoLink.endSession(end);
  xiaoSessionOpen = false;
  if (!queued) {
    Serial.println("# XIAO LINK ERROR,reason=TX_OVERFLOW");
    return 0;
  }
  Serial.print("# XIAO SESSION END,id="); Serial.print(endedId);
  Serial.print(",scheduled="); Serial.println(xiaoScheduledSamples);
  return endedId;
}

void queueSampleForXiao(bool valid) {
  if (!xiaoSessionOpen) return;
  TremorLink::RawSample sample = xiaoLastRawSample;
  sample.timestampUs = micros();
  ++xiaoScheduledSamples;
  if (valid) {
    ++xiaoValidSamples;
    if (sample.flags & TremorLink::SAMPLE_CLIPPED) ++xiaoClippedSamples;
  } else {
    memset(sample.accel, 0, sizeof(sample.accel));
    memset(sample.gyro, 0, sizeof(sample.gyro));
    sample.flags = 0;
    ++xiaoReadFailures;
  }
  if (!xiaoLink.addSample(sample)) {
    xiaoSessionOpen = false;
    Serial.println("# XIAO LINK ERROR,reason=SAMPLE_QUEUE_OVERFLOW");
    return;
  }

  // Normal Monitor sends independent 512-sample windows to XIAO; episode state stays on MYOSA.
  if (xiaoSessionMode == TremorLink::MODE_MONITOR_WINDOW && xiaoScheduledSamples >= FFT_SIZE) {
    const uint16_t endedId = endXiaoCompute(0.0f);
    if (endedId) beginXiaoCompute(TremorLink::MODE_MONITOR_WINDOW, FFT_SIZE);
  }
}

// ---------------- APDS-9960 (proximity + ambient/RGB) ----------------
void apdsWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(APDS_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
uint8_t apdsRead8(uint8_t reg) {
  Wire.beginTransmission(APDS_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(APDS_ADDR, 1, true) != 1) return 0;
  return Wire.available() ? Wire.read() : 0;
}
int apdsReadBlock(uint8_t reg, uint8_t *data, int len) {
  Wire.beginTransmission(APDS_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  int got = Wire.requestFrom(APDS_ADDR, len, true);
  for (int i = 0; i < got && Wire.available(); i++) data[i] = Wire.read();
  return got;
}
uint16_t apdsRead16(uint8_t regLo) { return apdsRead8(regLo) | (apdsRead8(regLo + 1) << 8); }
bool apdsBegin() {
  if (!APDS.begin()) return false;
  APDS.setGestureSensitivity(GESTURE_SENSITIVITY);
  Wire.setClock(400000);

  // Exactly matches the gesture_diagnostic settings verified on hardware.
  apdsWrite8(0x80, 0x00);   // disable sensing during configuration
  apdsWrite8(0x83, 0xFF);   // WTIME: 2.78ms
  apdsWrite8(0xA0, GESTURE_ENTER_THRESH); // GPENTH: gesture-engine enter threshold
  apdsWrite8(0xA1, GESTURE_EXIT_THRESH);  // GEXTH: gesture-engine exit threshold
  apdsWrite8(0xA2, 0x40);   // GCONF1: FIFO threshold=4 datasets
  apdsWrite8(0xA3, 0x51);   // GCONF2: gesture gain 4x, LED 25mA, wait 2.8ms (further reduces power-pulsing noise)
  apdsWrite8(0xA4, 0x00);   // GOFFSET_U
  apdsWrite8(0xA5, 0x00);   // GOFFSET_D
  apdsWrite8(0xA6, 0xC9);   // GPULSE: 32us, 10 pulses
  apdsWrite8(0xA7, 0x00);   // GOFFSET_L
  apdsWrite8(0xA9, 0x00);   // GOFFSET_R
  apdsWrite8(0xAA, 0x00);   // GCONF3: four-direction gestures
  apdsWrite8(0xAB, 0x01);   // GCONF4: gesture mode, no interrupt pin
  apdsWrite8(0x80, 0x4D);   // ENABLE: GEN | WEN | PEN | PON
  delay(10);
  return apdsRead8(0x92) == 0xAB && apdsRead8(0x80) == 0x4D;
}
uint8_t  apdsProximity() { return apdsRead8(0x9C); }
void apdsSetTapProximityProfile(bool tapProfile) {
  // CONTROL: keep LED 100mA, ALS gain 16x; for Tap only raise proximity gain from 1x to 2x.
  apdsWrite8(0x8F, tapProfile ? 0x06 : 0x02);
}
uint16_t apdsClear() {
  uint8_t enable = apdsRead8(0x80);
  bool ambientWasEnabled = (enable & 0x02) != 0;
  if (!ambientWasEnabled) {
    apdsWrite8(0x80, enable | 0x02); // enable AEN only temporarily while reading ambient light
    delay(12);                       // Arduino_APDS9960 ATIME is about 10ms
  }
  uint16_t clearValue = apdsRead16(0x94);
  if (!ambientWasEnabled) apdsWrite8(0x80, enable);
  return clearValue;
}

void noteReadyMotion(uint32_t now) {
  if (handNear) holdStartMs = now;
}

void cancelFifoHold(const char *reason) {
  if (handNear) {
    Serial.print("# HOLD cancelled: "); Serial.println(reason);
  }
  handNear = false;
  holdStartMs = 0;
  fifoPresenceStreak = 0;
  fifoPresenceLevel = 0;
  fifoPresenceX4 = 0;
}

void updateFifoPresence(uint8_t u, uint8_t d, uint8_t l, uint8_t r, uint32_t now) {
  uint8_t level = ((uint16_t)u + d + l + r) / 4;
  if (fifoPresenceX4 == 0) fifoPresenceX4 = (uint16_t)level * 4;
  else fifoPresenceX4 = (fifoPresenceX4 * 3 + (uint16_t)level * 4) / 4;
  fifoPresenceLevel = fifoPresenceX4 / 4;

  if (!handNear) {
    if (level >= FIFO_HOLD_ON_THRESH) {
      if (++fifoPresenceStreak >= FIFO_HOLD_ON_COUNT) {
        handNear = true;
        holdStartMs = now;
        fifoLastPresenceMs = now;
        fifoPresenceStreak = 0;
        Serial.println("# HOLD candidate from gesture FIFO");
      }
    } else {
      fifoPresenceStreak = 0;
    }
    return;
  }

  // After dwell starts, use a lower exit threshold so tiny noise while the hand is still doesn't interrupt the countdown.
  if (level > FIFO_HOLD_OFF_THRESH) fifoLastPresenceMs = now;
  else if (now - fifoLastPresenceMs >= FIFO_HOLD_LOST_MS) cancelFifoHold("hand left FIFO");
}

int apdsReadGesture() {
  uint32_t now = millis();
  if (now - lastGesturePollMs < GESTURE_FIFO_POLL_MS) return GESTURE_NONE;
  lastGesturePollMs = now;

  uint8_t level = apdsRead8(0xAE);                          // GFLVL
  bool gestureValid = (apdsRead8(0xAF) & 0x01) != 0;        // GSTATUS.GVALID
  if (level > 32) level = 32;
  if (level == 0) {
    if (handNear && now - fifoLastPresenceMs >= FIFO_HOLD_LOST_MS) {
      cancelFifoHold("FIFO timeout");
    }
    if (!gestureValid) {
      gestureDirectionX = gestureDirectionY = 0;
      gestureEntryX = gestureEntryY = 0;
      gestureIn = false;
    }
    return GESTURE_NONE;
  }

  uint8_t fifo[128];
  int got = apdsReadBlock(0xFC, fifo, level * 4);            // GFIFO_U/D/L/R
  int detectedGesture = GESTURE_NONE;
  for (int i = 0; i + 3 < got; i += 4) {
    uint8_t u = fifo[i], d = fifo[i+1], l = fifo[i+2], r = fifo[i+3];
    updateFifoPresence(u, d, l, r, now);

    bool lowSignal = u < GESTURE_CHANNEL_THRESH && d < GESTURE_CHANNEL_THRESH &&
                     l < GESTURE_CHANNEL_THRESH && r < GESTURE_CHANNEL_THRESH;
    if (lowSignal) {
      gestureIn = true;
      if (gestureEntryX != 0 || gestureEntryY != 0) {
        int totalX = gestureEntryX - gestureDirectionX;
        int totalY = gestureEntryY - gestureDirectionY;
        if (totalX < -GESTURE_DIRECTION_THRESH) detectedGesture = GESTURE_LEFT;
        if (totalX >  GESTURE_DIRECTION_THRESH) detectedGesture = GESTURE_RIGHT;
        if (totalY < -GESTURE_DIRECTION_THRESH) detectedGesture = GESTURE_DOWN;
        if (totalY >  GESTURE_DIRECTION_THRESH) detectedGesture = GESTURE_UP;
        gestureDirectionX = gestureDirectionY = 0;
        gestureEntryX = gestureEntryY = 0;
      }
      continue;
    }

    gestureDirectionX = (int)r - l;
    gestureDirectionY = (int)u - d;
    if (gestureIn) {
      gestureIn = false;
      gestureEntryX = gestureDirectionX;
      gestureEntryY = gestureDirectionY;
    }
  }

  int gesture = detectedGesture;
#if GESTURE_SWAP_LR
  if (gesture == GESTURE_LEFT) gesture = GESTURE_RIGHT;
  else if (gesture == GESTURE_RIGHT) gesture = GESTURE_LEFT;
#endif
  return gesture;
}

void apdsSetGestureEngine(bool enabled) {
  if (!apdsOk) return;
  if (enabled) {
    apdsWrite8(0xAB, 0x01); // GMODE=1
    apdsWrite8(0x80, 0x4D); // GEN | WEN | PEN | PON
  } else {
    apdsWrite8(0x80, 0x0D); // keep WEN | PEN | PON, only turn off Gesture
    apdsWrite8(0xAB, 0x00); // GMODE=0, end and clear gesture state
  }
}

void prepareGestureSelection() {
  lastPollMs = 0;
  lastGesturePollMs = 0;
  lastGestureSwitchMs = 0;
  gestureIn = false;
  gestureDirectionX = gestureDirectionY = 0;
  gestureEntryX = gestureEntryY = 0;
  handNear = false;
  holdStartMs = 0;
  fifoLastPresenceMs = 0;
  fifoPresenceX4 = 0;
  fifoPresenceLevel = 0;
  fifoPresenceStreak = 0;
  apdsSetTapProximityProfile(false);
  apdsSetGestureEngine(true);
}

// ---------------- buzzer ----------------
void buzzerInit() { pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW); }
void beep(int n, int onMs, int offMs) {
  for (int i = 0; i < n; i++) {
#if BUZZER_ACTIVE
    digitalWrite(BUZZER_PIN, HIGH); delay(onMs); digitalWrite(BUZZER_PIN, LOW);
#else
    int periods = (onMs * 1000) / 360;                  // passive: bit-bang ~2.7kHz square wave
    for (int p = 0; p < periods; p++) { digitalWrite(BUZZER_PIN, HIGH); delayMicroseconds(180); digitalWrite(BUZZER_PIN, LOW); delayMicroseconds(180); }
#endif
    if (i < n - 1) delay(offMs);
  }
}

// ---------------- FFT ----------------
void fft(float *re, float *im, int n) {
  for (int i = 1, j = 0; i < n; i++) { int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit; j ^= bit;
    if (i < j) { float t = re[i]; re[i]=re[j]; re[j]=t; t=im[i]; im[i]=im[j]; im[j]=t; } }
  for (int len = 2; len <= n; len <<= 1) {
    float ang = -2.0f*PI/len, wlr = cosf(ang), wli = sinf(ang);
    for (int i = 0; i < n; i += len) { float wr = 1, wi = 0;
      for (int k = 0; k < len/2; k++) {
        float ur = re[i+k], ui = im[i+k];
        float vr = re[i+k+len/2]*wr - im[i+k+len/2]*wi, vi = re[i+k+len/2]*wi + im[i+k+len/2]*wr;
        re[i+k]=ur+vr; im[i+k]=ui+vi; re[i+k+len/2]=ur-vr; im[i+k+len/2]=ui-vi;
        float nwr = wr*wlr - wi*wli; wi = wr*wli + wi*wlr; wr = nwr; } } }
}

float featureChannelSample(uint8_t channel, int index) {
  (void)channel;
  return bufX[index];  // during offline feature computation the selected channel is first loaded into the shared bufX.
}

void computeNormalizedChannelPsd(uint8_t channel) {
  double sum = 0;
  for (int i = 0; i < FFT_SIZE; i++) sum += featureChannelSample(channel, i);
  float mean = (float)(sum / FFT_SIZE);
  for (int i = 0; i < FFT_SIZE; i++) {
    // Matches Python one_sided_psd: each 2.56 s window has its mean removed first.
    fReal[i] = (featureChannelSample(channel, i) - mean) * hann[i];
    fImag[i] = 0;
  }
  fft(fReal, fImag, FFT_SIZE);
  float scale = SAMPLE_HZ * hannPowerSum;
  for (int i = 0; i < FFT_SIZE/2; i++) {
    float density = (fReal[i]*fReal[i] + fImag[i]*fImag[i]) / (scale > 0 ? scale : 1.0f);
    if (i > 0) density *= 2.0f;
    qualityWindowPsd[i] = density;
  }
}

float integratedFeaturePower(float loHz, float hiHz) {
  float binHz = (float)SAMPLE_HZ / FFT_SIZE;
  int lo = (int)ceilf(loHz / binHz), hi = (int)floorf(hiHz / binHz);
  if (lo < 1) lo = 1;
  if (hi > FFT_SIZE/2-1) hi = FFT_SIZE/2-1;
  double sum = 0;
  for (int i = lo; i <= hi; i++) sum += qualityWindowPsd[i];
  return (float)(sum * binHz);
}

float medianSmall(float *values, int count) {
  for (int i = 1; i < count; i++) {
    float key = values[i]; int j = i - 1;
    while (j >= 0 && values[j] > key) { values[j+1] = values[j]; j--; }
    values[j+1] = key;
  }
  if (count <= 0) return 0;
  return (count & 1) ? values[count/2] : 0.5f * (values[count/2-1] + values[count/2]);
}

WindowSpectralFeatures calculateWindowSpectralFeatures(uint8_t channel) {
  WindowSpectralFeatures result = {};
  computeNormalizedChannelPsd(channel);
  float binHz = (float)SAMPLE_HZ / FFT_SIZE;
  int lo = (int)ceilf(BAND_LO_HZ / binHz), hi = (int)floorf(BAND_HI_HZ / binHz);
  if (lo < 1) lo = 1;
  if (hi > FFT_SIZE/2-2) hi = FFT_SIZE/2-2;
  int peak = lo;
  double bandSum = 0, weighted = 0;
  float bandValues[64]; int bandCount = 0;
  for (int i = lo; i <= hi; i++) {
    if (qualityWindowPsd[i] > qualityWindowPsd[peak]) peak = i;
    bandSum += qualityWindowPsd[i];
    weighted += qualityWindowPsd[i] * (i * binHz);
    if (bandCount < 64) bandValues[bandCount++] = qualityWindowPsd[i];
  }
  // The deployed model must reproduce Python's argmax bin; the quality gate's separate frequency estimate can still use parabolic interpolation.
  result.dominantFrequency = peak * binHz;
  result.power3To7 = integratedFeaturePower(TREMOR_LO_HZ, TREMOR_HI_HZ);
  result.power2To12 = integratedFeaturePower(BAND_LO_HZ, BAND_HI_HZ);
  float power7To12 = integratedFeaturePower(7.0f, BAND_HI_HZ);
  result.ratio3To7Over2To12 = result.power2To12 > 0 ? result.power3To7 / result.power2To12 : 0;
  result.ratio3To7Over7To12 = power7To12 > 0 ? result.power3To7 / power7To12 : 0;
  result.spectralCentroid = bandSum > 0 ? (float)(weighted / bandSum) : 0;
  double spreadSum = 0, entropy = 0;
  for (int i = lo; i <= hi; i++) {
    float probability = bandSum > 0 ? (float)(qualityWindowPsd[i] / bandSum) : 0;
    float offset = i * binHz - result.spectralCentroid;
    spreadSum += probability * offset * offset;
    if (probability > 0) entropy -= probability * logf(probability);
  }
  result.spectralSpread = sqrtf((float)spreadSum);
  result.spectralEntropy = bandCount > 1 ? (float)(entropy / logf((float)bandCount)) : 0;
  float background = medianSmall(bandValues, bandCount);
  result.peakProminenceDb = 10.0f * log10f((qualityWindowPsd[peak] + 1e-18f) / (background + 1e-18f));
  double sumSq = 0;
  for (int i = 0; i < FFT_SIZE; i++) {
    float value = featureChannelSample(channel, i);
    sumSq += (double)value * value;
  }
  result.amplitudeRms = sqrtf((float)(sumSq / FFT_SIZE));
  return result;
}

float storedSessionFeatureValue(uint8_t channel, int index) {
  switch (channel) {
    case CH_ACC_X: return sessionChannelSamples[AX_ACC_X][index];
    case CH_ACC_Y: return sessionChannelSamples[AX_ACC_Y][index];
    case CH_ACC_Z: return sessionChannelSamples[AX_ACC_Z][index];
    case CH_ACC_MAG: {
      float x=sessionChannelSamples[AX_ACC_X][index], y=sessionChannelSamples[AX_ACC_Y][index], z=sessionChannelSamples[AX_ACC_Z][index];
      return sqrtf(x*x+y*y+z*z);
    }
    case CH_GYRO_X: return sessionChannelSamples[AX_GYRO_X][index];
    case CH_GYRO_Y: return sessionChannelSamples[AX_GYRO_Y][index];
    case CH_GYRO_Z: return sessionChannelSamples[AX_GYRO_Z][index];
    default: {
      float x=sessionChannelSamples[AX_GYRO_X][index], y=sessionChannelSamples[AX_GYRO_Y][index], z=sessionChannelSamples[AX_GYRO_Z][index];
      return sqrtf(x*x+y*y+z*z);
    }
  }
}

void computeAllSessionWindowFeatures() {
  // Do the 8-channel x 7-window FFT only after sampling completes, to avoid heavy computation inside the 5ms sampling period.
  for (int windowIndex = 0; windowIndex < MEAS_WINDOWS; windowIndex++) {
    int start = windowIndex * FFT_HOP_SIZE;
    for (uint8_t channel = 0; channel < FEATURE_CHANNEL_COUNT; channel++) {
      for (int i = 0; i < FFT_SIZE; i++) bufX[i] = storedSessionFeatureValue(channel, start+i);
      sessionWindowFeatures[channel][windowIndex] = calculateWindowSpectralFeatures(CH_ACC_X);
    }
  }
}

void prepareStoredSignalsForResearchFeatures() {
  if (sessSamples < 2) return;

  // Python remove_gravity_ema also removes each Acc axis's whole-segment mean at the end.
  for (uint8_t axis = AX_ACC_X; axis <= AX_ACC_Z; axis++) {
    double sum = 0;
    for (uint32_t i = 0; i < sessSamples; i++) sum += sessionChannelSamples[axis][i];
    float mean = (float)(sum / sessSamples);
    for (uint32_t i = 0; i < sessSamples; i++) sessionChannelSamples[axis][i] -= mean;
  }

  // Python gyro preprocessing is a whole-segment linear detrend; x uses evenly spaced [-1, 1] coordinates.
  for (uint8_t axis = AX_GYRO_X; axis <= AX_GYRO_Z; axis++) {
    double sumY = 0, sumXY = 0, sumXX = 0;
    for (uint32_t i = 0; i < sessSamples; i++) {
      float x = -1.0f + 2.0f * i / (float)(sessSamples - 1);
      float y = sessionChannelSamples[axis][i];
      sumY += y; sumXY += x * y; sumXX += x * x;
    }
    float intercept = (float)(sumY / sessSamples);
    float slope = sumXX > 0 ? (float)(sumXY / sumXX) : 0;
    for (uint32_t i = 0; i < sessSamples; i++) {
      float x = -1.0f + 2.0f * i / (float)(sessSamples - 1);
      sessionChannelSamples[axis][i] -= slope * x + intercept;
    }
  }
}

void shiftAnalysisBuffersForOverlap() {
  memmove(bufX,  bufX  + FFT_HOP_SIZE, FFT_HOP_SIZE * sizeof(float));
  memmove(bufY,  bufY  + FFT_HOP_SIZE, FFT_HOP_SIZE * sizeof(float));
  memmove(bufZ,  bufZ  + FFT_HOP_SIZE, FFT_HOP_SIZE * sizeof(float));
  bufIdx = FFT_HOP_SIZE;
}

int compareFloatForSort(const void *left, const void *right) {
  float a = *(const float *)left, b = *(const float *)right;
  return (a > b) - (a < b);
}

float quantileSorted(const float *values, int count, float quantile) {
  if (count <= 0) return 0;
  float position = quantile * (count - 1);
  int lower = (int)floorf(position), upper = (int)ceilf(position);
  float fraction = position - lower;
  return values[lower] * (1.0f - fraction) + values[upper] * fraction;
}

float windowFieldValue(const WindowSpectralFeatures &feature, uint8_t field) {
  switch (field) {
    case 0: return feature.dominantFrequency;
    case 1: return log10f(feature.power3To7 + 1e-18f);
    case 2: return log10f(feature.power2To12 + 1e-18f);
    case 3: return feature.ratio3To7Over2To12;
    case 4: return feature.ratio3To7Over7To12;
    case 5: return feature.spectralCentroid;
    case 6: return feature.spectralSpread;
    case 7: return feature.spectralEntropy;
    default: return feature.peakProminenceDb;
  }
}

float medianWindowField(uint8_t channel, uint8_t field) {
  float values[MEAS_WINDOWS];
  for (int i = 0; i < sessWindows; i++) values[i] = windowFieldValue(sessionWindowFeatures[channel][i], field);
  return medianSmall(values, sessWindows);
}

const char *sessionTaskName() {
  if (sessionTask == SESSION_REST) return "REST";
  if (sessionTask == SESSION_LIFT_HOLD) return "LIFT_HOLD";
  return "TREMOR_QUANT";
}

void runPadsResearchClassifier(const float *featureValues, MeasurementQuality sessionQuality) {
  float score = PADS_MODEL_INTERCEPT;
  bool valid = true;
  for (uint8_t i = 0; i < PADS_MODEL_FEATURE_COUNT; i++) {
    float value = featureValues[i];
    if (!isfinite(value)) {
      value = PADS_MODEL_IMPUTER[i];
      valid = false;
    }
    float scale = PADS_MODEL_SCALER_SCALE[i];
    if (!isfinite(scale) || scale <= 1e-12f) scale = 1.0f;
    score += PADS_MODEL_COEFFICIENTS[i] * ((value - PADS_MODEL_SCALER_MEAN[i]) / scale);
  }
  rPadsResearchProbability = 1.0f / (1.0f + expf(-score));
  bool qualityAccepted = sessionQuality != QUALITY_REPEAT_TEST;
  rPadsResearchValid = isfinite(rPadsResearchProbability) && qualityAccepted;

  Serial.print("# PADS_RESEARCH_SCORE,model="); Serial.print(PADS_MODEL_VERSION);
  Serial.print(",task=LiftHold,probability="); Serial.print(rPadsResearchProbability, 6);
  Serial.print(",threshold="); Serial.print(PADS_MODEL_THRESHOLD, 6);
  Serial.print(",screen_positive=");
  if (rPadsResearchValid) Serial.print(rPadsResearchProbability >= PADS_MODEL_THRESHOLD ? 1 : 0);
  else Serial.print("SUPPRESSED");
  Serial.print(",quality="); Serial.print(qualityLevelText(sessionQuality));
  Serial.print(",classification_suppressed="); Serial.print(qualityAccepted ? 0 : 1);
  Serial.print(",all_features_observed="); Serial.print(valid ? 1 : 0);
  Serial.println();

  if (rPadsResearchValid && !xiaoAwaitingResult) {
    String message = String("{\"type\":\"pads_research_score\",\"model\":\"") + PADS_MODEL_VERSION
      + "\",\"task\":\"LiftHold\",\"probability\":" + String(rPadsResearchProbability, 6)
      + ",\"threshold\":" + String(PADS_MODEL_THRESHOLD, 6)
      + ",\"screen_positive\":" + String(rPadsResearchProbability >= PADS_MODEL_THRESHOLD ? 1 : 0) + "}";
    webSocket.broadcastTXT(message);
  }
}

void printSixAxisFeatureSummary(MeasurementQuality sessionQuality) {
  const char *taskName = sessionTaskName();
  float deployFeatureValues[PADS_MODEL_FEATURE_COUNT];
  for (uint8_t i = 0; i < PADS_MODEL_FEATURE_COUNT; i++) deployFeatureValues[i] = NAN;
  float means[FEATURE_CHANNEL_COUNT], energies[FEATURE_CHANNEL_COUNT];
  float variances[FEATURE_CHANNEL_COUNT], minimums[FEATURE_CHANNEL_COUNT], maximums[FEATURE_CHANNEL_COUNT];
  // Compute the eight-channel moments before any sorting, so magnitude is still derived from time-aligned XYZ.
  for (uint8_t channel = 0; channel < FEATURE_CHANNEL_COUNT; channel++) {
    double sum = 0, sumSq = 0;
    float minimum = storedSessionFeatureValue(channel, 0), maximum = minimum;
    for (uint32_t i = 0; i < sessSamples; i++) {
      float value = storedSessionFeatureValue(channel, i);
      sum += value; sumSq += (double)value * value;
      if (value < minimum) minimum = value;
      if (value > maximum) maximum = value;
    }
    means[channel] = sessSamples ? (float)(sum / sessSamples) : 0;
    energies[channel] = sessSamples ? (float)(sumSq / sessSamples) : 0;
    variances[channel] = energies[channel] - means[channel] * means[channel];
    if (variances[channel] < 0) variances[channel] = 0;
    minimums[channel] = minimum; maximums[channel] = maximum;
  }
  for (uint8_t channel = 0; channel < FEATURE_CHANNEL_COUNT; channel++) {
    int storedAxis = -1;
    if (channel == CH_ACC_X) storedAxis = AX_ACC_X;
    else if (channel == CH_ACC_Y) storedAxis = AX_ACC_Y;
    else if (channel == CH_ACC_Z) storedAxis = AX_ACC_Z;
    else if (channel == CH_GYRO_X) storedAxis = AX_GYRO_X;
    else if (channel == CH_GYRO_Y) storedAxis = AX_GYRO_Y;
    else if (channel == CH_GYRO_Z) storedAxis = AX_GYRO_Z;
    float *samples = storedAxis >= 0 ? sessionChannelSamples[storedAxis]
                                     : magnitudeQuantileSamples[channel == CH_ACC_MAG ? 0 : 1];
    int quantileCount = storedAxis >= 0 ? (int)sessSamples : MAG_QUANTILE_SAMPLES;
    qsort(samples, quantileCount, sizeof(float), compareFloatForSort);
    float median = quantileSorted(samples, quantileCount, 0.50f);
    float q1 = quantileSorted(samples, quantileCount, 0.25f);
    float q3 = quantileSorted(samples, quantileCount, 0.75f);
    for (int i = 0; i < quantileCount; i++) samples[i] = fabsf(samples[i] - median);
    qsort(samples, quantileCount, sizeof(float), compareFloatForSort);
    float mad = quantileSorted(samples, quantileCount, 0.50f);

    float validFreq[MEAS_WINDOWS], tremorPower[MEAS_WINDOWS], amplitudes[MEAS_WINDOWS];
    int validCount = 0, persistenceCount = 0;
    double powerSum = 0, amplitudeSum = 0;
    for (int i = 0; i < sessWindows; i++) {
      const WindowSpectralFeatures &feature = sessionWindowFeatures[channel][i];
      if (feature.peakProminenceDb >= MIN_PEAK_PROM_DB) {
        validFreq[validCount++] = feature.dominantFrequency;
        if (feature.dominantFrequency >= TREMOR_LO_HZ && feature.dominantFrequency <= TREMOR_HI_HZ)
          persistenceCount++;
      }
      tremorPower[i] = feature.power3To7;
      amplitudes[i] = feature.amplitudeRms;
      powerSum += tremorPower[i]; amplitudeSum += amplitudes[i];
    }
    float freqStability = NAN;
    if (validCount >= 2) {
      double freqMean = 0, freqVariance = 0;
      for (int i = 0; i < validCount; i++) freqMean += validFreq[i];
      freqMean /= validCount;
      for (int i = 0; i < validCount; i++) { float d = validFreq[i] - freqMean; freqVariance += d*d; }
      freqStability = sqrtf((float)(freqVariance / validCount));
    }
    float powerMean = sessWindows ? (float)(powerSum / sessWindows) : 0;
    float amplitudeMean = sessWindows ? (float)(amplitudeSum / sessWindows) : 0;
    double powerVariance = 0, amplitudeVariance = 0;
    for (int i = 0; i < sessWindows; i++) {
      float pd = tremorPower[i] - powerMean, ad = amplitudes[i] - amplitudeMean;
      powerVariance += pd*pd; amplitudeVariance += ad*ad;
    }
    float powerCv = powerMean > 0 ? sqrtf((float)(powerVariance / sessWindows)) / powerMean : 0;
    float amplitudeCv = amplitudeMean > 0 ? sqrtf((float)(amplitudeVariance / sessWindows)) / amplitudeMean : 0;

    // Fields needed by the LiftHold Top-15 model; the remaining full six-axis features are still output for research audit.
    if (channel == CH_ACC_X) {
      deployFeatureValues[0] = medianWindowField(channel, 2);
      deployFeatureValues[1] = medianWindowField(channel, 5);
      deployFeatureValues[7] = medianWindowField(channel, 3);
      deployFeatureValues[10] = amplitudeCv;
    } else if (channel == CH_ACC_Y) {
      deployFeatureValues[2] = medianWindowField(channel, 5);
      deployFeatureValues[12] = medianWindowField(channel, 3);
    } else if (channel == CH_ACC_Z) {
      deployFeatureValues[4] = medianWindowField(channel, 5);
      deployFeatureValues[6] = medianWindowField(channel, 8);
      deployFeatureValues[11] = medianWindowField(channel, 7);
    } else if (channel == CH_GYRO_X) {
      deployFeatureValues[5] = freqStability;
      deployFeatureValues[14] = medianWindowField(channel, 5);
    } else if (channel == CH_GYRO_Y) {
      deployFeatureValues[3] = mad;
      deployFeatureValues[13] = medianWindowField(channel, 3);
    } else if (channel == CH_GYRO_Z) {
      deployFeatureValues[8] = mad;
    } else if (channel == CH_GYRO_MAG) {
      deployFeatureValues[9] = q3 - q1;
    }

    Serial.print("# SIXAXIS_FEATURE,task="); Serial.print(taskName);
    Serial.print(",channel="); Serial.print(FEATURE_CHANNEL_NAMES[channel]);
    Serial.print(",mean="); Serial.print(means[channel], 6);
    Serial.print(",std="); Serial.print(sqrtf(variances[channel]), 6);
    Serial.print(",median="); Serial.print(median, 6);
    Serial.print(",rms="); Serial.print(sqrtf(energies[channel]), 6);
    Serial.print(",mad="); Serial.print(mad, 6);
    Serial.print(",iqr="); Serial.print(q3-q1, 6);
    Serial.print(",peak_to_peak="); Serial.print(maximums[channel]-minimums[channel], 6);
    Serial.print(",energy="); Serial.print(energies[channel], 8);
    Serial.print(",dominant_frequency="); Serial.print(medianWindowField(channel, 0), 4);
    Serial.print(",power_3_7_log="); Serial.print(medianWindowField(channel, 1), 6);
    Serial.print(",power_2_12_log="); Serial.print(medianWindowField(channel, 2), 6);
    Serial.print(",ratio_3_7_to_2_12="); Serial.print(medianWindowField(channel, 3), 6);
    Serial.print(",ratio_3_7_to_7_12="); Serial.print(medianWindowField(channel, 4), 6);
    Serial.print(",spectral_centroid="); Serial.print(medianWindowField(channel, 5), 4);
    Serial.print(",spectral_spread="); Serial.print(medianWindowField(channel, 6), 4);
    Serial.print(",spectral_entropy="); Serial.print(medianWindowField(channel, 7), 6);
    Serial.print(",peak_prominence="); Serial.print(medianWindowField(channel, 8), 4);
    Serial.print(",frequency_stability="); Serial.print(freqStability, 4);
    Serial.print(",tremor_persistence="); Serial.print(sessWindows ? (float)persistenceCount/sessWindows : 0, 6);
    Serial.print(",tremor_power_variability="); Serial.print(powerCv, 6);
    Serial.print(",amplitude_variability="); Serial.print(amplitudeCv, 6);
    Serial.print(",windows="); Serial.println(sessWindows);
  }
  if (sessionTask == SESSION_LIFT_HOLD) runPadsResearchClassifier(deployFeatureValues, sessionQuality);
}

void accumulateSpectrum(float *buf) {
  for (int i = 0; i < FFT_SIZE; i++) { fReal[i] = buf[i]*hann[i]; fImag[i] = 0; }
  fft(fReal, fImag, FFT_SIZE);
  for (int i = 0; i < FFT_SIZE/2; i++) psd[i] += fReal[i]*fReal[i] + fImag[i]*fImag[i];
}
float windowRms() {
  double ss = 0;
  for (int i = 0; i < FFT_SIZE; i++) ss += (double)bufX[i]*bufX[i]+(double)bufY[i]*bufY[i]+(double)bufZ[i]*bufZ[i];
  return sqrtf((float)(ss/FFT_SIZE));
}
int peakBinFromSpectrum(const float *spectrum) {
  float binHz = (float)SAMPLE_HZ/FFT_SIZE;
  int lo = (int)ceilf(BAND_LO_HZ/binHz), hi = (int)floorf(BAND_HI_HZ/binHz);
  if (lo < 1) lo = 1; if (hi > FFT_SIZE/2-2) hi = FFT_SIZE/2-2;
  int peak = lo;
  for (int i = lo; i <= hi; i++) if (spectrum[i] > spectrum[peak]) peak = i;
  return peak;
}
float peakFreqFromSpectrum(const float *spectrum) {
  float binHz = (float)SAMPLE_HZ/FFT_SIZE;
  int peak = peakBinFromSpectrum(spectrum);
  float y0 = spectrum[peak-1], y1 = spectrum[peak], y2 = spectrum[peak+1];
  float denom = (y0-2*y1+y2), delta = (denom!=0)?0.5f*(y0-y2)/denom:0;
  return (peak+delta)*binHz;
}
float peakMagnitudeFromSpectrum(const float *spectrum) {
  int peak = peakBinFromSpectrum(spectrum);
  float hannSum = 0;
  for (int i = 0; i < FFT_SIZE; i++) hannSum += hann[i];
  return (hannSum > 0 && spectrum[peak] > 0)
       ? 2.0f * sqrtf(spectrum[peak]) / hannSum : 0;
}
float calcBandRatioFromSpectrum(const float *spectrum) {
  float binHz = (float)SAMPLE_HZ / FFT_SIZE;
  int analysisLo = (int)ceilf(BAND_LO_HZ / binHz);
  int analysisHi = (int)floorf(BAND_HI_HZ / binHz);
  int tremorLo = (int)ceilf(TREMOR_LO_HZ / binHz);
  int tremorHi = (int)floorf(TREMOR_HI_HZ / binHz);
  if (analysisLo < 1) analysisLo = 1;
  if (analysisHi > FFT_SIZE/2-1) analysisHi = FFT_SIZE/2-1;
  if (tremorLo < analysisLo) tremorLo = analysisLo;
  if (tremorHi > analysisHi) tremorHi = analysisHi;

  double analysisPower = 0, tremorPower = 0;
  for (int i = analysisLo; i <= analysisHi; i++) {
    analysisPower += spectrum[i];
    if (i >= tremorLo && i <= tremorHi) tremorPower += spectrum[i];
  }
  return analysisPower > 0 ? (float)(tremorPower / analysisPower) : 0;
}
float peakFreqFromPsd() { return peakFreqFromSpectrum(psd); }
float calcBandRatioFromPsd() { return calcBandRatioFromSpectrum(psd); }

// ---------------- neutral signal interpretation ----------------
// The signal level only describes acceleration RMS and maps to no clinical severity grade.
const char* classifySignalLevel(float rms) {
  if (rms >= ampStrong) return "HIGH";
  if (rms >= ampElevated) return "MEDIUM";
  if (rms >= ampDetect) return "LOW";
  return "BELOW_THRESHOLD";
}

// The motion pattern only describes this measurement's spectral features; it infers no disease or risk.
const char* classifyMotorPattern(bool inBand, float rms, float bandRatio) {
  if (rms < ampDetect) return "NO_CHARACTERISTIC_TREMOR";
  if (!inBand || bandRatio < BAND_RATIO_MIN) return "NON_RHYTHMIC_MOTION";
  return (rms >= ampStrong) ? "STRONG_RHYTHMIC_TREMOR" : "RHYTHMIC_TREMOR";
}

// ---------------- Measurement Quality ----------------
const uint16_t QUALITY_REASON_FLAGS[] = {
  QUALITY_REASON_SAMPLING_RATE, QUALITY_REASON_READ_FAILURES,
  QUALITY_REASON_CLIPPED, QUALITY_REASON_JITTER, QUALITY_REASON_WINDOWS,
  QUALITY_REASON_FFT_FREQ, QUALITY_REASON_FFT_MAG, QUALITY_REASON_FFT_RATIO
};
const char *QUALITY_REASON_TEXT[] = {
  "Sampling Rate Variation", "MPU Communication Errors",
  "Sensor Clipping", "High Sampling Jitter", "Incomplete FFT Windows",
  "FFT Frequency Variation", "FFT Magnitude Variation", "FFT Band Ratio Variation"
};
const int QUALITY_REASON_COUNT = sizeof(QUALITY_REASON_FLAGS) / sizeof(QUALITY_REASON_FLAGS[0]);

const char* qualityLevelText(MeasurementQuality quality) {
  if (quality == QUALITY_REPEAT_TEST) return "REPEAT TEST";
  if (quality == QUALITY_WARNING) return "WARNING";
  return "PASS";
}
const char* qualityOledText(MeasurementQuality quality) {
  return quality == QUALITY_REPEAT_TEST ? "REPEAT" : qualityLevelText(quality);
}
void addQualityIssue(MeasurementQuality &quality, uint16_t &reasons,
                     MeasurementQuality issueLevel, uint16_t reason) {
  if (issueLevel > quality) quality = issueLevel;
  reasons |= reason;
}
void printQualityReasonsInline(uint16_t reasons) {
  if (reasons == 0) {
    Serial.print("All Applicable Checks Passed");
    return;
  }
  bool first = true;
  for (int i = 0; i < QUALITY_REASON_COUNT; i++) {
    if ((reasons & QUALITY_REASON_FLAGS[i]) == 0) continue;
    if (!first) Serial.print('|');
    Serial.print(QUALITY_REASON_TEXT[i]);
    first = false;
  }
}
void printQualityReasonLines(uint16_t reasons) {
  Serial.println("# QUALITY REASONS:");
  if (reasons == 0) {
    Serial.println("# - All Applicable Checks Passed");
    return;
  }
  for (int i = 0; i < QUALITY_REASON_COUNT; i++) {
    if (reasons & QUALITY_REASON_FLAGS[i]) {
      Serial.print("# - "); Serial.println(QUALITY_REASON_TEXT[i]);
    }
  }
}
void evaluateMeasurementQuality(float rms, MeasurementQuality &quality, uint16_t &reasons) {
  quality = QUALITY_PASS;
  reasons = 0;
  qualityFftChecked = false;
  qualityFreqRangeHz = qualityMagnitudeCv = qualityBandRatioRange = 0;
  qualityPeakToRmsRatio = 0;

  int availableWindows = sessWindows;
  if (availableWindows > MEAS_WINDOWS) availableWindows = MEAS_WINDOWS;
  if (sessWindows != MEAS_WINDOWS) {
    addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_WINDOWS);
  }

  bool fsWarning = false, fsRepeat = false;
  for (int i = 0; i < availableWindows; i++) {
    float fs = qualityWindowFs[i];
    if (!isfinite(fs) || fs < QUALITY_FS_WARN_MIN_HZ || fs > QUALITY_FS_WARN_MAX_HZ) fsRepeat = true;
    else if (fs < QUALITY_FS_PASS_MIN_HZ || fs > QUALITY_FS_PASS_MAX_HZ) fsWarning = true;
  }
  if (fsRepeat) addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_SAMPLING_RATE);
  else if (fsWarning) addQualityIssue(quality, reasons, QUALITY_WARNING, QUALITY_REASON_SAMPLING_RATE);

  if (sessionReadFailures > 2) {
    addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_READ_FAILURES);
  } else if (sessionReadFailures > 0) {
    addQualityIssue(quality, reasons, QUALITY_WARNING, QUALITY_REASON_READ_FAILURES);
  }

  if (sessionClippedSamples > 5) {
    addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_CLIPPED);
  } else if (sessionClippedSamples > 0) {
    addQualityIssue(quality, reasons, QUALITY_WARNING, QUALITY_REASON_CLIPPED);
  }

  if (maxJitterUs > QUALITY_JITTER_WARN_MAX_US) {
    addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_JITTER);
  } else if (maxJitterUs > QUALITY_JITTER_PASS_MAX_US) {
    addQualityIssue(quality, reasons, QUALITY_WARNING, QUALITY_REASON_JITTER);
  }

  if (sessWindows != MEAS_WINDOWS) return;
  float freqMin = qualityWindowFreq[0], freqMax = qualityWindowFreq[0];
  float ratioMin = qualityWindowBandRatio[0], ratioMax = qualityWindowBandRatio[0];
  float magnitudeMean = 0;
  bool freqValid = true, magnitudeValid = true, ratioValid = true;
  for (int i = 0; i < MEAS_WINDOWS; i++) {
    float freq = qualityWindowFreq[i];
    float magnitude = qualityWindowMagnitude[i];
    float ratio = qualityWindowBandRatio[i];
    if (!isfinite(freq)) freqValid = false;
    else { if (freq < freqMin) freqMin = freq; if (freq > freqMax) freqMax = freq; }
    if (!isfinite(magnitude) || magnitude < 0) magnitudeValid = false;
    else magnitudeMean += magnitude;
    if (!isfinite(ratio)) ratioValid = false;
    else { if (ratio < ratioMin) ratioMin = ratio; if (ratio > ratioMax) ratioMax = ratio; }
  }

  magnitudeMean /= MEAS_WINDOWS;
  // When quiet or with no clear periodic peak, the FFT max peak is just noise and its drift should not force a re-measure.
  if (rms < QUALITY_FFT_MIN_RMS_G) return;
  if (magnitudeValid && magnitudeMean > 0.000001f) {
    qualityPeakToRmsRatio = magnitudeMean / rms;
    if (qualityPeakToRmsRatio < QUALITY_FFT_MIN_PEAK_RMS_RATIO) return;
  }
  qualityFftChecked = true;

  if (!freqValid) {
    addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_FFT_FREQ);
  } else {
    qualityFreqRangeHz = freqMax - freqMin;
    if (qualityFreqRangeHz > QUALITY_FREQ_WARN_RANGE_HZ)
      addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_FFT_FREQ);
    else if (qualityFreqRangeHz > QUALITY_FREQ_PASS_RANGE_HZ)
      addQualityIssue(quality, reasons, QUALITY_WARNING, QUALITY_REASON_FFT_FREQ);
  }

  if (!magnitudeValid || magnitudeMean <= 0.000001f) {
    addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_FFT_MAG);
  } else {
    float magnitudeVariance = 0;
    for (int i = 0; i < MEAS_WINDOWS; i++) {
      float delta = qualityWindowMagnitude[i] - magnitudeMean;
      magnitudeVariance += delta * delta;
    }
    qualityMagnitudeCv = sqrtf(magnitudeVariance / MEAS_WINDOWS) / magnitudeMean;
    if (qualityMagnitudeCv > QUALITY_MAG_WARN_CV)
      addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_FFT_MAG);
    else if (qualityMagnitudeCv > QUALITY_MAG_PASS_CV)
      addQualityIssue(quality, reasons, QUALITY_WARNING, QUALITY_REASON_FFT_MAG);
  }

  if (!ratioValid) {
    addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_FFT_RATIO);
  } else {
    qualityBandRatioRange = ratioMax - ratioMin;
    if (qualityBandRatioRange > QUALITY_RATIO_WARN_RANGE)
      addQualityIssue(quality, reasons, QUALITY_REPEAT_TEST, QUALITY_REASON_FFT_RATIO);
    else if (qualityBandRatioRange > QUALITY_RATIO_PASS_RANGE)
      addQualityIssue(quality, reasons, QUALITY_WARNING, QUALITY_REASON_FFT_RATIO);
  }
}

// ---------------- OLED ----------------
void drawReady(uint8_t prox) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2); display.setCursor(0,0);  display.println("READY");
  display.setTextSize(1); display.setCursor(0,20);
  display.print("Mode: ");
  display.println(selMode==MODE_GUIDED ? "Guided test" : (selMode==MODE_TAP ? "Tap test" : (selMode==MODE_MONITOR ? "Monitor" : "Tremor")));
  display.setCursor(0,32);
  display.print("swipe mode / hold "); display.print(MODE_START_HOLD_MS / 1000.0f, 1); display.println("s");
  display.setCursor(0,42);
  if (handNear) {
    uint32_t heldMs = millis() - holdStartMs;
    if (heldMs > MODE_START_HOLD_MS) heldMs = MODE_START_HOLD_MS;
    display.print("hold:"); display.print(heldMs / 1000.0f, 1);
    display.print("/"); display.print(MODE_START_HOLD_MS / 1000.0f, 1); display.print("s");
  } else {
    display.print("hold:--");
  }
  display.setCursor(0,54); display.print("FIFO IR:"); display.print(prox);
  display.print("/"); display.print(FIFO_HOLD_ON_THRESH); display.display();
}
bool handleGestureModeSelection() {
  int gesture = apdsOk ? apdsReadGesture() : GESTURE_NONE;
  if (gesture == GESTURE_NONE) return false;
  uint32_t now = millis();
  noteReadyMotion(now);                                      // do not accumulate start-dwell time while waving

  if (lastGestureSwitchMs != 0 && now - lastGestureSwitchMs < GESTURE_REPEAT_MS) {
    Serial.println("# GESTURE cooldown");
    return false;
  }

  // Any gesture = cycle to the next mode (Tremor -> Tap -> Monitor -> Guided -> Tremor...), no need to tell direction
  if (gesture == GESTURE_LEFT || gesture == GESTURE_RIGHT || gesture == GESTURE_UP || gesture == GESTURE_DOWN)
    selMode = (selMode + 1) % 4;
  else return false;

  lastGestureSwitchMs = now;
  Serial.print("# GESTURE=");
  Serial.print(gesture==GESTURE_LEFT ? "LEFT" : (gesture==GESTURE_RIGHT ? "RIGHT" : (gesture==GESTURE_UP ? "UP" : "DOWN")));
  Serial.print(",MODE=");
  Serial.println(selMode==MODE_GUIDED ? "GUIDED" : (selMode==MODE_TAP ? "TAP" : (selMode==MODE_MONITOR ? "MONITOR" : "TREMOR")));
  // ==== WIFI/WS ADDON: once a mode is picked, tell the web page to switch to that page to get ready ====
  webSocket.broadcastTXT(String("{\"type\":\"mode\",\"mode\":\"") +
    (selMode==MODE_GUIDED ? "guided" : selMode==MODE_TAP ? "tap" : selMode==MODE_MONITOR ? "monitor" : "tremor") + "\"}");
  drawReady(lastProx);
  return true;
}
void drawSettle() {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  if (liftHoldActive) {
    display.setTextSize(1); display.setCursor(0,0); display.println("LIFT+HOLD ML");
    display.setTextSize(2); display.setCursor(0,14); display.println("ARM DOWN");
    display.setTextSize(1); display.setCursor(0,42); display.println("Raise at next beep");
    display.setCursor(0,54); display.println("calibrating gyro...");
    display.display();
    return;
  }
  if (guidedRestActive) {
    display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED TEST");
    display.setTextSize(2); display.setCursor(0,14); display.println("1/3 REST");
    display.setTextSize(1); display.setCursor(0,42); display.println("Keep hand relaxed");
    display.setCursor(0,54); display.println("settling...");
    display.display();
    return;
  }
  display.setTextSize(2); display.setCursor(0,8);  display.println("Get set");
  display.setTextSize(1); display.setCursor(0,40); display.println("hold still..."); display.display();
}
void drawMeasuringLive(int done, int total, float freq, float rmsMg) {
  if (!oledOk) return;
  float secs = sessSamples / (float)SAMPLE_HZ;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0);
  if (guidedRestActive) display.print("1/3 REST ");
  else if (liftHoldActive) display.print("LIFT+HOLD ");
  else display.print("Measuring ");
  display.print(secs,1); display.print("s");
  display.setTextSize(2); display.setCursor(0,12); display.print(freq,1); display.print(" Hz");
  display.setTextSize(1); display.setCursor(0,33); display.print("Amp:"); display.print((int)(rmsMg+0.5f)); display.print("mg");
  int barW = (int)(120.0f*done/total);
  display.drawRect(0,46,122,12,SSD1306_WHITE); display.fillRect(1,47,barW,10,SSD1306_WHITE); display.display();
}
void drawProgressCal(const char *title, int done, int total, const char *hint) {
  if (!oledOk) return;
  float secs = done * (float)FFT_SIZE / SAMPLE_HZ;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2); display.setCursor(0,0);  display.println(title);
  display.setTextSize(1); display.setCursor(0,22); display.print(secs,1); display.print("s");
  int barW = (int)(120.0f*done/total);
  display.drawRect(0,36,122,12,SSD1306_WHITE); display.fillRect(1,37,barW,10,SSD1306_WHITE);
  display.setCursor(0,54); display.print(hint); display.display();
}
void drawResult() {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0);
  if (computeSource == COMPUTE_SOURCE_XIAO) display.print("XIAO RESULT");
  else if (computeSource == COMPUTE_SOURCE_LOCAL_FALLBACK) display.print("LOCAL FALLBACK");
  else display.print("LOCAL SHADOW");
  display.setCursor(0,10);
  display.print(rFreq,1); display.print("Hz "); display.print((int)(rRmsMg+0.5f)); display.print("mg");
  if (rQuality == QUALITY_REPEAT_TEST) {
    display.setCursor(0,22); display.println("MEASUREMENT INVALID");
    display.setCursor(0,34); display.println("classification suppressed");
    display.setTextSize(2); display.setCursor(0,48); display.print("REPEAT");
    display.display();
    return;
  }
  if (sessionTask == SESSION_LIFT_HOLD && rPadsResearchValid) {
    display.setCursor(0,20); display.println("PD score");
    display.setTextSize(2); display.setCursor(0,30); display.print(rPadsResearchProbability, 3);
    display.setTextSize(1); display.setCursor(0,43); display.print("threshold:"); display.print(PADS_MODEL_THRESHOLD, 3);
    display.setCursor(0,55); display.println("NOT A DIAGNOSIS");
    display.display();
    return;
  }
  display.setCursor(0,20); display.print("level:"); display.print(rSignalLevel);
  display.setCursor(0,30); display.print("PAT:");
  if      (strcmp(rMotorPattern, "NO_CHARACTERISTIC_TREMOR") == 0) display.print("NO CHAR");
  else if (strcmp(rMotorPattern, "NON_RHYTHMIC_MOTION") == 0) display.print("NON-RHYTHM");
  else if (strcmp(rMotorPattern, "STRONG_RHYTHMIC_TREMOR") == 0) display.print("STRONG RHYTHM");
  else display.print("RHYTHMIC");
  display.setCursor(0,40); display.print("QUALITY:");
  display.setTextSize(2); display.setCursor(0,48); display.print(qualityOledText(rQuality));
  display.display();
}

void drawXiaoProcessing() {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("XIAO PROCESSING");
  display.setTextSize(2); display.setCursor(0,18); display.println("WAIT...");
  display.setTextSize(1); display.setCursor(0,48); display.println("timeout -> local");
  display.setCursor(0,57); display.println("fallback automatically");
  display.display();
}
void drawGuidedRestComplete(MeasurementQuality quality) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED TEST");
  display.setTextSize(2); display.setCursor(0,12); display.println("REST DONE");
  display.setTextSize(1); display.setCursor(0,38); display.print("Quality: "); display.println(qualityLevelText(quality));
  display.setCursor(0,52); display.println("Next: 2/3 TAP");
  display.display();
}

void saveResultOled() {
  resultOledSaved = false;
  if (!oledOk) return;
  memcpy(resultOledBuffer, display.getBuffer(), sizeof(resultOledBuffer));
  resultOledSaved = true;
}
void restoreResultOled() {
  if (!oledOk || !resultOledSaved) return;
  memcpy(display.getBuffer(), resultOledBuffer, sizeof(resultOledBuffer));
  display.display();
}
void drawResultExit(uint32_t heldMs) {
  if (!oledOk) return;
  if (heldMs > MONITOR_EXIT_HOLD_MS) heldMs = MONITOR_EXIT_HOLD_MS;
  int barW = (int)(120.0f * heldMs / MONITOR_EXIT_HOLD_MS);
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("EXIT RESULT");
  display.setTextSize(1); display.setCursor(0,22); display.println("keep hand close");
  display.setCursor(0,34); display.print(heldMs / 1000.0f, 1); display.print("/");
  display.print(MONITOR_EXIT_HOLD_MS / 1000.0f, 1); display.print("s P:"); display.print(resultExitProx);
  display.drawRect(0,50,122,12,SSD1306_WHITE); display.fillRect(1,51,barW,10,SSD1306_WHITE);
  display.display();
}
void enterPersistentResult() {
  apdsSetTapProximityProfile(false);
  resultExitArmed = false;
  resultExitAwayStartMs = 0;
  resultExitHoldStartMs = 0;
  resultExitLastPollMs = 0;
  resultExitLastDrawMs = 0;
  resultExitProx = 0;
  lastPollMs = 0;
  saveResultOled();
  state = ST_RESULT;
  Serial.print("# RESULT stays on screen — move hand away, then hold close ");
  Serial.print(MONITOR_EXIT_HOLD_MS / 1000.0f, 1); Serial.println("s to return READY");
}

// ---------------- session ----------------
void startSession() {                 // → SETTLE → MEASURING
  state = ST_SETTLE; settleStartMs = millis();
  gravInit = false; bufIdx = 0; mpuFailStreak = 0;
  gyroBiasX = gyroBiasY = gyroBiasZ = 0;
  gyroBiasSumX = gyroBiasSumY = gyroBiasSumZ = 0;
  gyroBiasSamples = 0;
  rPadsResearchProbability = NAN;
  rPadsResearchValid = false;
  Serial.println("# SESSION START");
  if (streamRaw) Serial.println("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps");
  drawSettle();
  if (!liftHoldActive) beep(1, 50, 0); // for LiftHold, prompt to raise the hand only after gyro calibration finishes
  maxJitterUs = 0; nextSampleUs = micros(); windowStartUs = micros();
}
void beginMeasuring() {
  state = ST_MEASURING;
  for (int i = 0; i < FFT_SIZE/2; i++) psd[i] = 0;
  sessSumSq = gyroSumSq = jerkSumSq = 0; peakMag = 0; zcrCount = 0;
  prevLxValid = false; prevSign = 0;
  sessSamples = 0; sessWindows = 0; bufIdx = 0;
  analysisWindowPrimed = false;
  sessionReadFailures = 0; sessionClippedSamples = 0;
  for (int i = 0; i < MEAS_WINDOWS; i++) {
    qualityWindowFreq[i] = qualityWindowMagnitude[i] = 0;
    qualityWindowBandRatio[i] = qualityWindowFs[i] = 0;
  }
  qualityFreqRangeHz = qualityMagnitudeCv = qualityBandRatioRange = 0;
  qualityPeakToRmsRatio = 0;
  qualityFftChecked = false;
  beginXiaoCompute(xiaoModeForMeasurement(), MEAS_TOTAL_SAMPLES);
  drawMeasuringLive(0, MEAS_WINDOWS, 0, 0);
  maxJitterUs = 0; sessionStartUs = micros(); windowStartUs = micros(); nextSampleUs = micros();
}
void finalizeSession() {
  xiaoAwaitSessionId = endXiaoCompute(effectiveFs);
  xiaoAwaitingResult = xiaoAwaitSessionId != 0;
  xiaoResultDeadlineMs = millis() + XIAO_RESULT_TIMEOUT_MS;
  computeSource = xiaoAwaitingResult ? COMPUTE_SOURCE_LOCAL : COMPUTE_SOURCE_LOCAL_FALLBACK;
  float peakFreq = peakFreqFromPsd();
  float bandRatio = calcBandRatioFromPsd();
  prepareStoredSignalsForResearchFeatures();
  computeAllSessionWindowFeatures();

  float rms   = (sessSamples>0) ? sqrtf((float)(sessSumSq/sessSamples)) : 0;   // amplitude RMS (g)
  float gyro  = (sessSamples>0) ? sqrtf((float)(gyroSumSq/sessSamples)) : 0;   // stability: angular-velocity RMS (dps)
  float jerk  = (sessSamples>1) ? sqrtf((float)(jerkSumSq/(sessSamples-1))) : 0; // smoothness: jerk RMS (g/s)
  float durS  = (sessSamples>0) ? (float)sessSamples/SAMPLE_HZ : 1;
  float zcrHz = zcrCount / durS;                                               // zero-crossing rate (Hz)
  MeasurementQuality quality;
  uint16_t qualityReasons;
  evaluateMeasurementQuality(rms, quality, qualityReasons);
  bool inBand = (peakFreq >= TREMOR_LO_HZ && peakFreq <= TREMOR_HI_HZ);
  bool classificationSuppressed = quality == QUALITY_REPEAT_TEST;
  const char *signalLevel = classificationSuppressed ? "INVALID" : classifySignalLevel(rms);
  const char *motorPattern = classificationSuppressed
                           ? "RESULT_SUPPRESSED"
                           : classifyMotorPattern(inBand, rms, bandRatio);
  uint16_t ambient = apdsOk ? apdsClear() : 0;                                 // ambient light

  Serial.print("# RESULT,peak_freq_Hz="); Serial.print(peakFreq,2);
  Serial.print(",rms_mg=");   Serial.print(rms*1000.0f,1);
  Serial.print(",band_ratio="); Serial.print(bandRatio,2);
  Serial.print(",in_tremor_band="); Serial.print(inBand?1:0);
  Serial.print(",signal_level="); Serial.print(signalLevel);
  Serial.print(",motor_pattern="); Serial.print(motorPattern);
  Serial.print(",classification_suppressed="); Serial.print(classificationSuppressed ? 1 : 0);
  Serial.print(",gyro_rms_dps="); Serial.print(gyro,1);      // stability
  Serial.print(",jerk_rms_gps="); Serial.print(jerk,2);      // smoothness
  Serial.print(",peak_g=");   Serial.print(peakMag,3);
  Serial.print(",zcr_hz=");   Serial.print(zcrHz,1);
  Serial.print(",ambient=");  Serial.print(ambient);
  Serial.print(",windows=");  Serial.print(sessWindows);
  Serial.print(",fs_Hz=");    Serial.print(effectiveFs,1);
  Serial.print(",read_failures="); Serial.print(sessionReadFailures);
  Serial.print(",clipped_samples="); Serial.print(sessionClippedSamples);
  Serial.print(",max_jitter_us="); Serial.print(maxJitterUs);
  Serial.print(",quality="); Serial.print(qualityLevelText(quality));
  Serial.print(",quality_reason="); printQualityReasonsInline(qualityReasons); Serial.println();
  for (int i = 0; i < sessWindows && i < MEAS_WINDOWS; i++) {
    Serial.print("# QUALITY_WINDOW,index="); Serial.print(i + 1);
    Serial.print(",fs_Hz="); Serial.print(qualityWindowFs[i], 2);
    Serial.print(",peak_freq_Hz="); Serial.print(qualityWindowFreq[i], 2);
    Serial.print(",peak_magnitude_g="); Serial.print(qualityWindowMagnitude[i], 5);
    Serial.print(",band_ratio="); Serial.println(qualityWindowBandRatio[i], 3);
  }
  Serial.print("# QUALITY_METRICS,fft_consistency=");
  if (qualityFftChecked) {
    Serial.print("CHECKED,freq_range_Hz="); Serial.print(qualityFreqRangeHz, 2);
    Serial.print(",magnitude_cv="); Serial.print(qualityMagnitudeCv, 3);
    Serial.print(",band_ratio_range="); Serial.println(qualityBandRatioRange, 3);
  } else if (sessWindows != MEAS_WINDOWS) {
    Serial.println("SKIPPED_INCOMPLETE_WINDOWS");
  } else if (rms < QUALITY_FFT_MIN_RMS_G) {
    Serial.println("SKIPPED_LOW_SIGNAL");
  } else {
    Serial.print("SKIPPED_NO_DOMINANT_PEAK,peak_to_rms_ratio=");
    Serial.println(qualityPeakToRmsRatio, 3);
  }
  Serial.print("# QUALITY "); Serial.println(qualityLevelText(quality));
  printQualityReasonLines(qualityReasons);
  // The eight-channel full features only describe the measured signal; the firmware makes no direct PD/HC diagnosis.
  printSixAxisFeatureSummary(quality);
  Serial.println("# SESSION END");

  rFreq=peakFreq; rRmsMg=rms*1000.0f; rBandRatio=bandRatio; rInBand=inBand;
  rSignalLevel=signalLevel; rMotorPattern=motorPattern;
  rGyro=gyro; rJerk=jerk; rPeak=peakMag; rZcr=zcrHz; rAmbient=ambient;
  rQuality=quality; rQualityReasons=qualityReasons;
  broadcastCurrentResult();
  if (!guidedRestActive) broadcastSessionWaveform();   // send the full waveform once after measurement (replaces the live stream during measurement)
  if (guidedRestActive) {
    Serial.print("# GUIDED TEST 1/3 REST complete,quality=");
    Serial.println(qualityLevelText(quality));
    // Guided exam: continue even if quality judges REPEAT TEST (quality is already stored in rQuality,
    // the final guidedOverallQuality reflects it; OLED/serial still show quality).
    // Purpose: keep the whole clinic guided-exam flow from getting stuck at the first step.
    guidedRestActive = false;
    guidedRestResultStartMs = millis();
    state = ST_GUIDED_REST_DONE;
    drawGuidedRestComplete(quality);
    beep(1, 100, 0);
    return;
  }
  if (xiaoAwaitingResult) drawXiaoProcessing();
  else drawResult();
  guidedRestActive = false;                    // standalone Tremor stays on the result page
  liftHoldActive = false;                      // LiftHold result already stored in the OLED buffer / serial output
  enterPersistentResult();
  if      (classificationSuppressed) beep(1, 400, 0);                          // invalid measurement: one long beep
  else if (strcmp(motorPattern, "STRONG_RHYTHMIC_TREMOR") == 0) beep(3, 120, 100);
  else if (strcmp(motorPattern, "RHYTHMIC_TREMOR") == 0) beep(2, 120, 120);
  else beep(1, 150, 0);
}

// ---------------- calibration ----------------
bool validCalibrationThresholds(float detect, float elevated, float strong) {
  return isfinite(detect) && isfinite(elevated) && isfinite(strong) &&
         detect > 0.0f && detect < elevated && elevated < strong &&
         strong <= CAL_MAX_THRESHOLD_G;
}

uint32_t calibrationRecordChecksum(const CalibrationRecord &record) {
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&record);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < sizeof(CalibrationRecord) - sizeof(record.checksum); i++) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool loadCalibrationFromNvs() {
  Preferences prefs;
  if (!prefs.begin(CAL_NVS_NAMESPACE, true)) {
    Serial.println("# CAL NVS WARN - cannot open NVS, using built-in default thresholds");
    return false;
  }

  CalibrationRecord record = {};
  size_t recordSize = prefs.getBytesLength(CAL_NVS_KEY);
  size_t bytesRead = (recordSize == sizeof(record))
                   ? prefs.getBytes(CAL_NVS_KEY, &record, sizeof(record)) : 0;
  prefs.end();

  if (recordSize == 0) {
    Serial.println("# CAL NVS - no saved data, using built-in default thresholds");
    return false;
  }
  bool recordValid = bytesRead == sizeof(record) &&
                     record.magic == CAL_NVS_MAGIC &&
                     record.version == CAL_NVS_VERSION &&
                     record.checksum == calibrationRecordChecksum(record) &&
                     validCalibrationThresholds(record.detect, record.elevated, record.strong);
  if (!recordValid) {
    Serial.println("# CAL NVS WARN - saved-data version mismatch or corruption, using built-in default thresholds");
    return false;
  }

  ampDetect = record.detect;
  ampElevated = record.elevated;
  ampStrong = record.strong;
  Serial.print("# CAL NVS LOADED — detect="); Serial.print(ampDetect, 4);
  Serial.print("g elevated="); Serial.print(ampElevated, 4);
  Serial.print("g strong="); Serial.print(ampStrong, 4); Serial.println("g");
  return true;
}

bool saveCalibrationToNvs(float detect, float elevated, float strong) {
  CalibrationRecord record = {
    CAL_NVS_MAGIC, CAL_NVS_VERSION, detect, elevated, strong, 0
  };
  record.checksum = calibrationRecordChecksum(record);

  Preferences prefs;
  if (!prefs.begin(CAL_NVS_NAMESPACE, false)) return false;
  bool ok = prefs.putBytes(CAL_NVS_KEY, &record, sizeof(record)) == sizeof(record);
  prefs.end();
  return ok;
}

void showCalibrationFailure(const char *serialReason, const char *oledReason) {
  Serial.print("# CAL FAILED — "); Serial.println(serialReason);
  Serial.println("# CAL RETAINED - kept the previous valid thresholds, not written to NVS");
  if (oledOk) {
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2); display.setCursor(0,0); display.println("CAL failed");
    display.setTextSize(1); display.setCursor(0,25); display.println(oledReason);
    display.setCursor(0,45); display.println("old values kept");
    display.display();
  }
  beep(3, 80, 80);
  state = ST_CAL_RESULT;
  calResultStartMs = millis();
}

void clearStoredCalibration() {
  Preferences prefs;
  bool cleared = false;
  if (prefs.begin(CAL_NVS_NAMESPACE, false)) {
    cleared = prefs.clear();
    prefs.end();
  }
  if (!cleared) {
    showCalibrationFailure("NVS clear failed", "NVS clear error");
    return;
  }

  ampDetect = AMP_DETECT_DEFAULT;
  ampElevated = AMP_ELEVATED_DEFAULT;
  ampStrong = AMP_STRONG_DEFAULT;
  Serial.println("# CAL NVS CLEARED - restored built-in default thresholds");
  Serial.print("# DEFAULT — detect="); Serial.print(ampDetect, 4);
  Serial.print("g elevated="); Serial.print(ampElevated, 4);
  Serial.print("g strong="); Serial.print(ampStrong, 4); Serial.println("g");
  if (oledOk) {
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2); display.setCursor(0,0); display.println("CAL reset");
    display.setTextSize(1); display.setCursor(0,25); display.println("NVS cleared");
    display.setCursor(0,43); display.println("defaults active");
    display.display();
  }
  beep(1, 120, 0);
  state = ST_CAL_RESULT;
  calResultStartMs = millis();
}

void startCalibration() {
  Serial.println("# CAL START - rest ~15s (place it down and keep still)");
  state = ST_CAL_REST; calIdx = 0; gravInit = false; bufIdx = 0; mpuFailStreak = 0;
  analysisWindowPrimed = false;
  calReadFailures = 0; calClippedSamples = 0;
  beginXiaoCompute(TremorLink::MODE_CAL_REST, CAL_WINDOWS * FFT_SIZE);
  drawProgressCal("CAL rest", 0, CAL_WINDOWS, "keep STILL");
  maxJitterUs = 0; nextSampleUs = micros(); windowStartUs = micros();
}
void sortAsc(float *a, int n) {
  for (int i = 1; i < n; i++) { float k = a[i]; int j = i-1; while (j>=0 && a[j]>k) { a[j+1]=a[j]; j--; } a[j+1]=k; }
}
void finishCalibration() {
  endXiaoCompute(effectiveFs);
  for (int i = 0; i < CAL_WINDOWS; i++) {
    if (!isfinite(calRest[i]) || !isfinite(calTremor[i])) {
      showCalibrationFailure("samples contain invalid values", "invalid samples");
      return;
    }
  }
  sortAsc(calRest, CAL_WINDOWS); sortAsc(calTremor, CAL_WINDOWS);
  float restMax = calRest[CAL_WINDOWS-1], restMed = calRest[CAL_WINDOWS/2];
  float trMin = calTremor[0], trMed = calTremor[CAL_WINDOWS/2], trMax = calTremor[CAL_WINDOWS-1];
  Serial.println("# CAL CHECK - stats (g):");
  Serial.print("#   rest  median="); Serial.print(restMed,4); Serial.print(" max="); Serial.println(restMax,4);
  Serial.print("#   tremor min="); Serial.print(trMin,4); Serial.print(" median="); Serial.print(trMed,4); Serial.print(" max="); Serial.println(trMax,4);
  Serial.print("#   read_failures="); Serial.print(calReadFailures);
  Serial.print(" clipped_samples="); Serial.println(calClippedSamples);

  if (calReadFailures > CAL_MAX_READ_FAILURES) {
    showCalibrationFailure("too many MPU read failures", "sensor read error");
    return;
  }
  if (calClippedSamples > CAL_MAX_CLIPPED_SAMPLES) {
    showCalibrationFailure("too many saturated accel samples", "sensor clipped");
    return;
  }
  if (restMax > CAL_REST_MAX_G) {
    showCalibrationFailure("too much motion during rest, place it steady and retry", "rest moved");
    return;
  }
  if (trMed < CAL_TREMOR_MIN_G) {
    showCalibrationFailure("shake amplitude too weak, shake as shown", "shake too weak");
    return;
  }
  if (trMed < restMax * CAL_MIN_SEPARATION) {
    showCalibrationFailure("rest and shake data overlap, cannot separate reliably", "data overlap");
    return;
  }

  float detect = restMax*1.5f; if (detect < 0.010f) detect = 0.010f;
  float strong = trMed;
  float elevated = sqrtf(detect*strong);
  if (!validCalibrationThresholds(detect, elevated, strong)) {
    showCalibrationFailure("computed thresholds are unreasonable", "bad thresholds");
    return;
  }
  if (!saveCalibrationToNvs(detect, elevated, strong)) {
    showCalibrationFailure("NVS save failed", "NVS save error");
    return;
  }

  ampDetect = detect;
  ampElevated = elevated;
  ampStrong = strong;
  Serial.println("# CAL SAVED - applied and persisted to NVS:");
  Serial.print("ampDetect = "); Serial.print(ampDetect,3); Serial.println("f;");
  Serial.print("ampElevated = "); Serial.print(ampElevated,3); Serial.println("f;");
  Serial.print("ampStrong = "); Serial.print(ampStrong,3); Serial.println("f;");
  if (oledOk) {
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2); display.setCursor(0,0); display.println("CAL done");
    display.setTextSize(1); display.setCursor(0,24);
    display.print("det "); display.print((int)(detect*1000)); display.println("mg");
    display.print("mid "); display.print((int)(elevated*1000)); display.println("mg");
    display.print("high "); display.print((int)(strong*1000)); display.println("mg"); display.display();
  }
  state = ST_CAL_RESULT;
  calResultStartMs = millis();
}

// ---------------- continuous monitoring ----------------
void drawMonitor(float freq, float rmsMg, bool active, int episodes, uint32_t elapsedS) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  if (guidedPostureActive) {
    display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED 3/3 HOLD");
    display.setTextSize(2); display.setCursor(0,12); display.print(freq,1); display.print("Hz");
    display.setTextSize(1); display.setCursor(0,34); display.print("amp:"); display.print((int)(rmsMg+0.5f)); display.println("mg");
    display.setCursor(0,46); display.print("window:"); display.print(guidedPostureWindows); display.print("/"); display.print(GUIDED_POSTURE_WINDOWS);
    display.setCursor(0,56); display.print("target:"); display.print(guidedPostureTremorWindows);
    display.display();
    return;
  }
  display.setTextSize(1); display.setCursor(0,0); display.print("MONITOR "); display.print(elapsedS); display.print("s");
  display.setTextSize(2); display.setCursor(0,12); display.print(freq,1); display.print("Hz");
  display.setTextSize(1); display.setCursor(0,33); display.print("amp:"); display.print((int)(rmsMg+0.5f)); display.print("mg");
  display.setTextSize(2); display.setCursor(0,44); display.print(active ? "TREMOR" : "clear");
  display.setTextSize(1); display.setCursor(92,50); display.print("e:"); display.print(episodes);
  display.display();
}
void drawMonitorExit(uint32_t heldMs) {
  if (!oledOk) return;
  if (heldMs > MONITOR_EXIT_HOLD_MS) heldMs = MONITOR_EXIT_HOLD_MS;
  int barW = (int)(120.0f * heldMs / MONITOR_EXIT_HOLD_MS);
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2); display.setCursor(0,0); display.println("EXIT MON");
  display.setTextSize(1); display.setCursor(0,22); display.println("keep hand close");
  display.setCursor(0,34); display.print(heldMs / 1000.0f, 1); display.print("/");
  display.print(MONITOR_EXIT_HOLD_MS / 1000.0f, 1); display.print("s P:"); display.print(monExitProx);
  display.drawRect(0,50,122,12,SSD1306_WHITE); display.fillRect(1,51,barW,10,SSD1306_WHITE);
  display.display();
}
void startMonitor() {
  apdsSetTapProximityProfile(false);
  if (guidedPostureActive) {
    Serial.println("# MONITOR START — guided fixed-duration posture capture");
  } else {
    Serial.print("# MONITOR START - move the hand away first, then approach the sensor ");
    Serial.print(MONITOR_EXIT_HOLD_MS / 1000.0f, 1); Serial.println(" s to exit; you can also send o to leave");
  }
  state = ST_MONITOR;
  for (int i=0;i<FFT_SIZE/2;i++) psd[i]=0;
  monWinSumSq = 0; monEpisodes = 0; monTremorStreak = 0; monActive = false;
  monExitArmed = false;
  monExitAwayStartMs = 0;
  monExitHoldStartMs = 0;
  monExitLastPollMs = 0;
  monExitLastDrawMs = 0;
  monExitProx = 0;
  monLastFreq = 0;
  monLastRmsMg = 0;
  bufIdx = 0; gravInit = false;
  analysisWindowPrimed = false;
  beginXiaoCompute(guidedPostureActive ? TremorLink::MODE_GUIDED_POSTURE
                                       : TremorLink::MODE_MONITOR_WINDOW,
                   guidedPostureActive ? MEAS_TOTAL_SAMPLES : FFT_SIZE);
  drawMonitor(0, 0, false, 0, 0);
  monStartMs = millis();
  maxJitterUs = 0;
  windowStartUs = micros();
  nextSampleUs = windowStartUs;
}

void evaluateGuidedPostureQuality() {
  guidedPostureQuality = QUALITY_PASS;
  guidedPostureQualityReasons = 0;
  if (guidedPostureWindows != GUIDED_POSTURE_WINDOWS) {
    addQualityIssue(guidedPostureQuality, guidedPostureQualityReasons,
                    QUALITY_REPEAT_TEST, QUALITY_REASON_WINDOWS);
  }

  bool fsWarning = false, fsRepeat = false;
  int availableWindows = guidedPostureWindows;
  if (availableWindows > GUIDED_POSTURE_WINDOWS) availableWindows = GUIDED_POSTURE_WINDOWS;
  for (int i = 0; i < availableWindows; i++) {
    float fs = guidedPostureWindowFs[i];
    if (!isfinite(fs) || fs < QUALITY_FS_WARN_MIN_HZ || fs > QUALITY_FS_WARN_MAX_HZ) fsRepeat = true;
    else if (fs < QUALITY_FS_PASS_MIN_HZ || fs > QUALITY_FS_PASS_MAX_HZ) fsWarning = true;
  }
  if (fsRepeat) addQualityIssue(guidedPostureQuality, guidedPostureQualityReasons,
                                QUALITY_REPEAT_TEST, QUALITY_REASON_SAMPLING_RATE);
  else if (fsWarning) addQualityIssue(guidedPostureQuality, guidedPostureQualityReasons,
                                      QUALITY_WARNING, QUALITY_REASON_SAMPLING_RATE);

  if (guidedPostureReadFailures > 2) {
    addQualityIssue(guidedPostureQuality, guidedPostureQualityReasons,
                    QUALITY_REPEAT_TEST, QUALITY_REASON_READ_FAILURES);
  } else if (guidedPostureReadFailures > 0) {
    addQualityIssue(guidedPostureQuality, guidedPostureQualityReasons,
                    QUALITY_WARNING, QUALITY_REASON_READ_FAILURES);
  }
  if (guidedPostureClippedSamples > 5) {
    addQualityIssue(guidedPostureQuality, guidedPostureQualityReasons,
                    QUALITY_REPEAT_TEST, QUALITY_REASON_CLIPPED);
  } else if (guidedPostureClippedSamples > 0) {
    addQualityIssue(guidedPostureQuality, guidedPostureQualityReasons,
                    QUALITY_WARNING, QUALITY_REASON_CLIPPED);
  }
  if (maxJitterUs > QUALITY_JITTER_WARN_MAX_US) {
    addQualityIssue(guidedPostureQuality, guidedPostureQualityReasons,
                    QUALITY_REPEAT_TEST, QUALITY_REASON_JITTER);
  } else if (maxJitterUs > QUALITY_JITTER_PASS_MAX_US) {
    addQualityIssue(guidedPostureQuality, guidedPostureQualityReasons,
                    QUALITY_WARNING, QUALITY_REASON_JITTER);
  }
}
float clampPrototypeScore(float value) {
  if (value < 0) return 0;
  if (value > 100) return 100;
  return value;
}
int computeGuidedRestScore(float &bandScore, float &consistencyScore) {
  bandScore = 0;
  consistencyScore = 0;
  float rmsG = rRmsMg / 1000.0f;
  if (rQuality == QUALITY_REPEAT_TEST || rmsG < ampDetect || !rInBand || rBandRatio < BAND_RATIO_MIN) return 0;

  bandScore = clampPrototypeScore(100.0f * (rBandRatio - BAND_RATIO_MIN) / (1.0f - BAND_RATIO_MIN));
  if (qualityFftChecked) {
    consistencyScore = clampPrototypeScore(100.0f * (1.0f - qualityFreqRangeHz / QUALITY_FREQ_WARN_RANGE_HZ));
  }
  return (int)roundf(0.65f * bandScore + 0.35f * consistencyScore);
}
void drawGuidedFinalResult() {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("PROTOTYPE MOTOR INDEX");
  display.setCursor(0,14); display.print("Rest:"); display.print(guidedRestScore);
  display.print(" Tap:"); display.print(guidedTapScore);
  display.setCursor(0,26); display.print("Hold:"); display.print(guidedPostureScore);
  display.print(" Q:"); display.print(qualityOledText(guidedOverallQuality));
  display.setTextSize(2); display.setCursor(0,38); display.print("CMPI "); display.print(guidedCmpi);
  display.setTextSize(1); display.setCursor(0,56); display.println("0-100 Not diagnosis");
  display.display();
}
void drawGuidedFinalInvalid() {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED TEST COMPLETE");
  display.setTextSize(2); display.setCursor(0,12); display.println("REPEAT");
  display.setTextSize(1); display.setCursor(0,36); display.println("Hold quality invalid");
  display.setCursor(0,48); display.println("CMPI suppressed");
  display.setCursor(0,56); display.println("Not a diagnosis");
  display.display();
}
void completeGuidedPostureTest();

void finishGuidedPostureTest() {
  guidedPosturePersistencePct = guidedPostureWindows > 0
                               ? 100.0f * guidedPostureTremorWindows / guidedPostureWindows
                               : 0;
  evaluateGuidedPostureQuality();
  Serial.print("# GUIDED TEST 3/3 HOLD complete,windows="); Serial.print(guidedPostureWindows);
  Serial.print(",target_tremor_windows="); Serial.print(guidedPostureTremorWindows);
  Serial.print(",persistence_pct="); Serial.print(guidedPosturePersistencePct, 1);
  Serial.print(",read_failures="); Serial.print(guidedPostureReadFailures);
  Serial.print(",clipped_samples="); Serial.print(guidedPostureClippedSamples);
  Serial.print(",max_jitter_us="); Serial.print(maxJitterUs);
  Serial.print(",quality="); Serial.print(qualityLevelText(guidedPostureQuality));
  Serial.print(",quality_reason="); printQualityReasonsInline(guidedPostureQualityReasons); Serial.println();

  xiaoAwaitSessionId = endXiaoCompute(effectiveFs);
  guidedPostureActive = false;
  if (xiaoAwaitSessionId != 0) {
    xiaoAwaitingResult = true;
    xiaoAwaitingGuidedPosture = true;
    xiaoResultDeadlineMs = millis() + XIAO_RESULT_TIMEOUT_MS;
    state = ST_XIAO_WAIT;
    drawXiaoProcessing();
    return;
  }
  computeSource = COMPUTE_SOURCE_LOCAL_FALLBACK;
  completeGuidedPostureTest();
}

void completeGuidedPostureTest() {
  guidedPostureActive = false;
  if (guidedPostureQuality == QUALITY_REPEAT_TEST) {
    guidedCmpi = -1;
    Serial.println("# GUIDED CMPI,result=SUPPRESSED,reason=HOLD_QUALITY_REPEAT_TEST");
    { // ==== WIFI/WS ADDON: send guided exam (posture quality insufficient, suppressed) ====
      webSocket.broadcastTXT(String("{\"type\":\"guided\",\"state\":\"done\",\"cmpi\":-1,\"suppressed\":1,\"quality\":\"")
        + qualityLevelText(guidedPostureQuality) + "\",\"classification_suppressed\":true,\"compute_source\":\""
        + computeSourceText() + "\",\"xiao_link\":\"" + (xiaoLink.online() ? "ONLINE" : "FAILED")
        + "\"}"); }
    drawGuidedFinalInvalid();
    enterPersistentResult();
    beep(1, 400, 0);
    return;
  }

  float restBandScore = 0, restConsistencyScore = 0;
  if (computeSource != COMPUTE_SOURCE_XIAO) {
    guidedRestScore = computeGuidedRestScore(restBandScore, restConsistencyScore);
    guidedTapScore = guidedTapPrototypeScore >= 0 ? guidedTapPrototypeScore * 25 : -1;
    guidedPostureScore = (int)roundf(clampPrototypeScore(guidedPosturePersistencePct));
    guidedCmpi = (guidedRestScore >= 0 && guidedTapScore >= 0)
        ? (int)roundf(0.40f * guidedRestScore + 0.40f * guidedTapScore + 0.20f * guidedPostureScore) : -1;
  }
  guidedOverallQuality = rQuality > guidedPostureQuality ? rQuality : guidedPostureQuality;
  if (guidedOverallQuality == QUALITY_REPEAT_TEST || guidedCmpi < 0) {
    guidedCmpi = -1;
    Serial.println("# GUIDED CMPI,result=SUPPRESSED,reason=OVERALL_QUALITY_OR_MISSING_STAGE");
    webSocket.broadcastTXT(String("{\"type\":\"guided\",\"state\":\"done\",\"cmpi\":-1,\"suppressed\":1,\"quality\":\"")
        + qualityLevelText(guidedOverallQuality) + "\",\"classification_suppressed\":true,\"compute_source\":\""
        + computeSourceText() + "\",\"xiao_link\":\"" + (xiaoLink.online() ? "ONLINE" : "FAILED")
        + "\"}");
    drawGuidedFinalInvalid();
    enterPersistentResult();
    beep(1, 400, 0);
    return;
  }

  // ---- guided three-task XGBoost (replaces 0.40/0.40/0.20 manual weighting; features computed by XIAO and returned, model in flash) ----
  { float g3f[PADS_G3_NFEAT]; for (int gi = 0; gi < PADS_G3_NFEAT; ++gi) g3f[gi] = NAN;
    if (g3RestValid) for (int gi = 0; gi < 12; ++gi) g3f[gi]      = g3RestFeat[gi];   // rest 0..11
    if (g3PostValid) for (int gi = 0; gi < 12; ++gi) g3f[12 + gi] = g3PostFeat[gi];   // posture 12..23
    // g3f[24..27] tap: the device tap test has no continuous accel spectrum -> leave NAN, imputed by the model with the training median
    g3PdProbability = (g3RestValid || g3PostValid) ? pads_g3_PD_vs_HC(g3f) : NAN; }
  Serial.print("# GUIDED SCORES,rest_band_score="); Serial.print(restBandScore, 1);
  Serial.print(",rest_consistency_score="); Serial.print(restConsistencyScore, 1);
  Serial.print(",rest_score="); Serial.print(guidedRestScore);
  Serial.print(",tap_prototype_score="); Serial.print(guidedTapPrototypeScore);
  Serial.print(",tap_score="); Serial.print(guidedTapScore);
  Serial.print(",hold_score="); Serial.print(guidedPostureScore);
  Serial.print(",cmpi="); Serial.print(guidedCmpi);
  Serial.print(",formula=0.40_REST+0.40_TAP+0.20_HOLD");
  Serial.print(",quality="); Serial.print(qualityLevelText(guidedOverallQuality));
  Serial.print(",pd_probability="); Serial.print(g3PdProbability, 4);   // guided three-task XGBoost (features computed by XIAO)
  Serial.println();
  { // ==== WIFI/WS ADDON: send guided exam ====
    String j = String("{\"type\":\"guided\",\"state\":\"done\",\"cmpi\":") + String(guidedCmpi)
      + ",\"rest_score\":" + String(guidedRestScore) + ",\"tap_score\":" + String(guidedTapScore)
      + ",\"hold_score\":" + String(guidedPostureScore)
      + ",\"quality\":\"" + qualityLevelText(guidedOverallQuality)
      + "\",\"classification_suppressed\":false,\"compute_source\":\"" + computeSourceText()
      + "\",\"xiao_link\":\"" + (xiaoLink.online() ? "ONLINE" : "FAILED")
      + "\",\"pd_probability\":" + (isfinite(g3PdProbability) ? String(g3PdProbability, 4) : "null") + "}";
    webSocket.broadcastTXT(j); }
  drawGuidedFinalResult();
  enterPersistentResult();
  beep(1, 120, 0);
}

void serviceMonitorExit() {
  uint32_t now = millis();
  if (now - monExitLastPollMs < MONITOR_EXIT_POLL_MS) return;
  monExitLastPollMs = now;

  uint8_t status = apdsOk ? apdsRead8(0x93) : 0;
  if (!apdsOk || (status & 0x02) == 0) return;                // wait for new proximity data
  monExitProx = apdsProximity();

  if (!monExitArmed) {
    if (monExitProx <= MONITOR_EXIT_ARM_MAX) {
      if (monExitAwayStartMs == 0) monExitAwayStartMs = now;
      if (now - monExitAwayStartMs >= MONITOR_EXIT_ARM_MS) {
        monExitArmed = true;
        Serial.print("# MONITOR EXIT armed — hold hand close ");
        Serial.print(MONITOR_EXIT_HOLD_MS / 1000.0f, 1); Serial.println("s");
      }
    } else {
      monExitAwayStartMs = 0;
    }
    return;
  }

  if (monExitHoldStartMs == 0) {
    if (monExitProx >= MONITOR_EXIT_PROX_ON) {
      monExitHoldStartMs = now;
      monExitLastDrawMs = now;
      Serial.println("# MONITOR EXIT hold started");
      drawMonitorExit(0);
    }
    return;
  }

  if (monExitProx < MONITOR_EXIT_PROX_OFF) {
    monExitHoldStartMs = 0;
    Serial.println("# MONITOR EXIT hold cancelled");
    drawMonitor(monLastFreq, monLastRmsMg, monActive, monEpisodes, (now-monStartMs)/1000);
    return;
  }

  uint32_t heldMs = now - monExitHoldStartMs;
  if (now - monExitLastDrawMs >= MONITOR_EXIT_DRAW_MS) {
    monExitLastDrawMs = now;
    drawMonitorExit(heldMs);
  }
  if (heldMs < MONITOR_EXIT_HOLD_MS) return;

  Serial.println("# MONITOR EXIT completed -> READY");
  endXiaoCompute(effectiveFs);
  beep(1, 120, 0);
  state = ST_IDLE;
  drawReady(0);
}

// ---------------- sample once ----------------
void doSample() {
  float ax, ay, az, gx, gy, gz;
  bool clipped = false;
  if (!mpuRead(ax, ay, az, gx, gy, gz, clipped)) {
    queueSampleForXiao(false);
    if (state == ST_MEASURING) sessionReadFailures++;
    if (state == ST_MONITOR && guidedPostureActive) guidedPostureReadFailures++;
    if (state == ST_CAL_REST || state == ST_CAL_TREMOR) {
      calReadFailures++;
      if (calReadFailures > CAL_MAX_READ_FAILURES) {
        mpuFailStreak = 0;
        showCalibrationFailure("too many MPU read failures", "sensor read error");
        return;
      }
    }
    if (++mpuFailStreak > 200) {
      Serial.println("# ERROR: MPU read lost, abort");
      guidedPostureActive = false;
      state = ST_IDLE; drawReady(lastProx); mpuFailStreak = 0;
    }
    return;
  }
  mpuFailStreak = 0;
  if (!gravInit) { gravX = ax; gravY = ay; gravZ = az; gravInit = true; }
  gravX = GRAV_ALPHA*gravX + (1-GRAV_ALPHA)*ax;
  gravY = GRAV_ALPHA*gravY + (1-GRAV_ALPHA)*ay;
  gravZ = GRAV_ALPHA*gravZ + (1-GRAV_ALPHA)*az;
  float lx = ax-gravX, ly = ay-gravY, lz = az-gravZ;
  // ==== WIFI/WS ADDON: continuous monitoring keeps the live waveform; measurement (TREMOR/LiftHold) instead sends the full waveform once when done.
  //      A synchronous broadcastTXT here during measurement would stall sampling via the TCP send and cause high jitter -> REPEAT TEST,
  //      so the measurement path instead uses broadcastSessionWaveform() to send it all after the result. ====
  if (state == ST_MONITOR && !guidedPostureActive && webSocket.connectedClients() > 0) {
    static uint8_t wsWaveN = 0;
    if ((++wsWaveN % 10) == 0)
      webSocket.broadcastTXT(String("{\"type\":\"wave\",\"amp\":") + String(sqrtf(lx*lx+ly*ly+lz*lz), 4) + "}");
  }

  if (streamRaw) {
    Serial.print(millis()); Serial.print(','); Serial.print(ax,4); Serial.print(','); Serial.print(ay,4); Serial.print(',');
    Serial.print(az,4); Serial.print(','); Serial.print(gx,2); Serial.print(','); Serial.print(gy,2); Serial.print(','); Serial.println(gz,2);
  }

  if (state == ST_SETTLE) {
    gyroBiasSumX += gx; gyroBiasSumY += gy; gyroBiasSumZ += gz; gyroBiasSamples++;
    if (millis()-settleStartMs >= SETTLE_MS) {
      if (gyroBiasSamples > 0) {
        gyroBiasX = (float)(gyroBiasSumX / gyroBiasSamples);
        gyroBiasY = (float)(gyroBiasSumY / gyroBiasSamples);
        gyroBiasZ = (float)(gyroBiasSumZ / gyroBiasSamples);
      }
      Serial.print("# GYRO BIAS,dps="); Serial.print(gyroBiasX,3); Serial.print(',');
      Serial.print(gyroBiasY,3); Serial.print(','); Serial.println(gyroBiasZ,3);
      if (liftHoldActive) {
        Serial.println("# LIFT_HOLD_CAPTURE_START,action=raise_and_extend_arm,seconds=10.24");
        beep(1, 120, 0);
      }
      beginMeasuring();
    }
    return;
  }

  queueSampleForXiao(true);

  if (state == ST_MEASURING) {                       // accumulate multi-features
    gx -= gyroBiasX; gy -= gyroBiasY; gz -= gyroBiasZ;
  }
  bufX[bufIdx] = lx; bufY[bufIdx] = ly; bufZ[bufIdx] = lz;
  bufIdx++;

  if (state == ST_MEASURING) {                       // accumulate multi-features
    if (clipped) sessionClippedSamples++;
    sessSumSq += (double)lx*lx + (double)ly*ly + (double)lz*lz;
    gyroSumSq += (double)gx*gx + (double)gy*gy + (double)gz*gz;
    if (prevLxValid) {
      float dlx = (lx-prevLx)*SAMPLE_HZ;             // x component of jerk (using lx; enough to reflect smoothness)
      jerkSumSq += (double)dlx*dlx;
    }
    prevLx = lx; prevLxValid = true;
    float mag = sqrtf(lx*lx + ly*ly + lz*lz); if (mag > peakMag) peakMag = mag;
    int s = (lx > 0) ? 1 : (lx < 0 ? -1 : 0);
    if (s != 0 && prevSign != 0 && s != prevSign) zcrCount++;
    if (s != 0) prevSign = s;
    if (sessSamples < MEAS_TOTAL_SAMPLES) {
      sessionChannelSamples[AX_ACC_X][sessSamples] = lx;
      sessionChannelSamples[AX_ACC_Y][sessSamples] = ly;
      sessionChannelSamples[AX_ACC_Z][sessSamples] = lz;
      sessionChannelSamples[AX_GYRO_X][sessSamples] = gx;
      sessionChannelSamples[AX_GYRO_Y][sessSamples] = gy;
      sessionChannelSamples[AX_GYRO_Z][sessSamples] = gz;
      if ((sessSamples % (MEAS_TOTAL_SAMPLES / MAG_QUANTILE_SAMPLES)) == 0) {
        int quantileIndex = sessSamples / (MEAS_TOTAL_SAMPLES / MAG_QUANTILE_SAMPLES);
        if (quantileIndex < MAG_QUANTILE_SAMPLES) {
          magnitudeQuantileSamples[0][quantileIndex] = mag;
          magnitudeQuantileSamples[1][quantileIndex] = sqrtf(gx*gx + gy*gy + gz*gz);
        }
      }
      sessSamples++;
    }
  }
  if ((state == ST_CAL_REST || state == ST_CAL_TREMOR) && clipped) calClippedSamples++;
  if (state == ST_MONITOR && guidedPostureActive && clipped) guidedPostureClippedSamples++;
  if (state == ST_MONITOR) monWinSumSq += (double)lx*lx + (double)ly*ly + (double)lz*lz;

  if (bufIdx >= FFT_SIZE) {
    int newSamples = analysisWindowPrimed ? FFT_HOP_SIZE : FFT_SIZE;
    effectiveFs = newSamples * 1000000.0f / (float)(micros() - windowStartUs);
    if (state == ST_MEASURING) {
      // Keep the pre-accumulation PSD; subtracting after accumulation yields this window's spectrum without re-running FFT.
      for (int i = 0; i < FFT_SIZE/2; i++) qualityWindowPsd[i] = psd[i];
      accumulateSpectrum(bufX); accumulateSpectrum(bufY); accumulateSpectrum(bufZ);
      for (int i = 0; i < FFT_SIZE/2; i++) {
        float windowPower = psd[i] - qualityWindowPsd[i];
        qualityWindowPsd[i] = windowPower > 0 ? windowPower : 0;
      }
      if (sessWindows < MEAS_WINDOWS) {
        qualityWindowFs[sessWindows] = effectiveFs;
        qualityWindowFreq[sessWindows] = peakFreqFromSpectrum(qualityWindowPsd);
        qualityWindowMagnitude[sessWindows] = peakMagnitudeFromSpectrum(qualityWindowPsd);
        qualityWindowBandRatio[sessWindows] = calcBandRatioFromSpectrum(qualityWindowPsd);
      }
      sessWindows++;
#if MEAS_LIVE_UPDATE
      float liveFreq = peakFreqFromPsd();
      float liveRmsMg = (sessSamples>0) ? sqrtf((float)(sessSumSq/sessSamples))*1000.0f : 0;
      Serial.print("# LIVE,win="); Serial.print(sessWindows); Serial.print(",freq_Hz="); Serial.print(liveFreq,2);
      Serial.print(",rms_mg="); Serial.println(liveRmsMg,1);
      drawMeasuringLive(sessWindows, MEAS_WINDOWS, liveFreq, liveRmsMg);
#endif
      if (sessWindows >= MEAS_WINDOWS || sessSamples >= MEAS_TOTAL_SAMPLES) { finalizeSession(); return; }
      shiftAnalysisBuffersForOverlap();
      analysisWindowPrimed = true;
    } else if (state == ST_CAL_REST) {
      calRest[calIdx++] = windowRms(); bufIdx = 0;
      drawProgressCal("CAL rest", calIdx, CAL_WINDOWS, "keep STILL");
      if (calIdx >= CAL_WINDOWS) {
        endXiaoCompute(effectiveFs);
        beginXiaoCompute(TremorLink::MODE_CAL_SHAKE, CAL_WINDOWS * FFT_SIZE);
        Serial.println("# CAL - shake ~15s (~5Hz)");
        state = ST_CAL_TREMOR; calIdx = 0;
        drawProgressCal("CAL shake", 0, CAL_WINDOWS, "shake ~5Hz");
      }
    } else if (state == ST_CAL_TREMOR) {
      calTremor[calIdx++] = windowRms(); bufIdx = 0;
      drawProgressCal("CAL shake", calIdx, CAL_WINDOWS, "shake ~5Hz");
      if (calIdx >= CAL_WINDOWS) { finishCalibration(); return; }
    } else if (state == ST_MONITOR) {                 // continuous monitoring: analyze each window independently
      for (int i = 0; i < FFT_SIZE/2; i++) psd[i] = 0;
      accumulateSpectrum(bufX); accumulateSpectrum(bufY); accumulateSpectrum(bufZ);
      float f = peakFreqFromPsd();
      float br = calcBandRatioFromPsd();
      float rms = windowRms();
      monLastFreq = f;
      monLastRmsMg = rms * 1000.0f;
      bool inB = (f>=TREMOR_LO_HZ && f<=TREMOR_HI_HZ);
      bool tremorNow = inB && (br>=BAND_RATIO_MIN) && (rms>=ampDetect); // false-alarm guard: in-band + regular + above detection threshold
      const bool useLocalMonitorResult = guidedPostureActive || !xiaoLink.online();
      if (useLocalMonitorResult) {
        if (tremorNow) {
          monTremorStreak++;
          if (monTremorStreak==2 && !monActive) {
            monActive=true; monEpisodes++;
            if (!guidedPostureActive) beep(2,80,80); // avoid the buzzer stalling sampling during Guided fixed sampling
          }
        }
        else { monTremorStreak=0; monActive=false; }
      }
      if (guidedPostureActive) {
        if (guidedPostureWindows < GUIDED_POSTURE_WINDOWS) {
          guidedPostureWindowFs[guidedPostureWindows] = effectiveFs;
        }
        guidedPostureWindows++;
        if (tremorNow) guidedPostureTremorWindows++;
      }
      uint32_t el=(millis()-monStartMs)/1000;
      Serial.print("# MON,t="); Serial.print(el); Serial.print("s,freq_Hz="); Serial.print(f,2);
      Serial.print(",rms_mg="); Serial.print(rms*1000.0f,1); Serial.print(",band_ratio="); Serial.print(br,2);
      Serial.print(",tremor="); Serial.print(tremorNow?1:0); Serial.print(",episodes="); Serial.println(monEpisodes);
      if (useLocalMonitorResult) { // ==== WIFI/WS ADDON: local fallback monitoring result ====
        String j = String("{\"type\":\"monitor\",\"active\":") + (tremorNow ? "true" : "false")
          + ",\"episodes\":" + String(monEpisodes) + ",\"elapsed_s\":" + String(el)
          + ",\"freq\":" + String(f,1) + ",\"mg\":" + String(rms*1000.0f,0)
          + ",\"compute_source\":\"LOCAL_FALLBACK\",\"xiao_link\":\"FAILED\"}";
        webSocket.broadcastTXT(j);
      }
      if (monExitHoldStartMs == 0 && useLocalMonitorResult)
        drawMonitor(f, monLastRmsMg, guidedPostureActive ? tremorNow : monActive, monEpisodes, el);
      monWinSumSq = 0;
      if (guidedPostureActive && guidedPostureWindows >= GUIDED_POSTURE_WINDOWS) {
        finishGuidedPostureTest();
        return;
      }
      shiftAnalysisBuffersForOverlap();
      analysisWindowPrimed = true;
    } else bufIdx = 0;
    windowStartUs = micros(); nextSampleUs = micros() + SAMPLE_US;
  }
}

// ---------------- IDLE / RESULT / CAL RESULT service ----------------
void startTapTest();                                           // forward declaration (called by serviceIdle, defined later)
void drawGuidedRestInstruction(uint8_t secondsLeft) {
  if (oledOk) {
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED TEST");
    display.setTextSize(2); display.setCursor(0,12); display.println("1/3 REST");
    display.setTextSize(1); display.setCursor(0,34); display.println("Hand on thigh");
    display.setCursor(0,44); display.println("Relax and keep still");
    display.setCursor(0,56); display.print("Starting in: "); display.print(secondsLeft);
    display.display();
  }
}
void startGuidedRestStage() {
  apdsSetTapProximityProfile(false);
  sessionTask = SESSION_REST;
  liftHoldActive = false;
  guidedRestActive = true;
  guidedTapActive = false;
  guidedPostureActive = false;
  guidedTapPrototypeScore = -1;
  g3RestValid = false; g3PostValid = false; g3PdProbability = NAN;   // reset guided three-task XGBoost state
  guidedPosturePersistencePct = 0;
  guidedRestScore = guidedTapScore = guidedPostureScore = guidedCmpi = -1;
  guidedOverallQuality = QUALITY_PASS;
  guidedRestInstructionStartMs = millis();
  guidedRestLastCountdown = (GUIDED_REST_INSTRUCTION_MS + 999) / 1000;
  state = ST_GUIDED_REST;
  Serial.println("# GUIDED TEST 1/3 REST — hand on thigh, relax and keep still");
  drawGuidedRestInstruction(guidedRestLastCountdown);
  beep(1, 100, 0);
}
void serviceGuidedRestStage() {
  uint32_t elapsedMs = millis() - guidedRestInstructionStartMs;
  if (elapsedMs >= GUIDED_REST_INSTRUCTION_MS) {
    Serial.println("# GUIDED TEST 1/3 REST — capture start");
    startSession();
    return;
  }
  int8_t secondsLeft = (GUIDED_REST_INSTRUCTION_MS - elapsedMs + 999) / 1000;
  if (secondsLeft != guidedRestLastCountdown) {
    guidedRestLastCountdown = secondsLeft;
    drawGuidedRestInstruction(secondsLeft);
  }
}

void drawLiftHoldInstruction(uint8_t secondsLeft) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("PADS ML");
  display.setTextSize(2); display.setCursor(0,12); display.println("LIFT+HOLD");
  display.setTextSize(1); display.setCursor(0,35); display.println("Arm relaxed at side");
  display.setCursor(0,46); display.println("Raise/extend at beep");
  display.setCursor(0,57); display.print("Ready in: "); display.print(secondsLeft);
  display.display();
}

void startLiftHoldProtocol() {
  apdsSetTapProximityProfile(false);
  guidedRestActive = false;
  guidedTapActive = false;
  guidedPostureActive = false;
  liftHoldActive = false;
  sessionTask = SESSION_LIFT_HOLD;
  liftHoldInstructionStartMs = millis();
  liftHoldLastCountdown = (LIFT_HOLD_INSTRUCTION_MS + 999) / 1000;
  state = ST_LIFT_HOLD_INSTRUCTION;
  Serial.println("# LIFT_HOLD_PROTOCOL,phase=instruction,arm=relaxed,raise_at=capture_beep");
  drawLiftHoldInstruction(liftHoldLastCountdown);
  webSocket.broadcastTXT("{\"type\":\"lift_hold\",\"phase\":\"instruction\"}");
}

void serviceLiftHoldInstruction() {
  uint32_t elapsedMs = millis() - liftHoldInstructionStartMs;
  if (elapsedMs >= LIFT_HOLD_INSTRUCTION_MS) {
    liftHoldActive = true;
    Serial.println("# LIFT_HOLD_PROTOCOL,phase=gyro_settle,arm=down");
    startSession();
    return;
  }
  int8_t secondsLeft = (LIFT_HOLD_INSTRUCTION_MS - elapsedMs + 999) / 1000;
  if (secondsLeft != liftHoldLastCountdown) {
    liftHoldLastCountdown = secondsLeft;
    drawLiftHoldInstruction(secondsLeft);
  }
}

void drawGuidedTapInstruction(uint8_t secondsLeft) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED TEST");
  display.setTextSize(2); display.setCursor(0,12); display.println("2/3 TAP");
  display.setTextSize(1); display.setCursor(0,34); display.println("Tap fast & regular");
  display.setCursor(0,44); display.println("Complete 10 taps");
  display.setCursor(0,56); display.print("Starting in: "); display.print(secondsLeft);
  display.display();
}
void startGuidedTapStage() {
  guidedTapActive = true;
  guidedTapInstructionStartMs = millis();
  guidedTapLastCountdown = (GUIDED_TAP_INSTRUCTION_MS + 999) / 1000;
  state = ST_GUIDED_TAP;
  Serial.println("# GUIDED TEST 2/3 TAP — tap fast and regular, complete 10 taps");
  drawGuidedTapInstruction(guidedTapLastCountdown);
  beep(2, 80, 80);
}
void serviceGuidedRestCompleteStage() {
  if (millis() - guidedRestResultStartMs < GUIDED_STAGE_RESULT_MS) return;
  startGuidedTapStage();
}
void serviceGuidedTapStage() {
  uint32_t elapsedMs = millis() - guidedTapInstructionStartMs;
  if (elapsedMs >= GUIDED_TAP_INSTRUCTION_MS) {
    Serial.println("# GUIDED TEST 2/3 TAP — preparation start");
    startTapTest();
    return;
  }
  int8_t secondsLeft = (GUIDED_TAP_INSTRUCTION_MS - elapsedMs + 999) / 1000;
  if (secondsLeft != guidedTapLastCountdown) {
    guidedTapLastCountdown = secondsLeft;
    drawGuidedTapInstruction(secondsLeft);
  }
}
void drawGuidedTapComplete(int prototypeScore) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED TEST");
  display.setTextSize(2); display.setCursor(0,12); display.println("TAP DONE");
  display.setTextSize(1); display.setCursor(0,38); display.print("Tap score: "); display.print(prototypeScore); display.println("/4");
  display.setCursor(0,52); display.println("Next: 3/3 HOLD");
  display.display();
}
void drawGuidedTapInvalid(uint8_t count) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED 2/3 TAP");
  display.setTextSize(2); display.setCursor(0,12); display.println("INCOMPLETE");
  display.setTextSize(1); display.setCursor(0,40); display.print("Taps: "); display.print(count); display.print("/"); display.println(N_TAPS);
  display.setCursor(0,54); display.println("Test stopped");
  display.display();
}
void drawGuidedPostureInstruction(uint8_t secondsLeft) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED TEST");
  display.setTextSize(2); display.setCursor(0,12); display.println("3/3 HOLD");
  display.setTextSize(1); display.setCursor(0,34); display.println("Get ready, arm rests");
  display.setCursor(0,44); display.println("Raise arm at beep");
  display.setCursor(0,56); display.print("Starting in: "); display.print(secondsLeft);
  display.display();
}
void drawGuidedPostureSettle() {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED TEST");
  display.setTextSize(2); display.setCursor(0,12); display.println("3/3 HOLD");
  display.setTextSize(1); display.setCursor(0,34); display.println("RAISE ARM NOW");
  display.setCursor(0,44); display.println("Palm down, keep still");
  display.setCursor(0,56); display.print("Settling "); display.print(SETTLE_MS / 1000.0f, 1); display.println("s");
  display.display();
}
void startGuidedPostureStage() {
  guidedPostureInstructionStartMs = millis();
  guidedPostureLastCountdown = (GUIDED_POSTURE_INSTRUCTION_MS + 999) / 1000;
  state = ST_GUIDED_POSTURE;
  Serial.println("# GUIDED TEST 3/3 HOLD — keep arm resting; raise at beep");
  drawGuidedPostureInstruction(guidedPostureLastCountdown);
}
void serviceGuidedTapCompleteStage() {
  if (millis() - guidedTapResultStartMs < GUIDED_STAGE_RESULT_MS) return;
  startGuidedPostureStage();
}
void startGuidedPostureSettle() {
  state = ST_GUIDED_POSTURE_SETTLE;
  Serial.print("# GUIDED TEST 3/3 HOLD — raise arm now, settling ");
  Serial.print(SETTLE_MS / 1000.0f, 1); Serial.println("s");
  drawGuidedPostureSettle();
  beep(1, 100, 0);
  guidedPostureSettleStartMs = millis();
}
void startGuidedPostureCapture() {
  guidedPostureWindows = 0;
  guidedPostureTremorWindows = 0;
  guidedPosturePersistencePct = 0;
  guidedPostureReadFailures = 0;
  guidedPostureClippedSamples = 0;
  guidedPostureQuality = QUALITY_PASS;
  guidedPostureQualityReasons = 0;
  for (int i = 0; i < GUIDED_POSTURE_WINDOWS; i++) guidedPostureWindowFs[i] = 0;
  guidedPostureActive = true;
  Serial.print("# GUIDED TEST 3/3 HOLD — capture start,windows=");
  Serial.println(GUIDED_POSTURE_WINDOWS);
  beep(1, 50, 0);
  startMonitor();
}
void serviceGuidedPostureStage() {
  uint32_t elapsedMs = millis() - guidedPostureInstructionStartMs;
  if (elapsedMs >= GUIDED_POSTURE_INSTRUCTION_MS) {
    startGuidedPostureSettle();
    return;
  }
  int8_t secondsLeft = (GUIDED_POSTURE_INSTRUCTION_MS - elapsedMs + 999) / 1000;
  if (secondsLeft != guidedPostureLastCountdown) {
    guidedPostureLastCountdown = secondsLeft;
    drawGuidedPostureInstruction(secondsLeft);
  }
}
void serviceGuidedPostureSettleStage() {
  if (millis() - guidedPostureSettleStartMs < SETTLE_MS) return;
  startGuidedPostureCapture();
}
void serviceIdle() {
  handleGestureModeSelection();                               // non-blocking: each loop only processes the current FIFO data

  uint32_t now = millis();
  if (now - lastPollMs < 60) return;
  lastPollMs = now;
  lastProx = apdsOk ? fifoPresenceLevel : 0;
  drawReady(lastProx);
  if (!handNear || now - holdStartMs < MODE_START_HOLD_MS) return;

  handNear = false;
  holdStartMs = 0;
  apdsSetGestureEngine(false);
  Serial.print("# MODE START after "); Serial.print(MODE_START_HOLD_MS / 1000.0f, 1);
  Serial.print("s hold: ");
  Serial.println(selMode==MODE_GUIDED ? "GUIDED" : (selMode==MODE_TAP ? "TAP" : (selMode==MODE_MONITOR ? "MONITOR" : "TREMOR")));
  if (selMode == MODE_GUIDED) startGuidedRestStage();
  else if (selMode == MODE_TAP) startTapTest();
  else if (selMode == MODE_MONITOR) startMonitor();
  else {
    sessionTask = SESSION_TREMOR_QUANT;
    liftHoldActive = false;
    startSession();
  }
}
void serviceCalResult() {
  if (millis() - calResultStartMs >= CAL_RESULT_MS) state = ST_IDLE;
}
void serviceResult() {
  uint32_t now = millis();
  if (now - resultExitLastPollMs < MONITOR_EXIT_POLL_MS) return;
  resultExitLastPollMs = now;

  uint8_t status = apdsOk ? apdsRead8(0x93) : 0;
  if (!apdsOk || (status & 0x02) == 0) return;
  resultExitProx = apdsProximity();
  lastProx = resultExitProx;

  if (!resultExitArmed) {
    if (resultExitProx <= MONITOR_EXIT_ARM_MAX) {
      if (resultExitAwayStartMs == 0) resultExitAwayStartMs = now;
      if (now - resultExitAwayStartMs >= MONITOR_EXIT_ARM_MS) {
        resultExitArmed = true;
        Serial.print("# RESULT EXIT armed — hold hand close ");
        Serial.print(MONITOR_EXIT_HOLD_MS / 1000.0f, 1); Serial.println("s");
      }
    } else {
      resultExitAwayStartMs = 0;
    }
    return;
  }

  if (resultExitHoldStartMs == 0) {
    if (resultExitProx >= MONITOR_EXIT_PROX_ON) {
      resultExitHoldStartMs = now;
      resultExitLastDrawMs = now;
      Serial.println("# RESULT EXIT hold started");
      drawResultExit(0);
    }
    return;
  }

  if (resultExitProx < MONITOR_EXIT_PROX_OFF) {
    resultExitHoldStartMs = 0;
    Serial.println("# RESULT EXIT hold cancelled");
    restoreResultOled();
    return;
  }

  uint32_t heldMs = now - resultExitHoldStartMs;
  if (now - resultExitLastDrawMs >= MONITOR_EXIT_DRAW_MS) {
    resultExitLastDrawMs = now;
    drawResultExit(heldMs);
  }
  if (heldMs < MONITOR_EXIT_HOLD_MS) return;

  Serial.println("# RESULT EXIT completed -> READY");
  beep(1, 120, 0);
  state = ST_IDLE;
  drawReady(0);
}

// ---------------- finger-tap multi-metric screening (not an MDS-UPDRS clinical score) ----------------
void drawTapTest(int n) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0); display.println(guidedTapActive ? "GUIDED 2/3 TAP" : "Finger-tap test");
  display.setTextSize(3); display.setCursor(28,20); display.print(n); display.print("/"); display.print(N_TAPS);
  display.setTextSize(1); display.setCursor(0,56); display.print("tap fast & big");
  display.display();
}
void drawTapPreparation(const char *title, const char *instruction) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  if (guidedTapActive) {
    display.setTextSize(1); display.setCursor(0,0); display.println("GUIDED 2/3 TAP");
    display.setTextSize(2); display.setCursor(0,14); display.println(title);
    display.setTextSize(1); display.setCursor(0,44); display.println(instruction);
    display.display();
    return;
  }
  display.setTextSize(2); display.setCursor(0,0); display.println(title);
  display.setTextSize(1); display.setCursor(0,36); display.println(instruction);
  display.display();
}
void startTapTest() {
  apdsSetTapProximityProfile(true);
  state = ST_TAPTEST;
  tapPhase = TAP_WAIT_RELEASE;
  tapPhaseStartMs = millis();
  tapReleaseStableStartMs = 0;
  tapBaselineSum = 0;
  tapBaselineMax = 0;
  tapBaselineSamples = 0;
  tapCount = 0;
  tapDown = false;
  tapPeakProximity = 0;
  for (int i = 0; i < N_TAPS; i++) tapAmplitudes[i] = 0;
  lastTapPollMs = 0;
  Serial.println("# TAPTEST: move hand away");
  drawTapPreparation("Move away", "remove hand to calibrate");
}
int scoreTapSpeed(float meanS) {
  if (meanS < 0.45f) return 0;
  if (meanS < 0.55f) return 1;
  if (meanS < 0.70f) return 2;
  if (meanS < 0.90f) return 3;
  return 4;
}
int scoreTapRhythm(float cv) {
  if (cv < 0.10f) return 0;
  if (cv < 0.15f) return 1;
  if (cv < 0.22f) return 2;
  if (cv < 0.30f) return 3;
  return 4;
}
int scoreTapInterruptions(int interruptions) {
  if (interruptions == 0) return 0;
  if (interruptions <= 2) return 1;  // 1-2 interruptions/hesitations
  if (interruptions <= 5) return 2;  // 3-5 times
  return 3;                          // >5 times
}
void finishTapTest() {
  int prototypeScore = -1, speedScore = -1, rhythmScore = -1, interruptionScore = -1;
  int interruptions = 0; float meanS = 0, cv = 0;
  float amplitudeDecrementPct = 0, fatigueSlopePctPerTap = 0, itiFatigueSlopeMsPerTap = 0;
  if (tapCount >= 5) {                                   // need at least 5 taps to compute timing features
    int n = tapCount - 1;
    uint32_t intervals[N_TAPS-1], sortedIntervals[N_TAPS-1];
    double sum = 0;
    for (int i = 1; i < tapCount; i++) {
      intervals[i-1] = tapTimes[i] - tapTimes[i-1];
      sortedIntervals[i-1] = intervals[i-1];
      sum += intervals[i-1];
    }
    double meanMs = sum / n;
    for (int i=1; i<n; i++) { uint32_t key=sortedIntervals[i]; int j=i-1; while(j>=0 && sortedIntervals[j]>key){sortedIntervals[j+1]=sortedIntervals[j];j--;} sortedIntervals[j+1]=key; }
    float medianMs = (n & 1) ? sortedIntervals[n/2] : (sortedIntervals[n/2-1] + sortedIntervals[n/2]) * 0.5f;
    float interruptionThresholdMs = medianMs * 2.0f;  // personalized threshold: over 2x the person's median period counts as an interruption
    double var = 0;
    for (int i = 0; i < n; i++) {
      double d = (double)intervals[i] - meanMs; var += d*d;
      if (intervals[i] > interruptionThresholdMs) interruptions++;
    }
    cv = (meanMs > 0) ? (float)(sqrt(var/n)/meanMs) : 0; // rhythm coefficient of variation
    meanS = (float)(meanMs/1000.0);                      // mean inter-tap interval (s)
    // Linear slope of ITI vs tap index; positive means tapping slows down over time.
    double xMean = (n - 1) * 0.5, itiCov = 0, xVar = 0;
    for (int i = 0; i < n; i++) {
      double xd = i - xMean;
      itiCov += xd * ((double)intervals[i] - meanMs);
      xVar += xd * xd;
    }
    itiFatigueSlopeMsPerTap = xVar > 0 ? (float)(itiCov / xVar) : 0;

    int edgeCount = tapCount >= 6 ? 3 : 2;
    float earlyAmplitude = 0, lateAmplitude = 0, amplitudeMean = 0;
    for (int i = 0; i < tapCount; i++) amplitudeMean += tapAmplitudes[i];
    amplitudeMean /= tapCount;
    for (int i = 0; i < edgeCount; i++) {
      earlyAmplitude += tapAmplitudes[i];
      lateAmplitude += tapAmplitudes[tapCount-edgeCount+i];
    }
    earlyAmplitude /= edgeCount; lateAmplitude /= edgeCount;
    amplitudeDecrementPct = earlyAmplitude > 0 ? 100.0f * (earlyAmplitude-lateAmplitude)/earlyAmplitude : 0;
    // Normalized slope of proximity amplitude vs tap index; negative means amplitude decreases tap by tap.
    double tapXMean = (tapCount - 1) * 0.5, ampCov = 0, tapXVar = 0;
    for (int i = 0; i < tapCount; i++) {
      double xd = i - tapXMean;
      ampCov += xd * (tapAmplitudes[i] - amplitudeMean);
      tapXVar += xd * xd;
    }
    fatigueSlopePctPerTap = (tapXVar > 0 && amplitudeMean > 0)
                           ? (float)(100.0 * ampCov / tapXVar / amplitudeMean) : 0;
    speedScore = scoreTapSpeed(meanS);
    rhythmScore = scoreTapRhythm(cv);
    interruptionScore = scoreTapInterruptions(interruptions);

    prototypeScore = (int)roundf(speedScore*0.45f + rhythmScore*0.35f + interruptionScore*0.20f);
    int maxComponent = speedScore;
    if (rhythmScore > maxComponent) maxComponent = rhythmScore;
    if (interruptionScore > maxComponent) maxComponent = interruptionScore;
    if (prototypeScore < maxComponent-1) prototypeScore = maxComponent-1; // avoid a single clear anomaly being fully diluted by the average
    if (prototypeScore < 0) prototypeScore = 0;
    if (prototypeScore > 4) prototypeScore = 4;
  }
  Serial.print("# TAPTEST,taps="); Serial.print(tapCount);
  Serial.print(",mean_iti_s="); Serial.print(meanS,3);
  Serial.print(",cv="); Serial.print(cv,2);
  Serial.print(",interruptions="); Serial.print(interruptions);
  Serial.print(",amplitude_decrement_pct="); Serial.print(amplitudeDecrementPct,2);
  Serial.print(",fatigue_slope_pct_per_tap="); Serial.print(fatigueSlopePctPerTap,3);
  Serial.print(",iti_fatigue_slope_ms_per_tap="); Serial.print(itiFatigueSlopeMsPerTap,2);
  Serial.print(",speed_score="); Serial.print(speedScore);
  Serial.print(",rhythm_score="); Serial.print(rhythmScore);
  Serial.print(",interruption_score="); Serial.print(interruptionScore);
  Serial.print(",prototype_tap_score="); Serial.println(prototypeScore);
  TremorLink::TapSummaryPayload tapSummary{};
  tapSummary.score = prototypeScore >= 0 ? static_cast<uint8_t>(prototypeScore) : 0xFF;
  tapSummary.quality = (tapCount == N_TAPS && prototypeScore >= 0)
                       ? TremorLink::LINK_QUALITY_PASS : TremorLink::LINK_QUALITY_REPEAT_TEST;
  tapSummary.tapCount = tapCount;
  tapSummary.meanIntervalMs = meanS * 1000.0f;
  tapSummary.intervalCv = cv;
  xiaoLink.sendTapSummary(tapSummary);
  { // ==== WIFI/WS ADDON: send tap result ====
    String j = String("{\"type\":\"tap\",\"state\":\"done\",\"count\":") + String(tapCount)
      + ",\"n\":10,\"mean_ms\":" + String(meanS*1000.0f,0) + ",\"cv\":" + String(cv,2)
      + ",\"interruptions\":" + String(interruptions)
      + ",\"amplitude_decrement_pct\":" + String(amplitudeDecrementPct,2)
      + ",\"fatigue_slope_pct_per_tap\":" + String(fatigueSlopePctPerTap,3)
      + ",\"grade\":\"G" + String(prototypeScore) + "\"}";
    webSocket.broadcastTXT(j); }
  if (oledOk) {
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1); display.setCursor(0,0);
    if (prototypeScore < 0) display.println("TAP: too few");
    else {
      display.print("taps:"); display.print(tapCount);
      display.print(" ITI"); display.print(meanS,2); display.println("s");
      display.setCursor(0,12); display.print("CV:"); display.print(cv,2);
      display.print(" pause:"); display.print(interruptions);
      display.setCursor(0,30); display.print("Prototype Tap Score");
      display.setTextSize(2); display.setCursor(38,49); display.print(prototypeScore); display.print("/4");
    }
    display.display();
  }
  if (guidedTapActive) {
    bool guidedTapValid = tapCount == N_TAPS && prototypeScore >= 0;
    Serial.print("# GUIDED TEST 2/3 TAP complete,valid=");
    Serial.println(guidedTapValid ? 1 : 0);
    guidedTapActive = false;
    if (guidedTapValid) {
      guidedTapPrototypeScore = prototypeScore;
      guidedTapResultStartMs = millis();
      state = ST_GUIDED_TAP_DONE;
      drawGuidedTapComplete(prototypeScore);
      beep(1, 100, 0);
      return;
    }
    drawGuidedTapInvalid(tapCount);
    enterPersistentResult();
    beep(1, 400, 0);
    return;
  }
  enterPersistentResult();
  beep(1, 100, 0);                                        // just signals test completion; does not encode the score in beep count
}
void serviceTapTest() {
  if (millis()-lastTapPollMs < TAP_POLL_MS) return; lastTapPollMs = millis();
  uint8_t p = apdsOk ? apdsProximity() : 0; uint32_t now = millis();

  if (tapPhase == TAP_WAIT_RELEASE) {
    if (p < PROX_ON_THRESH) {
      if (tapReleaseStableStartMs == 0) tapReleaseStableStartMs = now;
      if (now - tapReleaseStableStartMs >= TAP_RELEASE_STABLE_MS) {
        tapPhase = TAP_BASELINE;
        tapPhaseStartMs = now;
        tapBaselineSum = 0;
        tapBaselineMax = 0;
        tapBaselineSamples = 0;
        Serial.println("# TAPTEST: baseline 1s, keep hand away");
        drawTapPreparation("Baseline", "keep hand away 1s");
      }
    } else {
      tapReleaseStableStartMs = 0;
    }

    if (now - tapPhaseStartMs >= TAP_RELEASE_TIMEOUT_MS) {
      Serial.println("# TAPTEST abort: hand was not removed");
      if (guidedTapActive) {
        guidedTapActive = false;
        drawGuidedTapInvalid(tapCount);
        enterPersistentResult();
        beep(1, 400, 0);
        return;
      }
      state = ST_IDLE;
      drawReady(p);
    }
    return;
  }

  if (tapPhase == TAP_BASELINE) {
    if (p >= PROX_ON_THRESH) {
      tapPhase = TAP_WAIT_RELEASE;
      tapPhaseStartMs = now;
      tapReleaseStableStartMs = 0;
      Serial.println("# TAPTEST: hand returned, restart release check");
      drawTapPreparation("Move away", "remove hand to calibrate");
      return;
    }

    tapBaselineSum += p;
    tapBaselineSamples++;
    if (p > tapBaselineMax) tapBaselineMax = p;
    if (now - tapPhaseStartMs < TAP_BASELINE_MS) return;

    tapBaseline = tapBaselineSamples ? tapBaselineSum / tapBaselineSamples : 0;
    tapOn = tapBaselineMax + TAP_ON_MARGIN;
    if (tapOn < tapBaseline + TAP_ON_MARGIN) tapOn = tapBaseline + TAP_ON_MARGIN;
    if (tapOn > 255) tapOn = 255;
    uint16_t tapSpan = tapOn > tapBaseline ? tapOn - tapBaseline : 1;
    tapOff = tapBaseline + (tapSpan * TAP_RELEASE_RATIO) / 100;
    if (tapOff >= tapOn) tapOff = tapOn > 0 ? tapOn - 1 : 0;

    Serial.print("# baseline avg="); Serial.print(tapBaseline);
    Serial.print(" max="); Serial.print(tapBaselineMax);
    Serial.print(" -> TAP_ON="); Serial.print(tapOn);
    Serial.print(" TAP_OFF="); Serial.println(tapOff);
    Serial.println("# TAPTEST active: tap 10 times");
    tapPhase = TAP_ACTIVE;
    tapStartMs = now;
    tapDown = false;
    drawTapTest(0);
    return;
  }

  if (tapDown && tapCount > 0 && p > tapPeakProximity) {
    tapPeakProximity = p;
    tapAmplitudes[tapCount-1] = tapPeakProximity > tapBaseline ? tapPeakProximity - tapBaseline : 0;
  }
  if (!tapDown && p >= tapOn) {                          // finger pressed down (rising edge, adaptive threshold)
    if (tapCount == 0 || now - tapTimes[tapCount-1] >= TAP_REFRACT_MS) {
      tapDown = true;
      if (tapCount < N_TAPS) {
        tapTimes[tapCount] = now;
        tapPeakProximity = p;
        tapAmplitudes[tapCount] = p > tapBaseline ? p - tapBaseline : 0;
        tapCount++;
        drawTapTest(tapCount);
      }
    }
  } else if (tapDown && p < tapOff) tapDown = false;          // finger lifted
  if (tapCount >= N_TAPS && (!tapDown || now-tapTimes[tapCount-1] > 300)) { finishTapTest(); return; }
  if (now - tapStartMs > TAPTEST_TIMEOUT_MS) { finishTapTest(); return; }
}

// ---------------- proximity watch (for threshold tuning, does not trigger) ----------------
void startProxMon() {
  apdsSetTapProximityProfile(false);
  Serial.println("# PROX MONITOR — move hand between 2cm and 10cm, press p to exit");
  state = ST_PROXMON;
  lastPollMs = 0;
  proxMonHeaderPrinted = false;
}
void serviceProxMon() {
  if (millis()-lastPollMs < 120) return; lastPollMs = millis();
  uint8_t enable = apdsOk ? apdsRead8(0x80) : 0;
  uint8_t status = apdsOk ? apdsRead8(0x93) : 0;
  uint8_t gconf4 = apdsOk ? apdsRead8(0xAB) : 0;
  uint8_t control = apdsOk ? apdsRead8(0x8F) : 0;
  uint8_t ppulse = apdsOk ? apdsRead8(0x8E) : 0;
  uint8_t config2 = apdsOk ? apdsRead8(0x90) : 0;
  uint8_t p = apdsOk ? apdsProximity() : 0;
  bool pvalid = (status & 0x02) != 0;

  if (!proxMonHeaderPrinted) {
    proxMonHeaderPrinted = true;
    Serial.print("# APDS REG ENABLE=0x"); Serial.print(enable, HEX);
    Serial.print(",GCONF4=0x"); Serial.print(gconf4, HEX);
    Serial.print(",STATUS=0x"); Serial.print(status, HEX);
    Serial.print(",CONTROL=0x"); Serial.print(control, HEX);
    Serial.print(",PPULSE=0x"); Serial.print(ppulse, HEX);
    Serial.print(",CONFIG2=0x"); Serial.println(config2, HEX);
  }
  Serial.print("# PROX PDATA="); Serial.print(p);
  Serial.print(",PVALID="); Serial.println(pvalid ? 1 : 0);
  if (oledOk) {
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1); display.setCursor(0,0); display.println("PROX monitor");
    display.setTextSize(4); display.setCursor(24,18); display.print(p);
    display.setTextSize(1); display.setCursor(0,50); display.print("PVALID:"); display.print(pvalid ? "YES" : "NO");
    display.setCursor(0,58); display.print("press p to exit");
    display.display();
  }
}

// ---------------- commands ----------------
const char *computeSourceText() {
  if (computeSource == COMPUTE_SOURCE_XIAO) return "XIAO";
  if (computeSource == COMPUTE_SOURCE_LOCAL_FALLBACK) return "LOCAL_FALLBACK";
  return "LOCAL_SHADOW";
}

void broadcastCurrentResult() {
  const bool suppressed = rQuality == QUALITY_REPEAT_TEST;
  const char *signal = suppressed ? "INVALID" : rSignalLevel;
  const char *pattern = suppressed ? "RESULT_SUPPRESSED" : rMotorPattern;
  String json = String("{\"type\":\"result\",\"state\":\"done\",\"freq\":") + String(rFreq, 2)
      + ",\"mg\":" + String(rRmsMg, 1)
      + ",\"band_ratio\":" + String(rBandRatio, 3)
      + ",\"signal_level\":\"" + signal + "\""
      + ",\"motor_pattern\":\"" + pattern + "\""
      + ",\"quality\":\"" + qualityLevelText(rQuality) + "\""
      + ",\"quality_reason\":" + String(rQualityReasons)
      + ",\"classification_suppressed\":" + (suppressed ? "true" : "false")
      + ",\"compute_source\":\"" + computeSourceText() + "\""
      + ",\"xiao_link\":\"" + (xiaoLink.online() ? "ONLINE" : "FAILED") + "\""
      + ",\"pads_probability\":" + (rPadsResearchValid ? String(rPadsResearchProbability, 6) : "null") + "}";
  webSocket.broadcastTXT(json);
}

// Send the full (downsampled ~20Hz) waveform once after measurement, replacing the live stream during measurement.
// The waveform is rebuilt directly from the buffered sessionChannelSamples, so the sampling path no longer does any WebSocket send.
void broadcastSessionWaveform() {
  if (webSocket.connectedClients() == 0 || sessSamples == 0) return;
  // Pick the highest-energy accel axis and send the "signed" linear acceleration, so the plot is a normal oscillating waveform.
  // (Previously sending magnitude made it all positive, a one-sided upward curve.)
  double energy[3] = {0, 0, 0};
  for (uint32_t i = 0; i < sessSamples; ++i)
    for (uint8_t a = 0; a < 3; ++a) {
      const float v = sessionChannelSamples[AX_ACC_X + a][i];
      energy[a] += (double)v * v;
    }
  uint8_t axis = AX_ACC_X;
  if (energy[1] > energy[0] && energy[1] >= energy[2]) axis = AX_ACC_Y;
  else if (energy[2] > energy[0] && energy[2] > energy[1]) axis = AX_ACC_Z;
  const uint32_t stride = 10;                       // same ~20Hz density as the old live waveform
  String json; json.reserve(2400);
  json = "{\"type\":\"wave_full\",\"fs\":";
  json += String((float)SAMPLE_HZ / stride, 1);     // effective per-point sample rate (Hz)
  json += ",\"axis\":";
  json += String((int)(axis - AX_ACC_X));           // 0=x,1=y,2=z (debug reference)
  json += ",\"amp\":[";
  bool first = true;
  for (uint32_t i = 0; i < sessSamples; i += stride) {
    if (!first) json += ',';
    json += String(sessionChannelSamples[axis][i], 4);   // signed
    first = false;
  }
  json += "]}";
  webSocket.broadcastTXT(json);
}

void applyXiaoWindowResult(const TremorLink::ResultPayload &result) {
  if (state != ST_MONITOR || result.mode != TremorLink::MODE_MONITOR_WINDOW) return;
  const bool tremorNow = !result.classificationSuppressed &&
                         (result.flags & TremorLink::RESULT_TREMOR_NOW);
  monLastFreq = result.peakFreqHz;
  monLastRmsMg = result.rmsMg;
  if (tremorNow) {
    ++monTremorStreak;
    if (monTremorStreak == 2 && !monActive) {
      monActive = true;
      ++monEpisodes;
      beep(2, 80, 80);
    }
  } else {
    monTremorStreak = 0;
    monActive = false;
  }
  const uint32_t elapsed = (millis() - monStartMs) / 1000;
  Serial.print("# XIAO WINDOW_RESULT,freq_Hz="); Serial.print(result.peakFreqHz, 2);
  Serial.print(",rms_mg="); Serial.print(result.rmsMg, 1);
  Serial.print(",tremor="); Serial.print(tremorNow ? 1 : 0);
  Serial.print(",quality="); Serial.println(result.quality);
  if (monExitHoldStartMs == 0) drawMonitor(monLastFreq, monLastRmsMg, monActive, monEpisodes, elapsed);
  String json = String("{\"type\":\"monitor\",\"active\":") + (tremorNow ? "true" : "false")
      + ",\"episodes\":" + String(monEpisodes) + ",\"elapsed_s\":" + String(elapsed)
      + ",\"freq\":" + String(result.peakFreqHz, 2) + ",\"mg\":" + String(result.rmsMg, 1)
      + ",\"quality\":" + String(result.quality)
      + ",\"compute_source\":\"XIAO\",\"xiao_link\":\"ONLINE\"}";
  webSocket.broadcastTXT(json);
}

void applyXiaoSessionResult(const TremorLink::ResultPayload &result, uint16_t sessionId) {
  const uint32_t uartReason = 1UL << 8;
  if (!(result.flags & TremorLink::RESULT_LINK_COMPLETE) || (result.qualityReasons & uartReason)) {
    Serial.println("# XIAO RESULT REJECTED,reason=LINK_OR_SEQUENCE_ERROR,compute_source=LOCAL_FALLBACK");
    if (sessionId == xiaoAwaitSessionId) {
      xiaoAwaitingResult = false;
      computeSource = COMPUTE_SOURCE_LOCAL_FALLBACK;
      if (xiaoAwaitingGuidedPosture) {
        xiaoAwaitingGuidedPosture = false;
        completeGuidedPostureTest();
      } else {
        drawResult(); saveResultOled(); broadcastCurrentResult();
      }
    }
    return;
  }
  if (sessionId != xiaoAwaitSessionId) return;

  if (result.mode == TremorLink::MODE_GUIDED_POSTURE && xiaoAwaitingGuidedPosture) {
    guidedPostureWindows = result.windowCount;
    guidedPostureTremorWindows = result.tremorWindowCount;
    guidedPosturePersistencePct = result.persistencePct;
    guidedPostureQuality = static_cast<MeasurementQuality>(result.quality > QUALITY_REPEAT_TEST
                                                            ? QUALITY_REPEAT_TEST : result.quality);
    guidedPostureQualityReasons = static_cast<uint16_t>(result.qualityReasons & 0xFFFFU);
    guidedRestScore = result.guidedRestScore;
    guidedTapScore = result.guidedTapScore;
    guidedPostureScore = result.guidedPostureScore;
    guidedCmpi = result.guidedCompositeScore;
    xiaoAwaitingResult = false;
    xiaoAwaitingGuidedPosture = false;
    computeSource = COMPUTE_SOURCE_XIAO;
    Serial.print("# XIAO GUIDED POSTURE APPLIED,windows="); Serial.print(guidedPostureWindows);
    Serial.print(",tremor_windows="); Serial.print(guidedPostureTremorWindows);
    Serial.print(",persistence_pct="); Serial.println(guidedPosturePersistencePct, 1);
    if (result.flags & TremorLink::RESULT_G3_VALID) { for (int gi = 0; gi < 12; ++gi) g3PostFeat[gi] = result.padsFeat[gi]; g3PostValid = true; }
    completeGuidedPostureTest();
    return;
  }

  Serial.print("# XIAO SHADOW_DIFF,freq_Hz="); Serial.print(result.peakFreqHz - rFreq, 3);
  Serial.print(",rms_mg="); Serial.print(result.rmsMg - rRmsMg, 3);
  Serial.print(",band_ratio="); Serial.println(result.bandRatio - rBandRatio, 4);

  rFreq = result.peakFreqHz;
  rRmsMg = result.rmsMg;
  rBandRatio = result.bandRatio;
  rGyro = result.gyroRmsDps;
  rJerk = result.jerkRmsGps;
  rInBand = rFreq >= TREMOR_LO_HZ && rFreq <= TREMOR_HI_HZ;
  rQuality = static_cast<MeasurementQuality>(result.quality > QUALITY_REPEAT_TEST
                                              ? QUALITY_REPEAT_TEST : result.quality);
  rQualityReasons = static_cast<uint16_t>(result.qualityReasons & 0xFFFFU);
  strncpy(rSignalLevelStorage, result.signalLevel, sizeof(rSignalLevelStorage) - 1);
  rSignalLevelStorage[sizeof(rSignalLevelStorage) - 1] = '\0';
  strncpy(rMotorPatternStorage, result.motorPattern, sizeof(rMotorPatternStorage) - 1);
  rMotorPatternStorage[sizeof(rMotorPatternStorage) - 1] = '\0';
  rSignalLevel = rQuality == QUALITY_REPEAT_TEST ? "INVALID" : rSignalLevelStorage;
  rMotorPattern = rQuality == QUALITY_REPEAT_TEST ? "RESULT_SUPPRESSED" : rMotorPatternStorage;
  if (result.mode == TremorLink::MODE_GUIDED_REST && (result.flags & TremorLink::RESULT_G3_VALID)) {
    for (int gi = 0; gi < 12; ++gi) g3RestFeat[gi] = result.padsFeat[gi]; g3RestValid = true;   // rest PADS features returned by XIAO
  }
  rPadsResearchValid = result.mode == TremorLink::MODE_LIFT_HOLD &&
                       rQuality != QUALITY_REPEAT_TEST &&
                       (result.flags & TremorLink::RESULT_PADS_VALID) && isfinite(result.padsProbability);
  rPadsResearchProbability = rPadsResearchValid ? result.padsProbability : NAN;
  computeSource = COMPUTE_SOURCE_XIAO;
  xiaoAwaitingResult = false;

  Serial.print("# XIAO RESULT APPLIED,compute_source=XIAO,quality=");
  Serial.print(qualityLevelText(rQuality));
  Serial.print(",classification_suppressed="); Serial.println(rQuality == QUALITY_REPEAT_TEST ? 1 : 0);
  if (state == ST_GUIDED_REST_DONE) drawGuidedRestComplete(rQuality);
  else { drawResult(); saveResultOled(); }
  broadcastCurrentResult();
  if (rPadsResearchValid) {
    String padsJson = String("{\"type\":\"pads_research_score\",\"model\":\"") + PADS_MODEL_VERSION
        + "\",\"task\":\"LiftHold\",\"probability\":" + String(rPadsResearchProbability, 6)
        + ",\"threshold\":" + String(PADS_MODEL_THRESHOLD, 6)
        + ",\"screen_positive\":" + String(rPadsResearchProbability >= PADS_MODEL_THRESHOLD ? 1 : 0)
        + ",\"compute_source\":\"XIAO\",\"classification_suppressed\":false}";
    webSocket.broadcastTXT(padsJson);
  }
}

void applyXiaoCalibration(const TremorLink::CalibrationResultPayload &result) {
  if (!result.valid || result.quality == TremorLink::LINK_QUALITY_REPEAT_TEST || result.ampSevereMg <= 0) {
    Serial.println("# XIAO CALIBRATION ignored,compute_source=LOCAL_FALLBACK");
    return;
  }
  const float detect = result.ampDetectMg / 1000.0f;
  const float elevated = result.ampModerateMg / 1000.0f;
  const float strong = result.ampSevereMg / 1000.0f;
  if (!validCalibrationThresholds(detect, elevated, strong) ||
      !saveCalibrationToNvs(detect, elevated, strong)) {
    Serial.println("# XIAO CALIBRATION rejected,reason=RANGE_OR_NVS");
    return;
  }
  ampDetect = detect; ampElevated = elevated; ampStrong = strong;
  computeSource = COMPUTE_SOURCE_XIAO;
  Serial.print("# XIAO CALIBRATION SAVED,detect_mg="); Serial.print(result.ampDetectMg, 1);
  Serial.print(",moderate_mg="); Serial.print(result.ampModerateMg, 1);
  Serial.print(",severe_mg="); Serial.println(result.ampSevereMg, 1);
}

void serviceXiaoCompute() {
  xiaoLink.service();
  TremorLink::ResultPayload result{};
  uint16_t resultSessionId = 0;
  bool windowResult = false;
  while (xiaoLink.popResult(result, resultSessionId, windowResult)) {
    if (windowResult) applyXiaoWindowResult(result);
    else applyXiaoSessionResult(result, resultSessionId);
  }
  TremorLink::CalibrationResultPayload calibration{};
  uint16_t calibrationSessionId = 0;
  while (xiaoLink.popCalibration(calibration, calibrationSessionId)) applyXiaoCalibration(calibration);

  if (xiaoAwaitingResult && static_cast<int32_t>(millis() - xiaoResultDeadlineMs) >= 0) {
    xiaoAwaitingResult = false;
    computeSource = COMPUTE_SOURCE_LOCAL_FALLBACK;
    Serial.print("# XIAO LINK ERROR,reason=RESULT_TIMEOUT,session="); Serial.print(xiaoAwaitSessionId);
    Serial.println(",compute_source=LOCAL_FALLBACK");
    if (xiaoAwaitingGuidedPosture) {
      xiaoAwaitingGuidedPosture = false;
      completeGuidedPostureTest();
    } else {
      drawResult(); saveResultOled(); broadcastCurrentResult();
    }
  }
  if (xiaoLink.txOverruns() != xiaoLastReportedOverruns) {
    xiaoLastReportedOverruns = xiaoLink.txOverruns();
    Serial.print("# XIAO LINK ERROR,reason=TX_OVERRUN,count="); Serial.println(xiaoLastReportedOverruns);
  }
}

void handleSerialCmd() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'r') { streamRaw = !streamRaw; Serial.print("# streamRaw="); Serial.println(streamRaw?1:0); }
  else if (c == 'm') { if (state==ST_IDLE||state==ST_RESULT) {
    Serial.println("# manual tremor measurement");
    sessionTask = SESSION_TREMOR_QUANT; liftHoldActive = false; startSession();
  } }
  else if (c == 'l') { if (state==ST_IDLE||state==ST_RESULT) startLiftHoldProtocol(); }
  else if (c == 'c') { if (state==ST_IDLE||state==ST_RESULT) startCalibration(); }
  else if (c == 'x') { if (state==ST_IDLE||state==ST_RESULT) clearStoredCalibration(); }
  else if (c == 't') { if (state==ST_IDLE||state==ST_RESULT) startTapTest(); }   // finger-tap bradykinesia test
  else if (c == 'g') { if (state==ST_IDLE||state==ST_RESULT) startGuidedRestStage(); }
  else if (c == 'p') {                                                            // proximity watch (threshold tuning)
    if (state==ST_PROXMON) { state=ST_IDLE; drawReady(lastProx); }
    else if (state==ST_IDLE||state==ST_RESULT) startProxMon();
  }
  else if (c == 'o') {                                                            // continuous monitoring mode
    if (state==ST_MONITOR) { guidedPostureActive=false; state=ST_IDLE; drawReady(lastProx); }
    else if (state==ST_IDLE||state==ST_RESULT) startMonitor();
  }
  else if (c == 'z') { gravInit = false; Serial.println("# gravity reset"); }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(BAUD);
  xiaoLink.begin(XIAO_UART_RX_PIN, XIAO_UART_TX_PIN);
  Wire.begin(); Wire.setClock(400000);
  // ==== WIFI/WS ADDON: connect to phone hotspot ====
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("# WiFi connecting to \""); Serial.print(STA_SSID); Serial.print("\" ");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("# WiFi OK - open in phone browser  http://"); Serial.println(WiFi.localIP());
    if (MDNS.begin("tremorscope")) Serial.println("# or open http://tremorscope.local");
  } else {
    Serial.println("# WiFi failed: check hotspot name/password; on iPhone enable 'Maximum Compatibility' (2.4GHz)");
  }
  webSocket.begin();
  httpServer.on("/", [](){ httpServer.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  httpServer.begin();
  // ==== WIFI/WS ADDON end ====
  buzzerInit();
  hannPowerSum = 0;
  for (int i = 0; i < FFT_SIZE; i++) {
    hann[i] = 0.5f*(1.0f-cosf(2.0f*PI*i/(FFT_SIZE-1)));
    hannPowerSum += hann[i] * hann[i];
  }
  delay(200);
  loadCalibrationFromNvs();
  bool mpu = mpuBegin();
  apdsOk = apdsBegin();
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) {
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1); display.setCursor(0,0); display.println("Parkinson Tremor");
    display.println("Phase4  self-test");
    display.print("MPU6050 : "); display.println(mpu?"OK":"FAIL");
    display.print("APDS9960: "); display.println(apdsOk?"OK":"absent");
    display.print("OLED    : OK"); display.display(); delay(1500);
  }
  Serial.print("# self-test  MPU6050="); Serial.print(mpu?"OK":"FAIL");
  Serial.print("  APDS9960="); Serial.print(apdsOk?"OK":"absent"); Serial.println("  OLED=OK");
  if (!mpu) { Serial.println("# FATAL: MPU6050 no response. Check cable / try 0x68."); while (1) delay(1000); }
  if (!apdsOk) Serial.println("# WARN: APDS absent - use m to trigger manually, c to calibrate");
  Serial.print("# Phase4 ready. READY: swipe anytime / hold still ");
  Serial.print(MODE_START_HOLD_MS / 1000.0f, 1); Serial.println("s to start current mode");
  Serial.println("# Commands: c=calibrate x=clear-cal m=measure l=LiftHold-ML t=taptest o=monitor g=guided p=prox r=raw z=reset");
  lastPollMs = 0; state = ST_IDLE; drawReady(0);
}
void loop() {
  serviceXiaoCompute();
  webSocket.loop();                        // ==== WIFI/WS ADDON ====
  // Pause HTTP service during measurement/fixed sampling: the TCP send for a web request (especially the 37KB page)
  // would block the main loop and stall sampling, causing high jitter. An already-open dashboard uses WebSocket and is unaffected;
  // a new page load is deferred until measurement ends. Covers ST_SETTLE/ST_MEASURING and Guided fixed sampling.
  const bool samplingCritical = (state == ST_SETTLE || state == ST_MEASURING ||
                                 (state == ST_MONITOR && guidedPostureActive));
  if (!samplingCritical) httpServer.handleClient();
  static State observedState = ST_IDLE;
  if (state != observedState) {
    if (state == ST_IDLE) prepareGestureSelection();
    else apdsSetGestureEngine(false);
    observedState = state;
  }

  uint32_t now = micros();
  bool sampleRan = false;
  switch (state) {
    case ST_IDLE:    serviceIdle();    break;
    case ST_RESULT:  serviceResult();  break;
    case ST_CAL_RESULT: serviceCalResult(); break;
    case ST_TAPTEST: serviceTapTest(); break;
    case ST_PROXMON: serviceProxMon(); break;
    case ST_GUIDED_REST: serviceGuidedRestStage(); break;
    case ST_GUIDED_REST_DONE: serviceGuidedRestCompleteStage(); break;
    case ST_GUIDED_TAP: serviceGuidedTapStage(); break;
    case ST_GUIDED_TAP_DONE: serviceGuidedTapCompleteStage(); break;
    case ST_GUIDED_POSTURE: serviceGuidedPostureStage(); break;
    case ST_GUIDED_POSTURE_SETTLE: serviceGuidedPostureSettleStage(); break;
    case ST_LIFT_HOLD_INSTRUCTION: serviceLiftHoldInstruction(); break;
    case ST_XIAO_WAIT: break;
    case ST_SETTLE: case ST_MEASURING: case ST_CAL_REST: case ST_CAL_TREMOR: case ST_MONITOR:
      if ((int32_t)(now - nextSampleUs) >= 0) {
        int32_t jit = (int32_t)(now - nextSampleUs); if (jit > maxJitterUs) maxJitterUs = jit;
        nextSampleUs += SAMPLE_US;
        if ((int32_t)(micros()-nextSampleUs) >= (int32_t)SAMPLE_US) nextSampleUs = micros()+SAMPLE_US;
        doSample();
        sampleRan = true;
      } break;
  }
  if (state == ST_MONITOR && sampleRan && !guidedPostureActive) serviceMonitorExit();
  handleSerialCmd();
  serviceXiaoCompute();
}
