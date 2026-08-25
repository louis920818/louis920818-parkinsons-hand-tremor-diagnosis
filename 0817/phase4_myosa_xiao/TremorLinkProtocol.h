#pragma once

#include <Arduino.h>

// Binary UART protocol shared by MYOSA and XIAO.
// A packet gets CRC16 appended, then COBS encoding, with 0x00 as the boundary.
namespace TremorLink {

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint32_t UART_BAUD = 460800;
constexpr uint8_t SAMPLES_PER_BLOCK = 10;
constexpr size_t MAX_PAYLOAD_SIZE = 224;
constexpr size_t MAX_DECODED_SIZE = 240;
constexpr size_t MAX_ENCODED_SIZE = 244;

enum MessageType : uint8_t {
  MSG_HELLO = 1,
  MSG_HEARTBEAT = 2,
  MSG_SESSION_BEGIN = 3,
  MSG_SAMPLE_BLOCK = 4,
  MSG_TAP_SUMMARY = 5,
  MSG_SESSION_END = 6,
  MSG_WINDOW_RESULT = 7,
  MSG_SESSION_RESULT = 8,
  MSG_CALIBRATION_RESULT = 9,
  MSG_ERROR = 10
};

enum SessionMode : uint8_t {
  MODE_NONE = 0,
  MODE_CAL_REST = 1,
  MODE_CAL_SHAKE = 2,
  MODE_TREMOR_CAPTURE = 3,
  MODE_LIFT_HOLD = 4,
  MODE_GUIDED_REST = 5,
  MODE_GUIDED_POSTURE = 6,
  MODE_MONITOR_WINDOW = 7
};

enum LinkError : uint8_t {
  LINK_OK = 0,
  LINK_OFFLINE = 1,
  LINK_TX_OVERFLOW = 2,
  LINK_CRC_ERROR = 3,
  LINK_SEQUENCE_GAP = 4,
  LINK_SAMPLE_COUNT = 5,
  LINK_RESULT_TIMEOUT = 6,
  LINK_PROTOCOL_MISMATCH = 7,
  LINK_INTERNAL_ERROR = 8
};

enum SharedQuality : uint8_t {
  LINK_QUALITY_PASS = 0,
  LINK_QUALITY_WARNING = 1,
  LINK_QUALITY_REPEAT_TEST = 2
};

enum SampleFlags : uint8_t {
  SAMPLE_VALID = 1U << 0,
  SAMPLE_CLIPPED = 1U << 1
};

enum ResultFlags : uint8_t {
  RESULT_TREMOR_NOW = 1U << 0,
  RESULT_PADS_VALID = 1U << 1,
  RESULT_LINK_COMPLETE = 1U << 2,
  RESULT_G3_VALID = 1U << 3          // guided rest/posture: padsFeat[12] valid (PADS rec features computed by XIAO)
};

struct __attribute__((packed)) FrameHeader {
  uint8_t version;
  uint8_t type;
  uint16_t payloadLength;
  uint16_t sessionId;
  uint16_t sequence;
};

struct __attribute__((packed)) HelloPayload {
  uint32_t capabilities;
  char firmware[24];
};

struct __attribute__((packed)) SessionBeginPayload {
  uint8_t mode;
  uint8_t reserved;
  uint16_t expectedSamples;
  uint16_t sampleHz;
  float gravityAlpha;
  float ampDetect;
  float ampMild;
  float ampModerate;
  float ampSevere;
  float bandRatioMin;
  float gyroBias[3];
  float gravity[3];
};

struct __attribute__((packed)) RawSample {
  uint32_t timestampUs;
  int16_t accel[3];
  int16_t gyro[3];
  uint8_t flags;
};

struct __attribute__((packed)) SampleBlockPayload {
  uint8_t count;
  RawSample samples[SAMPLES_PER_BLOCK];
};

struct __attribute__((packed)) TapSummaryPayload {
  uint8_t score;
  uint8_t quality;
  uint16_t tapCount;
  float meanIntervalMs;
  float intervalCv;
};

struct __attribute__((packed)) SessionEndPayload {
  uint8_t mode;
  uint8_t reserved;
  uint16_t scheduledSamples;
  uint16_t validSamples;
  uint16_t readFailures;
  uint16_t clippedSamples;
  int32_t maxJitterUs;
  float effectiveFs;
};

struct __attribute__((packed)) ResultPayload {
  uint8_t mode;
  uint8_t quality;
  uint8_t classificationSuppressed;
  uint8_t flags;
  uint32_t qualityReasons;
  float peakFreqHz;
  float rmsMg;
  float bandRatio;
  float gyroRmsDps;
  float jerkRmsGps;
  uint16_t windowCount;
  uint16_t tremorWindowCount;
  float persistencePct;
  int16_t guidedCompositeScore;
  int16_t guidedRestScore;
  int16_t guidedTapScore;
  int16_t guidedPostureScore;
  float padsProbability;
  char signalLevel[16];
  char motorPattern[32];   // must fit "NO_CHARACTERISTIC_TREMOR" (24 chars) + trailing '\0'; the old [24] truncated it to ...TREMO
  float padsFeat[12];      // guided rest/posture: PADS rec features computed by XIAO (for MYOSA's three-task XGBoost; peak,bratio,bpow,rms,grms,spec_ent,peak_sharp,lohi,g_bratio,freq_stab,ac_reg,jerk)
};

struct __attribute__((packed)) CalibrationResultPayload {
  uint8_t quality;
  uint8_t valid;
  uint16_t qualityReasons;
  float restMedianMg;
  float shakeMedianMg;
  float ampDetectMg;
  float ampMildMg;
  float ampModerateMg;
  float ampSevereMg;
};

struct __attribute__((packed)) ErrorPayload {
  uint8_t error;
  uint8_t mode;
  uint16_t detail;
};

static_assert(sizeof(SampleBlockPayload) <= MAX_PAYLOAD_SIZE, "sample block too large");
static_assert(sizeof(ResultPayload) <= MAX_PAYLOAD_SIZE, "result payload too large");

inline uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output, size_t capacity) {
  if (capacity == 0) return 0;
  size_t readIndex = 0;
  size_t writeIndex = 1;
  size_t codeIndex = 0;
  uint8_t code = 1;

  while (readIndex < length) {
    if (writeIndex >= capacity) return 0;
    if (input[readIndex] == 0) {
      output[codeIndex] = code;
      code = 1;
      codeIndex = writeIndex++;
      ++readIndex;
    } else {
      output[writeIndex++] = input[readIndex++];
      ++code;
      if (code == 0xFF) {
        output[codeIndex] = code;
        code = 1;
        codeIndex = writeIndex++;
        if (codeIndex >= capacity) return 0;
      }
    }
  }
  output[codeIndex] = code;
  return writeIndex;
}

inline size_t cobsDecode(const uint8_t *input, size_t length, uint8_t *output, size_t capacity) {
  size_t readIndex = 0;
  size_t writeIndex = 0;
  while (readIndex < length) {
    const uint8_t code = input[readIndex++];
    if (code == 0) return 0;
    for (uint8_t i = 1; i < code; ++i) {
      if (readIndex >= length || writeIndex >= capacity) return 0;
      output[writeIndex++] = input[readIndex++];
    }
    if (code != 0xFF && readIndex < length) {
      if (writeIndex >= capacity) return 0;
      output[writeIndex++] = 0;
    }
  }
  return writeIndex;
}

inline bool sendFrame(Stream &stream, MessageType type, uint16_t sessionId,
                      uint16_t sequence, const void *payload, uint16_t payloadLength) {
  if (payloadLength > MAX_PAYLOAD_SIZE) return false;
  uint8_t decoded[MAX_DECODED_SIZE];
  uint8_t encoded[MAX_ENCODED_SIZE];
  FrameHeader header{PROTOCOL_VERSION, static_cast<uint8_t>(type), payloadLength, sessionId, sequence};
  const size_t bodyLength = sizeof(header) + payloadLength;
  if (bodyLength + sizeof(uint16_t) > sizeof(decoded)) return false;
  memcpy(decoded, &header, sizeof(header));
  if (payloadLength && payload) memcpy(decoded + sizeof(header), payload, payloadLength);
  const uint16_t crc = crc16Ccitt(decoded, bodyLength);
  memcpy(decoded + bodyLength, &crc, sizeof(crc));
  const size_t encodedLength = cobsEncode(decoded, bodyLength + sizeof(crc), encoded, sizeof(encoded));
  if (!encodedLength || stream.availableForWrite() < static_cast<int>(encodedLength + 1)) return false;
  if (stream.write(encoded, encodedLength) != encodedLength) return false;
  return stream.write(static_cast<uint8_t>(0)) == 1;
}

class FrameDecoder {
 public:
  FrameDecoder() : encodedLength_(0), payload_(nullptr), payloadLength_(0), crcErrors_(0), frameErrors_(0) {}

  bool push(uint8_t value) {
    if (value != 0) {
      if (encodedLength_ < sizeof(encoded_)) encoded_[encodedLength_++] = value;
      else {
        encodedLength_ = 0;
        ++frameErrors_;
      }
      return false;
    }
    if (encodedLength_ == 0) return false;
    const size_t decodedLength = cobsDecode(encoded_, encodedLength_, decoded_, sizeof(decoded_));
    encodedLength_ = 0;
    if (decodedLength < sizeof(FrameHeader) + sizeof(uint16_t)) {
      ++frameErrors_;
      return false;
    }
    uint16_t receivedCrc = 0;
    memcpy(&receivedCrc, decoded_ + decodedLength - sizeof(receivedCrc), sizeof(receivedCrc));
    const uint16_t expectedCrc = crc16Ccitt(decoded_, decodedLength - sizeof(receivedCrc));
    if (receivedCrc != expectedCrc) {
      ++crcErrors_;
      return false;
    }
    memcpy(&header_, decoded_, sizeof(header_));
    if (header_.version != PROTOCOL_VERSION ||
        header_.payloadLength != decodedLength - sizeof(FrameHeader) - sizeof(uint16_t)) {
      ++frameErrors_;
      return false;
    }
    payload_ = decoded_ + sizeof(FrameHeader);
    payloadLength_ = header_.payloadLength;
    return true;
  }

  const FrameHeader &header() const { return header_; }
  const uint8_t *payload() const { return payload_; }
  uint16_t payloadLength() const { return payloadLength_; }
  uint32_t crcErrors() const { return crcErrors_; }
  uint32_t frameErrors() const { return frameErrors_; }

 private:
  uint8_t encoded_[MAX_ENCODED_SIZE];
  uint8_t decoded_[MAX_DECODED_SIZE];
  size_t encodedLength_;
  FrameHeader header_{};
  const uint8_t *payload_;
  uint16_t payloadLength_;
  uint32_t crcErrors_;
  uint32_t frameErrors_;
};

}  // namespace TremorLink
