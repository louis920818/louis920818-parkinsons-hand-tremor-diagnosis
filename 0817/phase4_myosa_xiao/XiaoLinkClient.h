#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include "TremorLinkProtocol.h"

// Non-blocking UART transmitter on the MYOSA side.
// The sampling path only puts data into a fixed-size queue; the actual encoding and sending are done by service() during the sampling gaps.
class XiaoLinkClient {
 public:
  explicit XiaoLinkClient(HardwareSerial &serial) : serial_(serial) {}

  void begin(int8_t rxPin, int8_t txPin) {
    serial_.setRxBufferSize(4096);
    serial_.setTxBufferSize(2048);
    serial_.begin(TremorLink::UART_BAUD, SERIAL_8N1, rxPin, txPin);
    queueHello();
  }

  void service() {
    while (serial_.available()) {
      if (decoder_.push(static_cast<uint8_t>(serial_.read()))) processFrame();
    }

    const uint32_t now = millis();
    if (now - lastHelloSentMs_ >= 1000) queueHello();
    if (online_ && now - lastRxMs_ > 3000) {
      online_ = false;
      lastError_ = TremorLink::LINK_OFFLINE;
    }

    // At most two packets per loop, to avoid hogging the sampling schedule for long.
    for (uint8_t i = 0; i < 2 && queueCount_ > 0; ++i) {
      PendingFrame &frame = queue_[queueTail_];
      if (!TremorLink::sendFrame(serial_, frame.type, frame.sessionId, frame.sequence,
                                 frame.payload, frame.payloadLength)) break;
      queueTail_ = (queueTail_ + 1) % TX_QUEUE_DEPTH;
      --queueCount_;
    }
  }

  bool online() const { return online_; }
  uint8_t lastError() const { return lastError_; }
  uint32_t txOverruns() const { return txOverruns_; }
  uint32_t crcErrors() const { return decoder_.crcErrors(); }
  uint32_t frameErrors() const { return decoder_.frameErrors(); }
  uint16_t activeSessionId() const { return activeSessionId_; }

  uint16_t beginSession(const TremorLink::SessionBeginPayload &config) {
    flushCurrentBlock();
    activeSessionId_ = ++nextSessionId_;
    if (activeSessionId_ == 0) activeSessionId_ = ++nextSessionId_;
    blockSequence_ = 0;
    currentBlock_.count = 0;
    sessionOpen_ = queueFrame(TremorLink::MSG_SESSION_BEGIN, activeSessionId_, txSequence_++,
                              &config, sizeof(config));
    if (!sessionOpen_) lastError_ = TremorLink::LINK_TX_OVERFLOW;
    return sessionOpen_ ? activeSessionId_ : 0;
  }

  bool addSample(const TremorLink::RawSample &sample) {
    if (!sessionOpen_) return false;
    if (currentBlock_.count >= TremorLink::SAMPLES_PER_BLOCK && !flushCurrentBlock()) return false;
    currentBlock_.samples[currentBlock_.count++] = sample;
    if (currentBlock_.count >= TremorLink::SAMPLES_PER_BLOCK) return flushCurrentBlock();
    return true;
  }

  bool endSession(const TremorLink::SessionEndPayload &payload) {
    if (!sessionOpen_) return false;
    const bool blockOk = flushCurrentBlock();
    const bool endOk = blockOk && queueFrame(TremorLink::MSG_SESSION_END, activeSessionId_, txSequence_++,
                                             &payload, sizeof(payload));
    sessionOpen_ = false;
    if (!endOk) {
      ++txOverruns_;
      lastError_ = TremorLink::LINK_TX_OVERFLOW;
    }
    return endOk;
  }

  bool sendTapSummary(const TremorLink::TapSummaryPayload &payload) {
    return queueFrame(TremorLink::MSG_TAP_SUMMARY, activeSessionId_, txSequence_++,
                      &payload, sizeof(payload));
  }

  bool popResult(TremorLink::ResultPayload &result, uint16_t &sessionId, bool &windowResult) {
    if (!resultReady_) return false;
    result = result_;
    sessionId = resultSessionId_;
    windowResult = resultWasWindow_;
    resultReady_ = false;
    return true;
  }

  bool popCalibration(TremorLink::CalibrationResultPayload &result, uint16_t &sessionId) {
    if (!calibrationReady_) return false;
    result = calibration_;
    sessionId = calibrationSessionId_;
    calibrationReady_ = false;
    return true;
  }

 private:
  static constexpr uint8_t TX_QUEUE_DEPTH = 10;
  struct PendingFrame {
    TremorLink::MessageType type;
    uint16_t sessionId;
    uint16_t sequence;
    uint16_t payloadLength;
    uint8_t payload[TremorLink::MAX_PAYLOAD_SIZE];
  };

  HardwareSerial &serial_;
  TremorLink::FrameDecoder decoder_;
  PendingFrame queue_[TX_QUEUE_DEPTH]{};
  uint8_t queueHead_ = 0, queueTail_ = 0, queueCount_ = 0;
  TremorLink::SampleBlockPayload currentBlock_{};
  uint16_t txSequence_ = 0, blockSequence_ = 0;
  uint16_t nextSessionId_ = 0, activeSessionId_ = 0;
  bool sessionOpen_ = false, online_ = false;
  uint32_t lastRxMs_ = 0, lastHelloSentMs_ = 0, txOverruns_ = 0;
  uint8_t lastError_ = TremorLink::LINK_OFFLINE;

  TremorLink::ResultPayload result_{};
  uint16_t resultSessionId_ = 0;
  bool resultReady_ = false, resultWasWindow_ = false;
  TremorLink::CalibrationResultPayload calibration_{};
  uint16_t calibrationSessionId_ = 0;
  bool calibrationReady_ = false;

  bool queueFrame(TremorLink::MessageType type, uint16_t sessionId, uint16_t sequence,
                  const void *payload, uint16_t payloadLength) {
    if (payloadLength > TremorLink::MAX_PAYLOAD_SIZE || queueCount_ >= TX_QUEUE_DEPTH) {
      ++txOverruns_;
      lastError_ = TremorLink::LINK_TX_OVERFLOW;
      return false;
    }
    PendingFrame &frame = queue_[queueHead_];
    frame.type = type;
    frame.sessionId = sessionId;
    frame.sequence = sequence;
    frame.payloadLength = payloadLength;
    if (payloadLength && payload) memcpy(frame.payload, payload, payloadLength);
    queueHead_ = (queueHead_ + 1) % TX_QUEUE_DEPTH;
    ++queueCount_;
    return true;
  }

  bool flushCurrentBlock() {
    if (currentBlock_.count == 0) return true;
    if (!queueFrame(TremorLink::MSG_SAMPLE_BLOCK, activeSessionId_, blockSequence_,
                    &currentBlock_, sizeof(currentBlock_))) return false;
    ++blockSequence_;
    currentBlock_.count = 0;
    return true;
  }

  void queueHello() {
    if (queueCount_ >= TX_QUEUE_DEPTH) return;
    TremorLink::HelloPayload hello{};
    hello.capabilities = 0x0000000FUL;
    strncpy(hello.firmware, "myosa-xiao-v1", sizeof(hello.firmware) - 1);
    if (queueFrame(TremorLink::MSG_HELLO, 0, txSequence_++, &hello, sizeof(hello)))
      lastHelloSentMs_ = millis();
  }

  void processFrame() {
    const TremorLink::FrameHeader &header = decoder_.header();
    lastRxMs_ = millis();
    if (header.type == TremorLink::MSG_HELLO) {
      online_ = true;
      lastError_ = TremorLink::LINK_OK;
    } else if ((header.type == TremorLink::MSG_SESSION_RESULT ||
                header.type == TremorLink::MSG_WINDOW_RESULT) &&
               decoder_.payloadLength() == sizeof(TremorLink::ResultPayload)) {
      memcpy(&result_, decoder_.payload(), sizeof(result_));
      resultSessionId_ = header.sessionId;
      resultWasWindow_ = header.type == TremorLink::MSG_WINDOW_RESULT;
      resultReady_ = true;
      online_ = true;
    } else if (header.type == TremorLink::MSG_CALIBRATION_RESULT &&
               decoder_.payloadLength() == sizeof(TremorLink::CalibrationResultPayload)) {
      memcpy(&calibration_, decoder_.payload(), sizeof(calibration_));
      calibrationSessionId_ = header.sessionId;
      calibrationReady_ = true;
      online_ = true;
    } else if (header.type == TremorLink::MSG_ERROR &&
               decoder_.payloadLength() == sizeof(TremorLink::ErrorPayload)) {
      TremorLink::ErrorPayload error{};
      memcpy(&error, decoder_.payload(), sizeof(error));
      lastError_ = error.error;
    }
  }
};
