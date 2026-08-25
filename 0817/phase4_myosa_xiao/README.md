# MYOSA + XIAO ESP32-S3 dual-board firmware

This folder is the MYOSA-main-board version split out from `phase4_full_wifi.ino`. The original `phase4_full/` is left unchanged so Arduino doesn't compile two `setup()`/`loop()` at once.

## Wiring

| MYOSA main board | XIAO ESP32-S3 | Purpose |
|---|---|---|
| GPIO26 / TX2 | D7 / GPIO44 / RX1 | MYOSA sends raw IMU |
| GPIO27 / RX2 | D6 / GPIO43 / TX1 | XIAO returns compute results |
| GND | GND | common signal ground |

In this first version each board is USB-powered; do not connect the two boards' 5V or 3V3. GPIO25 is still the buzzer; GPIO21/22 are still I2C.

## Arduino IDE settings

MYOSA uses the current ESP32-WROOM-32E / Generic ESP32 settings. XIAO uses:

- Board: `XIAO_ESP32S3`
- Flash: 8 MB
- PSRAM: `OPI PSRAM`
- USB CDC On Boot: Enabled

Both boards' UART is fixed at `460800 / 8N1`. The XIAO does not start Wi-Fi or BLE.

## How it works

- The MPU6050 is still sampled by MYOSA at 200 Hz.
- The raw `int16_t` six-axis data is packed 10 samples at a time and sent with COBS + CRC16-CCITT.
- The XIAO returns FFT, RMS, band ratio, quality, pattern, the LiftHold research model, and the Guided composite score.
- Before the first on-hardware acceptance, MYOSA keeps a parallel local computation as a Shadow baseline; when the OLED/web page is connected normally it uses the XIAO result, and on error it uses the local result. Only after A/B acceptance is the heavy FFT on the normal path turned off, so a trusted fallback baseline is not lost before comparison.
- `REPEAT TEST` always sets `classification_suppressed=true`.
- The XIAO's FFT consistency check adds the same gate as MYOSA — "if peak/RMS < `QUALITY_FFT_MIN_PEAK_RMS_RATIO` (0.75), skip the drift re-measure": when there is no clear periodic peak (broadband noise) it does not flag `REPEAT TEST` on frequency drift, avoiding a healthy person's slight shake being misjudged and keeping the XIAO's and MYOSA shadow's quality decisions consistent. **This is a XIAO compute change and requires re-flashing the XIAO.**
- On XIAO disconnect, sequence/CRC error, TX overrun, or result timeout, MYOSA completes the fallback using the same batch of locally-buffered data.
- During a measurement (TREMOR / LiftHold) it **no longer streams the live waveform point-by-point**: sending WebSocket synchronously in the main loop during sampling stalls the sampling, causing high jitter and a `REPEAT TEST`. Instead, after the measurement ends and the result is sent, `broadcastSessionWaveform()` rebuilds it directly from the buffered `sessionChannelSamples` and sends the full waveform in a single `wave_full` message (downsampled to ~20 Hz). Continuous monitoring (MONITOR) keeps the original live `wave` stream.
- During measurement and Guided fixed-sampling (`ST_SETTLE` / `ST_MEASURING` / Guided posture) it **pauses `httpServer.handleClient()`**: the TCP write to serve the 37 KB page while handling an HTTP request would block the main loop and stall sampling. An already-open dashboard uses WebSocket and is unaffected; a new page load during a measurement is deferred until the measurement ends. `webSocket.loop()` is kept throughout (lightweight when idle, does not affect sampling timing).
- WebSocket adds `compute_source`, `xiao_link`, `quality_reason`, `classification_suppressed`, `external_validation_required`, `not_diagnosis`, and `wave_full` (the full measurement waveform, with `fs`, `axis`, and `amp[]`). On receiving `wave_full`, the web page rebuilds the waveform chart, spectrum, and CSV-export data in one go.
- `wave_full` sends the "highest-energy acceleration axis, signed" linear acceleration (not magnitude), so the plot is a normal oscillating waveform rather than a one-sided curve; the web waveform now fills the full canvas width.
- Fixed `ResultPayload.motorPattern` and MYOSA's `rMotorPatternStorage` buffers, enlarged from `[24]` to `[32]`: `"NO_CHARACTERISTIC_TREMOR"` is 24 characters, so with the trailing `\0` it needs 25 bytes; the old size truncated it to `...TREMO`, so the web page couldn't match the label and showed the raw string. **This is a protocol-struct change; both boards must be re-flashed together.**

## Recommended flashing order

1. Flash `xiao_tremor_compute/xiao_tremor_compute.ino` first.
2. The XIAO USB Serial should show `buffers=OK`.
3. Connect RX/TX/GND per the table, then flash this folder's `phase4_myosa_xiao.ino`.
4. MYOSA Serial should receive HELLO every second, and `XIAO SESSION BEGIN` should appear during a measurement.
5. First use `m` for a rest and ~5 Hz test, confirming `XIAO SHADOW_DIFF` is within the acceptance range.

## Acceptance checklist

- 2048-sample measurement: TX overrun, CRC error, and sequence gap are all 0.
- Frequency difference ≤ 0.391 Hz; RMS difference ≤ 1% or 1 mg; band-ratio difference ≤ 0.02.
- After unplugging the XIAO the OLED shows `LOCAL FALLBACK` and the measurement still completes.
- On `REPEAT TEST`, the OLED, USB Serial, and web page must not show a trusted classification.
- The LiftHold model is a research output only; it must continuously state that it needs external validation by MYOSA and is not a diagnosis.

Both sketches have been compiled with the `arduino-cli` built into the Arduino IDE; the dual-board wiring and on-hardware test in the list above must still be completed before submission.
