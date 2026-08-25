# XIAO Tremor Compute

Compute co-processor firmware for the XIAO ESP32-S3. It receives the raw six-axis MPU6050 data from MYOSA and performs gravity removal, gyro-bias correction, FFT/PSD, quality checks, the LiftHold 15-feature research model, calibration, and the Guided composite score. It does not include Wi-Fi, BLE, or Node-RED.

For flashing settings and wiring, see the [MYOSA + XIAO guide](../phase4_myosa_xiao/README.md).
