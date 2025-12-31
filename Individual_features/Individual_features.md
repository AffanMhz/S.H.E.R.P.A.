# S.H.E.R.P.A. Feature Test Modules

## Overview

This directory contains the unit test codes for the **S.H.E.R.P.A.** (Smart High-altitude Early Risk Prediction Assistant) project.

These modular files are designed to isolate and verify specific hardware components and algorithmic features before they are integrated into the main system. Use these codes to debug individual sensors or tune specific thresholds without the complexity of the full combined firmware.

## Module Descriptions

### 1. Altitude & Environment
- **File:** `altitude.ino`
- **Hardware:** BMP180 / BMP085 Barometric Sensor
- **Purpose:**
    - Tests the reading of barometric pressure to calculate:
        - Relative Altitude (height gained since start).
        - Vertical Speed (ascent/descent rate in m/s).
    - Verifies the Low-Pass Filter (smoothing) logic.

### 2. Gait & Rhythm Analysis
- **File:** `pedometer.ino`
- **Hardware:** MPU6050 (IMU)
- **Purpose:**
    - Validates the core step-detection engine.
    - Tests the "Irregularity Index" (Step Jitter).
    - Verifies energy thresholds for distinguishing steps from noise.

- **File:** `Arythmic_walk.ino`
- **Hardware:** MPU6050 + Buzzer
- **Purpose:**
    - A specialized test for the "Burst Detection" feature.
    - Triggers an alert specifically when rapid, arrhythmic steps (panic walking) are detected.

### 3. Stability & Posture
- **File:** `SWAY_test.ino`
- **Hardware:** MPU6050 (IMU)
- **Purpose:**
    - Tests the Sway Detection algorithm.
    - Calibrates the lumbar roll baseline.
    - Detects lateral instability (Ataxia) by correlating Gyroscope Energy with Roll Angle deviation.

### 4. User Interface & Control
- **File:** `gesture_logic.ino`
- **Hardware:** APDS9960 Gesture Sensor
- **Purpose:**
    - Verifies the non-contact control interface.
    - Tests sensor initialization.
    - Confirms detection of hand gestures (used for dismissing alarms).

### 5. Hardware Alerts
- **File:** `BUZZER_TEST.ino`
- **Hardware:** Active Buzzer (GPIO 12)
- **Purpose:**
    - Simple hardware verification to ensure the buzzer circuit is functioning correctly (ON/OFF logic).

---
**Team Hardcoders | S.H.E.R.P.A. Project 2025**
