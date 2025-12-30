2025-12-30

# Project S.H.E.R.P.A

S.H.E.R.P.A. is a lumbar-mounted wearable utilizing context-aware sensor fusion to detect early markers of Acute Mountain Sickness (AMS), distinguishing healthy exertion from pathological instability (Ataxia) in real-time without cloud reliance.

[!P1](link)

**Tags:** `embedded-systems` `esp32` `sensor-fusion` `wearable-technology` `edge-computing` `gait-analysis` `acute-mountain-sickness`
---

> **Your brain may not work at high altitude. S.H.E.R.P.A. does.**


---

## Acknowledgements

## Overview

## Demo / Examples

### Images

### Videos

##  Key Features & Capabilities

### 1.  Context-Aware Gain Scheduling (The "Barometric Supervisor")
Standard wearables fail on mountains because they mistake healthy exertion (heavy climbing) for distress. S.H.E.R.P.A. solves this "Terrain Paradox":
* **Terrain Recognition:** Uses the **BMP180** to calculate altitude and identify the climber's state: **Ascent**, **Descent**, or **Flat Plateau**.
* **Dynamic Sensitivity:** automatically adjusts the "Gain" (sensitivity) of the detection algorithms:
    * **Ascent:** Attenuates (lowers) Energy sensitivity to ignore the "noise" of hard climbing, while amplifying Sway detection.
    * **Flat/Plateau:** Maximizes sensitivity to Rhythm irregularities to detect the subtle "foot drag" of **Silent Hypoxia**.

### 2. Orthogonal Gait Analysis (Triple-Index System)
Instead of a single "movement score", the system decomposes motion into three non-overlapping dimensions to isolate specific pathologies:
* **EIRI (Energy Irregularity Index):** Measures the *Force* of movement. It filters out high-energy output (healthy) from low-energy stumbling (hypoxic).
* **SIRI (Step Irregularity Index):** Measures the *Rhythm*. It calculates the variance ($\sigma$) in step-to-step timing to identify **Ataxic Gait** (irregular intervals).
* **SSIRI (Stability Sway Irregularity Index):** Measures *Balance*. Using the Gyroscope, it quantifies lateral (side-to-side) deviation, detecting the failure of the Tandem Gait (walking in a straight line)

### 3.  L.U.M.B.A.R. Physical Filtering Strategy
Placement is a feature, not just a detail. By mounting on the **L3 Vertebrae** (Posterior Waist Belt):
* **Center of Mass (CoM) Acquisition:** Captures pure skeletal movement rather than limb artifacts.
* **Mechanical Noise Filtering:** The spine-coupled placement naturally filters out "Kinetic Noise" (arm swings from trekking poles) and "Soft Tissue Artifacts" (breathing/stomach movement) that confuse wrist-based sensors.

### 4.  Autonomous Edge Computing (RF-Denied Operation)
Designed for the "Death Zone" where cloud connectivity is impossible.
* **Zero Latency:** The **ESP32** dual-core MCU processes the **50Hz** sensor loop locally.
* **Privacy First:** No GPS tracking or data transmission; all analysis is performed on-device.

### 5.   Intuitive Haptic Feedback Loop
Communicates critical safety data without requiring visual attention (hands-free).
* **Pre-Ataxic Warnings:** Distinct vibration patterns warn the user when Step Irregularity (SIRI) deviates from the calibrated baseline.
* **Emergency Interventions:** Continuous alarms trigger only when the **Global Risk Score** confirms a combination of High Sway + Low Energy (signifying exhaustion-induced collapse).

##  Usage & Deployment Instructions


### 1. Device Mounting & Initialization
* **Placement:** Securely clip the **S.H.E.R.P.A.** unit to the center of the Posterior Waist Belt (Lower Back / L3 Vertebrae region).
* **Orientation:** Ensure the OLED screen is facing outward and the sensor stack is pressed firmly against the lumbar region to minimize mechanical vibration.
* **Power On:** Connect the power source. The device will emit a **single beep** to confirm initialization.

### 2. The Calibration Phase (Critical)
Upon startup, the screen will display `CALIBRATING...`.
* **Action:** For the first **30 seconds**, the user must walk at a normal, comfortable pace on flat ground to establish the baseline gait and sway variance.

### 3. Emergency Protocol & Gesture Control
* **Trigger:** If the Global AMS Score exceeds the calculated safety threshold, the device enters **EMERGENCY MODE**. The buzzer activates a continuous alarm, and the screen flashes `EMERGENCY! DESCEND`.
* **Silence Alarm (Touchless):**
    * **Action:** Wave your hand (even with thick gloves) within **5cm** of the Proximity Sensor to temporarily silence the buzzer.

## Tech Stack
## 🛠️ Tech Stack & System Architecture

### 1. Hardware Layer (The Edge Node)
Designed for autonomous operation in **RF-denied environments** (via local Edge Computing) and optimized for **high-fidelity stability analysis**

| Component | Specification | Role in S.H.E.R.P.A |
| :--- | :--- | :--- |
| **MCU** | **ESP32** (Dual Core) | Edge computing core handling the **50Hz** processing loop and autonomous decision-making without cloud reliance. |
| **IMU** | **6-Axis Accelerometer & Gyroscope** | Captures raw skeletal data for **SIRI** (Rhythm) and **SSIRI** (Balance) calculations. |
| **Altimeter** | **BMP-180 Barometric Pressure Sensor** | Provides **Raw Pressure Data** used to calculate Altitude and  drive the **Terrain State Control Loop**. |
| **Feedback** | **Buzzer Module** | Delivers the "Descend Alert" interrupt directly to the user upon pathological score validation. |
| **Mounting** | **L3 Lumbar Belt** | Mechanical coupling to the **L3 Vertebrae** to filter kinetic artifacts (arm swing) and soft-tissue noise. |

### 2. Firmware & Embedded Systems
* **Language:** C++ / Embedded C
* **Core Architecture:** Real-time loop processing (**50Hz**) with strictly local computation.
* **Signal Processing:**
    ```Affan will add```

### 3. The Algorithmic Engine (Stochastic Irregularity Engine)
The core logic implements a **Context-Aware Sensor Fusion** model that moves beyond simple thresholding by decoupling intensity from coordination.

* **Fusion Strategy:** Weighted Multi-Index System using three orthogonal dimensions:
    * **`EIRI` (Energy Irregularity Index):** Analyzes exertion magnitude (Force).
    * **`SIRI` (Step Irregularity Index):** Quantifies arrhythmic gait patterns via step-to-step timing intervals (Rhythm).
    * **`SSIRI` (Stability Sway Irregularity Index):** Uses Gyroscopic data to detect lateral sway and vestibular failure (Balance).

* **Control Logic:** **Dynamic Gain Scheduling (Barometric Control Loop)**
    * **Terrain State Determination:** Identifies user state (Ascent, Descent, Flat Plateau).
    * **Variable Gain Assignment:**
        * *Ascent:* Attenuates Energy Gain (exertion noise) / Amplifies SSIRI Gain (balance).
        * *Flat:* Amplifies SIRI Gain (detects "foot drag" indicative of Silent Hypoxia).
    * **Hypoxic Risk Multiplier:** Applies a global risk factor when "True Altitude" breaches safety thresholds.
<p align="center">
  <img src="https://raw.githubusercontent.com/NavyStudent2893/S.H.E.R.P.A./refs/heads/main/Indices_calculation_logic.png" alt="Indices Calculation Logic" width="800">
  <br>
  <em>Figure 1: The Context-Aware Sensor Fusion Architecture.</em>
</p>
 

## Requirements / Installation

## File Structure (Optional)

## License (Optional)

## Contribution Notes (Optional)
