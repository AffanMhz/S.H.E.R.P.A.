2025-12-30

# Project S.H.E.R.P.A

S.H.E.R.P.A. is a lumbar-mounted wearable utilizing context-aware sensor fusion to detect early markers of Acute Mountain Sickness (AMS), distinguishing healthy exertion from pathological instability (Ataxia) in real-time without cloud reliance.

<p align="center">
<img src="https://raw.githubusercontent.com/NavyStudent2893/S.H.E.R.P.A./refs/heads/patch-1/gait.jpg" width="800"><br/>
<i>Project S.H.E.R.P.A</i>
</p>

**Tags:** `embedded-systems` `esp32` `sensor-fusion` `wearable-technology` `edge-computing` `gait-analysis` `acute-mountain-sickness`
---

> **Your brain may not work at high altitude. S.H.E.R.P.A. does.**


---

## Acknowledgement

We would like to express our deep sense of gratitude to our respected mentor, **Dr. Shams Ul Haq**, for his exemplary guidance, valuable feedback, and constant encouragement throughout the development of this project. His technical insights and support were instrumental in the successful completion of our work.

We also extend our sincere thanks to the **Department of Electronics & Communication Engineering** and the **Department of Electrical Engineering** at **Jamia Millia Islamia, New Delhi**, for providing the necessary infrastructure, resources, and academic environment required to bring this idea to fruition.

Finally, we thank our colleagues and the technical staff for their direct and indirect cooperation during the course of this project.

### Project Team

* **Affan Danish** (Team Lead)
  * Department of Electronics & Communication Engineering
* **Mohd Zahaib Eqbal**
  * Department of Electronics & Communication Engineering
* **Ammar Rashid**
  * Department of Electrical Engineering
* **Mohd Maaz Quraishi**
  * Department of Electronics & Communication Engineering

**Jamia Millia Islamia**

# The Problem Landscape: The Physiology & Detection Gap

High-altitude mountaineering presents a unique dual-threat: the environment is hostile, and the human brain’s ability to detect that hostility is compromised. The current standard of safety relies on reactive measures that are often too slow, or sensor technology that is fundamentally misplaced.

### 1. The "Silent" Threat: Hypoxic Anosognosia
The most dangerous aspect of high-altitude hypoxia is not just the lack of oxygen, but the brain's inability to recognize it. Climbers suffer from **Hypoxic Anosognosia**—a state where cognitive function declines while subjective confidence remains high. A climber often feels "fine" until the moment of total physiological collapse.
* **The Latency Failure:** Current protocols rely on intermittent **SpO2** checks, which only happen when a climber stops or complains. However, **Ataxia** (loss of motor coordination)—the primary clinical precursor to High Altitude Cerebral Edema (HACE)—is a continuous, dynamic symptom that intermittent checks miss entirely.

### 2. The "Exertion vs. Distress" Algorithmic Paradox
Existing detection algorithms are designed for geriatric care or urban environments, not the violent kinetics of mountaineering.
* **The False Positive Loop:** A climber ascending a steep face generates high-energy accelerometry data. Standard algorithms interpret this high variance as instability or a fall.
* **The Result:** Because the device cannot distinguish between **High-Energy Exertion** (healthy climbing) and **High-Variance Instability** (ataxic stumbling), users are plagued by false alarms and eventually disable the safety systems entirely.

### 3. The Kinetic Noise Floor (Wrist & Ankle Limitations)
Smartwatches and peripheral wearables suffer from a fatal flaw in this context: **"Kinetic Noise."**
* **Signal Corruption:** Sensors on the extremities are subject to trekking pole impacts, erratic arm swings, and non-ambulatory movements.
* **Center of Mass Disconnect:** Peripheral sensors cannot isolate the **Center of Mass (CoM)**. To medically distinguish simple fatigue from pathological balance loss, we need to monitor the core, not the limbs.

### 4. The Infrastructure Gap
Mountaineering occurs in **RF-denied environments** (no cellular/WiFi). Systems that rely on cloud-based gait analysis or heavy Machine Learning models are useless. There is a critical lack of **Edge Computing** solutions capable of autonomous, low-latency decision-making without external connectivity.

---

## The Core Thesis
To solve the problems above, we cannot simply "track movement." We must fundamentally restructure how human stability is quantified in 3D space. The need for S.H.E.R.P.A. arises from three specific engineering requirements:

> **"We must decouple the intensity of movement from the coordination of movement."**

### 1. The Need for Multidimensional Signal Fusion
A single metric cannot capture the complexity of a climb. We need a system that separates the signal into three **orthogonal** (non-overlapping) dimensions to solve the "Intensity Trap":
* **Force (Energy):** Is the user working hard?
* **Rhythm (SIRI - Step Irregularity Index):** Is the movement inconsistent or arrhythmic?
* **Balance (SSIRI - Stability Sway Irregularity Index):** Is the vestibular system failing?

**The "Why":** Only by cross-referencing these three can we identify the dangerous quadrant: **Low Energy + High Sway** (a climber who is not moving fast but is struggling to stand).

### 2. The Necessity of Dynamic Gain Scheduling (The "Terrain Paradox")
Human gait is not static; it changes based on gravity vectors. A "stable" walk on flat ground looks mathematically distinct from a "stable" ascent on an incline.
* **The Requirement:** We need a **Barometric Control Loop**. The system must identify the terrain state (Ascent vs. Descent vs. Flat) and automatically adjust the sensitivity (Gain) of the sensors.
* **Scenario:** During a steep ascent, the system must **attenuate** (lower) the Energy sensitivity to ignore the heavy stepping, but **amplify** the Sway sensitivity to catch dangerous lateral drifts.

### 3. The Physiological Imperative (L3 Posterior Placement)
High Signal-to-Noise Ratio (SNR) is not achieved through software alone; it begins with physical location.
* **Maximizing Signal Clarity:** We position the sensor at the **Posterior Lumbar Vertebrae (L3)** because it acts as the functional proxy for the body’s Center of Mass (CoM).
* **Mechanical Filtration:** Unlike anterior placements susceptible to breathing and stomach movements, the L3 vertebrae offers a rigid skeletal anchor, isolating the sensor from soft-tissue noise.

---

## SOLUTION: S.H.E.R.P.A.
**(Surveillance for High-altitude Effects on Reflexes, Performance & Awareness)**
*First Model: **L.U.M.B.A.R.** (Local Unit for Monitoring Balance, Altitude and Rhythm)*

S.H.E.R.P.A. is a standalone, edge-computing safety node designed to act as a **"cognitive watchdog"** for high-altitude mountaineers. Physically, it is a ruggedized sensor unit hard-mounted to the L3 Vertebrae. Functionally, it serves as a continuous **Ataxia Monitor**. Unlike standard fitness trackers that measure output (heart rate, speed), this system measures **control** (balance, rhythm, coordination).

### 1. Orthogonal Analysis Engine
The system normalizes data by computing the **Coefficient of Variation** across three distinct dimensions:
* **EIRI (Energy):** Measures energy’s variance.
* **SIRI (Step):** Measures rhythm consistency (arrhythmia in gait).
* **SSIRI (Sway):** Measures balance retention (gyroscope stability).

By triangulating these three, the system creates a 3D picture of the user's movement, ensuring that a stumble is distinguished from a heavy footstep.

### 2. Barometric Control Loop (Gain Scheduling)
Because a climber moves differently on an incline than on a plateau, the system uses barometric pressure to identify the terrain (Ascent, Descent, Flat) and dynamically "re-weights" the sensors:
* **During Ascent:** The system **attenuates** sensitivity to Energy (knowing high exertion is normal) and **amplifies** the Sway sensor to prioritize balance detection.
* **During Flat Walking:** It **amplifies** the Step sensor to detect "foot drag"—a subtle early sign of ataxia.

This **Gain Scheduling** ensures the system adapts its definition of "danger" based on the environment, drastically reducing false alarms caused by normal physical effort.

### 3. The Interface & Intervention

**Routine Monitoring**
An integrated OLED display periodically cycles through critical metrics—Step Count, Stability Indices, and current Altitude—keeping the user informed without overwhelming them.

**Emergency Intervention**
If the fused "AMS Score" (Acute Mountain Sickness) breaches a critical safety threshold, the system shifts from Monitoring to **Emergency Mode**:
1.  **The Alarm:** A rhythmic buzzer triggers an audible alarm, and the OLED locks onto a high-contrast **"EMERGENCY"** warning displaying the specific AMS Risk Score (constituted using weighted indexes).
2.  **Gesture Acknowledgement:** To prevent panic and verify the user's motor control, the alarm **cannot** be muted by a simple button press. The user must perform a specific **Hand Gesture** over the sensor.
3.  **System Reset:** Once acknowledged, the system silences the alarm and, after a 10-second cool-down, automatically returns to standard monitoring, ensuring the device is ready for the next interval.
### Images
<p align="center">
<img src="https://raw.githubusercontent.com/NavyStudent2893/S.H.E.R.P.A./refs/heads/patch-1/Index.jpg" width="800"><br/>
<i>Energy Index on Display</i>
</p>
<p align="center">
<img src="https://raw.githubusercontent.com/NavyStudent2893/S.H.E.R.P.A./refs/heads/patch-1/Index_2.jpg" width="800"><br/>
<i>Another view of the device</i>
</p>
<p align="center">
<img src="https://raw.githubusercontent.com/NavyStudent2893/S.H.E.R.P.A./refs/heads/patch-1/circuit_Sherpa.jpg" width="800"><br/>
<i>Circuit Diagram</i>
</p>



##  Key Features & Capabilities

# 1. Algorithmic Architecture: The Weighted Multi-Index System
The solution moves beyond simple acceleration thresholding by implementing a **Stochastic Irregularity Engine** on the ESP32. It isolates "instability" from "exertion" by computing the **Coefficient of Variation (CV)** across three orthogonal dimensions of movement.

For each index, the system calculates the Irregularity Score using the ratio of Standard Deviation ($\sigma$) to the Mean ($\mu$) over a sliding window of $N$ samples:

$$Variance (\sigma^2) = \frac{\sum_{i=0}^{N} (A_i - \mu)^2}{N}$$

$$\text{Irregularity Index} = \frac{\sqrt{\sigma^2}}{\mu}$$

This statistical approach normalizes signal noise against the user's current activity level. The three indices are:

* **Energy Irregularity Index (EIRI):** Analyzes exertion magnitude. Here, $A_i$ represents the **Averaged Acceleration Magnitude** over a finite duration. The system uses a Dynamic Mean that updates continuously to adapt to the user's changing pace (e.g., walking vs. hiking).
* **SIRI (Step Irregularity Index):** Quantifies arrhythmic gait patterns. Here, $A_i$ represents the **Time Difference ($\Delta t$)** between consecutive heel strikes. A high score indicates irregular stepping rhythms.
* **SSIRI (Stability Sway Irregularity Index):** Detects vestibular failure and loss of balance. Here, $A_i$ represents the **Gyroscopic Angle Variation ($\theta$)** (side-to-side sway or forward-backward tip). This effectively digitizes the clinical "Tandem Gait Test."

---

# 2. Context-Aware Gain Scheduling (Barometric Control Loop)
To prevent false positives on different terrains, the system uses a **Barometric Pressure Sensor** as a "State Controller."

* **Terrain State Determination:** The system calculates the rate of altitude change over time to classify the user's kinetic state into **ASCENT**, **DESCENT**, or **FLAT_PLATEAU**.
* **Variable Gain Assignment:** Based on the identified state, the system dynamically re-weights the importance of each index ($K$ variables).
    * **Ascent State:** The system **attenuates** $K_{energy}$ (to ignore the high exertion noise of climbing) and **amplifies** $K_{stability}$ (to prioritize balance detection).
    * **Flat State:** The system **amplifies** $K_{step}$ to detect subtle "foot drag" anomalies or micro-stumbles that are indicative of early-onset ataxia.
* **Hypoxic Risk Multiplier:** If the calculated "True Altitude" breaches the **Silent Hypoxia Threshold**, a global risk factor is applied to all sensitivity gains ($K$), making the system hyper-sensitive in dangerous zones.

---

# 3. The AMS Risk Scoring & Feedback Loop
The core decision-making engine aggregates the weighted indices into a real-time **Global AMS Score**.

### The Scoring Formula
$$AMS  Score = (K_{energy} \times EIRI) + (K_{step} \times SIRI) + (K_{stability} \times SSIRI)$$

### Sequential Monitoring Mode
Under normal conditions, the OLED screen cycles through data periodically:
`Step Count` $\rightarrow$ `EIRI` $\rightarrow$ `SIRI` $\rightarrow$ `SSIRI` $\rightarrow$ `Terrain Status` $\rightarrow$ `AMS Score`.

### Emergency Intervention
If the AMS Score exceeds the calibrated **Critical Threshold**, the system triggers the **EMERGENCY Protocol**:
* **Visual:** The screen overrides the cycle to flash **"EMERGENCY! DESCEND DOWN"** along with the current Score.

---

# 4. Physical Implementation & Sensing Strategy
The hardware is designed for autonomy and bio-mechanical accuracy in RF-denied environments.

### Edge Computing Framework
The **ESP32** executes the entire logic loop locally at **50Hz**, eliminating the need for cloud connectivity or smartphone pairing.

### Posterior Lumbar Deployment
The device is mounted on the **Posterior Waist Belt (L3 Vertebrae)**.


* **Rationale:** This ensures direct mechanical coupling with the skeletal **Center of Mass (CoM)**. Unlike anterior (front) placement, this location filters out "Soft Tissue Artifacts" (diaphragmatic breathing and abdominal movement) that corrupt sensor data.

### Detachable Heads-Up Design
While the sensor module remains firmly clipped to the belt for data accuracy, the display unit is designed to be easily detachable or viewable by a partner, allowing climbers to check the sequential data without disrupting the sensor's position.

---

# 5. Gesture-Controlled Interaction (Touchless Interface)
Recognizing that mountaineers wear thick thermal gloves that make small buttons unusable, the device integrates a **Proximity/Gesture Sensor (APDS9960)**.


* **Contactless Alarm Dismissal:** In the event of an emergency alarm, the buzzer continues until the user manually waves their hand over the sensor.
* **Safety Timeout:** Once the gesture is registered, the alarm is silenced, but the screen continues to display the "EMERGENCY" data for an additional **10 seconds** before returning to the normal monitoring cycle. This ensures the user has acknowledged the critical warning. confirms a combination of High Sway + Low Energy (signifying exhaustion-induced collapse).

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
##  Tech Stack & System Architecture

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
## 📦 Requirements & Dependencies

This project consists of two parts: the **Firmware** (running on the ESP32) and the **Visualization Dashboard** (running on a PC).

### 1. Firmware Libraries (C++ / Arduino IDE)
To compile the code for the ESP32, you must install the following libraries via the Arduino Library Manager or by importing the `.zip` files manually:

* **Communication & Core:**
    * `Wire.h` (Standard Arduino I²C)
    * `I2Cdev` (i2cdevlib)
* **Sensors (IMU & Altimeter):**
    * `MPU6050 MotionApps 2.0` (DMP implementation)
    * `Adafruit BMP085 / BMP180 Library`
* **Display (OLED):**
    * `Adafruit GFX Library`
    * `Adafruit SSD1306`

> **Note:** The `MPU6050 MotionApps` and `I2Cdev` libraries often require manual installation. Ensure you use the versions compatible with ESP32 (Jeff Rowberg's `i2cdevlib` is recommended).

---

### 2. Python Dependencies (Live Dashboard & Analysis)
To run the **Live Telemetry Dashboard** or perform post-trek data analysis, you need the following Python libraries installed on your computer:

* **`pyserial`**: Reads live data streams from the ESP32 via USB.
* **`matplotlib`**: Plots real-time graphs for Rhythm, Energy, and Sway indices.
* **`pandas`**: Structures sensor logs into DataFrames for analysis.
* **`numpy`**: Handles statistical variance calculations.

**Quick Install:**
```bash
pip install numpy pandas matplotlib pyserial
```


