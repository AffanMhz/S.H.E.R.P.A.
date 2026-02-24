/*
 * ======================================================================================
 * Project:        S.H.E.R.P.A. (Smart High-altitude Early Risk Prediction Assistant)
 * ======================================================================================
 * Team:           Hardcoders
 * ---------------------------------------------------------------------------------------
 * TEAM MEMBERS & ROLES:
 * 1. Affan Danish    | Lead Firmware Architect
 * 2. Zahaib          | Project Specialist
 * 3. Ammar           | Systems Engineer
 * 4. Maaz            | Project Design
 * ---------------------------------------------------------------------------------------
 * Platform:       MYOSA (MakeSense EduTech / Pegasus Automation)
 * Component:      Multisensor Monitoring & Early-Risk Alert System (FreeRTOS Edition)
 * Focus:          Gait irregularity, sway detection, ascent profiling,
 *                 and contextual alerts related to Acute Mountain Sickness (AMS)
 * ======================================================================================
 *
 * ── HARDWARE & INTERFACES (MYOSA PLATFORM) ───────────────────────────────────────────
 *
 *  IMU Sensor:     MPU6050 (6-DOF Accelerometer + Gyroscope)
 *                  I2C Address : 0x69
 *
 *  Barometric:     BMP180 (Bosch Barometric Pressure Sensor)
 *                  I2C Address : 0x77
 *                  Pressure Range : 300–1100 hPa
 *                  Resolution     : ~0.25 m altitude
 *
 *  Display:        SSD1306 OLED Display (128×64)
 *                  I2C Address : 0x3C
 *
 *  Gesture Sensor: APDS9960 (Gesture / Proximity / Light)
 *                  I2C Address : 0x39
 *
 *  Alert Output:   Active Buzzer (GPIO-controlled)
 *
 * ── SYSTEM PURPOSE ───────────────────────────────────────────────────────────────────
 *
 *  S.H.E.R.P.A. is a wearable, real-time multisensor system designed to provide
 *  early, objective indicators associated with altitude stress and movement
 *  instability during high-altitude ascent.
 *
 *  The system combines:
 *    - Barometric ascent rate & relative altitude tracking
 *    - Step rhythm and energy irregularity analysis
 *    - Roll-based sway detection with gyro confirmation
 *    - Context-aware alerting with visual and auditory feedback
 *
 *  The goal is not medical diagnosis, but early risk awareness to support timely
 *  decisions such as rest, pace adjustment, or descent.
 *
 * ── SCIENTIFIC CONTEXT ───────────────────────────────────────────────────────────────
 *
 *  Acute Mountain Sickness (AMS) affects approximately 25–50% of individuals
 *  ascending above 2,500 meters, with 1–2% progressing to severe conditions
 *  such as High-Altitude Cerebral Edema (HACE).
 *
 *  Clinical research indicates that objective physiological and behavioral markers
 *  often precede subjective symptoms by 30–60 minutes. This window is critical
 *  for preventive intervention.
 *
 *  Unlike conventional altitude trackers that passively display elevation,
 *  S.H.E.R.P.A. integrates environmental and motion-derived indicators to surface
 *  early warning patterns in real time.
 *
 * ── RTOS ARCHITECTURE ────────────────────────────────────────────────────────────────
 *
 *  ┌─────────────────────┬──────────┬─────────┬──────────────────────────────────────┐
 *  │ Task                │ Priority │ Period  │ Responsibility                       │
 *  ├─────────────────────┼──────────┼─────────┼──────────────────────────────────────┤
 *  │ TaskSensorMPU       │    5     │  10 ms  │ DMP polling, gait, sway, alert logic │
 *  │ TaskProcessAlert    │    4     │  20 ms  │ Buzzer toggling, alert expiry         │
 *  │ TaskSensorBMP       │    3     │ 100 ms  │ Altitude LPF, vertical speed, status │
 *  │ TaskGestureComm     │    2     │  50 ms  │ Gesture dismiss, Serial comms        │
 *  │ TaskDisplay         │    1     │ 100 ms  │ Screen cycling (Gait/Altitude/Alert) │
 *  └─────────────────────┴──────────┴─────────┴──────────────────────────────────────┘
 *
 *  Shared State Protection : Single mutex (xSharedMutex) guards all shared variables
 *  Blocking Calls          : All delay() replaced with vTaskDelayUntil()
 *  Timing Integrity        : All millis()-based logic preserved inside tasks
 *
 * ── DEVELOPMENT NOTES ────────────────────────────────────────────────────────────────
 *
 *  - All signal processing performed onboard
 *  - Low-pass filtering applied to altitude and ascent rate
 *  - Event-based alerts prioritized over raw data display
 *  - Designed for low cognitive load in hypoxic environments
 *  - FreeRTOS port maintains 100% functional parity with original implementation
 *
 * Date:           December 2025 (FreeRTOS Port)
 * ======================================================================================
 */


 
// ─────────────────────────────────────────────
//  INCLUDES
// ─────────────────────────────────────────────
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include <LightProximityAndGesture.h>

// ─────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────
#define INTERRUPT_PIN 2
#define BUZZER_PIN    2

// ─────────────────────────────────────────────
//  DISPLAY CONFIG
// ─────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ─────────────────────────────────────────────
//  SENSOR OBJECTS
// ─────────────────────────────────────────────
MPU6050              mpu(0x69);
Adafruit_BMP085      bmp;
LightProximityAndGesture Lpg;

// ─────────────────────────────────────────────
//  RTOS SYNCHRONISATION PRIMITIVES
// ─────────────────────────────────────────────
SemaphoreHandle_t xSharedMutex;   // Protects all shared variables below

// ─────────────────────────────────────────────
//  SHARED STATE  (always access under xSharedMutex)
// ─────────────────────────────────────────────

// ── Altitude / BMP ──────────────────────────
float baselineAltitude        = 0.0f;
float smoothedAltitude        = 0.0f;
const float alpha             = 0.1f;
float prevAltitude            = 0.0f;
unsigned long bmpPrevTime     = 0;
float verticalSpeedRaw        = 0.0f;
float verticalSpeedFiltered   = 0.0f;
const float beta              = 0.25f;
float relativeHeight          = 0.0f;
String statusStr              = "STABLE";
const float MOVEMENT_THRESHOLD = 0.3f;

// ── MPU / DMP ────────────────────────────────
bool     dmpReady             = false;
uint8_t  mpuIntStatus;
uint8_t  devStatus;
uint16_t packetSize;
uint8_t  fifoBuffer[64];
volatile bool mpuInterrupt    = false;

Quaternion    q;
VectorInt16   aa;
VectorInt16   aaReal;
VectorInt16   gy;
VectorFloat   gravity;
float         ypr[3];

// ── Rhythm / Gait ────────────────────────────
#define RHYTHM_IRREG_LIMIT        0.45f
#define IRREGULAR_STEPS_REQUIRED  3
#define MIN_STEPS_BEFORE_ALERT    6
#define COMPOSITE_THRESHOLD       12000
#define STEP_MIN_DELAY_MS         300

int           irregularStepCounter = 0;
unsigned long lastStepTime         = 0;
int           stepCount            = 0;
int           stepSignal           = 0;
long          lastInterval         = 0;
long          irregularityIndex    = 0;
float         energySum            = 0.0f;
int           energySamples        = 0;
float         prevStepEnergy       = 0.0f;
float         energyIrregularity   = 0.0f;
float         compositeSignal      = 0.0f;

// ── Burst Detection ──────────────────────────
#define BURST_THRESHOLD_MS  1300
unsigned long stepTimestamps[3]    = {0, 0, 0};
bool          burstDetected        = false;

// ── Sway Detection ───────────────────────────
#define MIN_TILT_TRIGGER    9.0f
#define MIN_GYRO_TRIGGER    8.0f
#define HARD_TILT_LIMIT    35.0f
#define HARD_GYRO_LIMIT    40.0f
#define CONFIRM_MS          80

float         gyroEnergy           = 0.0f;
float         rollBaseline         = 0.0f;
unsigned long swayStart            = 0;
bool          swayDetected         = false;
float         rollDelta            = 0.0f;

#define IRREG_WINDOW  20
float rollBuffer[IRREG_WINDOW];
int   rollIndex                    = 0;
bool  rollBufferFilled             = false;
float rollIrregularity             = 0.0f;
#define IRREGULARITY_LIMIT  3.0f

bool calibrationDone               = false;

// ── Alert System ─────────────────────────────
bool          alertActive          = false;
unsigned long alertEndTime         = 0;
unsigned long buzzerTimer          = 0;
bool          buzzerState          = false;
int           alertReason          = 0;  // 0=None 1=Sway 2=Irregular 3=Burst
#define ALERT_DURATION_MS  3000

// ─────────────────────────────────────────────
//  ISR
// ─────────────────────────────────────────────
void dmpDataReady() {
    mpuInterrupt = true;
}

// ─────────────────────────────────────────────
//  FORWARD DECLARATIONS – display helpers
// ─────────────────────────────────────────────
void centerText(String text, int y, int size);
void updateCombinedDisplay();
void updateAltitudeDisplay();
void drawAlertScreen();

// ─────────────────────────────────────────────
//  TASK FORWARD DECLARATIONS
// ─────────────────────────────────────────────
void TaskSensorBMP    (void *pvParameters);
void TaskSensorMPU    (void *pvParameters);
void TaskGestureComm  (void *pvParameters);
void TaskProcessAlert (void *pvParameters);
void TaskDisplay      (void *pvParameters);

// ═════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════
void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(115200);

    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    delay(100);

    // ── Display Init ──────────────────────────
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for (;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println(F("Init Sensors..."));
    display.display();

    // ── BMP Init ──────────────────────────────
    if (!bmp.begin()) {
        display.println(F("BMP ERROR! Check wiring."));
    } else {
        display.println(F("BMP OK"));
    }
    display.display();

    // ── Gesture Sensor Init (2-sec timeout) ───
    display.println(F("Init Gesture..."));
    display.display();

    unsigned long gestureStart = millis();
    bool gestureFound = false;
    while (millis() - gestureStart < 2000) {
        if (Lpg.begin()) {
            gestureFound = true;
            Serial.println("Gesture Sensor Connected");
            break;
        }
        delay(100);
    }
    if (gestureFound) {
        Serial.println("APDS9960 init completed");
        if (Lpg.enableGestureSensor(ENABLE)) {
            Serial.println("Gesture sensor running");
        } else {
            Serial.println("Gesture sensor init failed!");
        }
    } else {
        Serial.println("Gesture Sensor Disconnected - Skipping");
        display.println(F("Gesture Failed"));
        display.display();
        delay(500);
    }

    // ── MPU / DMP Init ────────────────────────
    mpu.initialize();
    if (mpu.testConnection() == false) {
        display.println(F("MPU Failed!"));
        display.display();
        while (true);
    }

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady     = true;
        packetSize   = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Init failed"));
        while (1);
    }

    // ── Calibration Screen ────────────────────
    display.clearDisplay();
    centerText("CALIBRATING", 20, 1);
    centerText("Keep Still",  40, 1);
    display.display();

    // BMP baseline
    float sum = 0;
    for (int i = 0; i < 50; i++) {
        sum += bmp.readAltitude();
        delay(20);
    }
    baselineAltitude  = sum / 50.0f;
    smoothedAltitude  = baselineAltitude;
    prevAltitude      = smoothedAltitude;
    bmpPrevTime       = millis();

    // MPU roll baseline
    long  rSum = 0;
    int   validSamples = 0;
    mpu.resetFIFO();
    while (validSamples < 100) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            rSum += (long)(ypr[2] * 180.0f / M_PI);
            validSamples++;
            delay(5);
        }
    }
    rollBaseline   = rSum / 100.0f;
    calibrationDone = true;
    display.clearDisplay();
    display.display();

    // ── Create RTOS Mutex ─────────────────────
    xSharedMutex = xSemaphoreCreateMutex();

    // ── Create Tasks ──────────────────────────
    // Priority 1 (lowest) → Display
    // Priority 2           → Gesture / Communication
    // Priority 3           → BMP Altitude
    // Priority 4           → Processing / Alerts
    // Priority 5 (highest) → MPU Sensor (time-critical DMP)

    xTaskCreate(TaskSensorBMP,    "BMP",     256, NULL, 3, NULL);
    xTaskCreate(TaskSensorMPU,    "MPU",     512, NULL, 5, NULL);
    xTaskCreate(TaskGestureComm,  "Gesture", 256, NULL, 2, NULL);
    xTaskCreate(TaskProcessAlert, "Alert",   256, NULL, 4, NULL);
    xTaskCreate(TaskDisplay,      "Display", 512, NULL, 1, NULL);

    // Scheduler starts – setup() returns normally
}

// ═════════════════════════════════════════════
//  LOOP  (unused – FreeRTOS scheduler takes over)
// ═════════════════════════════════════════════
void loop() {
    // Intentionally empty – all work done in tasks
    vTaskDelay(portMAX_DELAY);
}

// ═════════════════════════════════════════════
//  TASK 1 – BMP ALTITUDE SENSOR  (Priority 3)
//  Period: ~100 ms poll; speed recalc every 250 ms
// ═════════════════════════════════════════════
void TaskSensorBMP(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);

    for (;;) {
        unsigned long now    = millis();
        float         rawAlt = bmp.readAltitude();

        if (xSemaphoreTake(xSharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            smoothedAltitude = alpha * rawAlt + (1.0f - alpha) * smoothedAltitude;
            relativeHeight   = smoothedAltitude - baselineAltitude;

            float dt = (now - bmpPrevTime) / 1000.0f;
            if (dt > 0.25f) {
                verticalSpeedRaw      = (smoothedAltitude - prevAltitude) / dt;
                verticalSpeedFiltered = beta * verticalSpeedRaw + (1.0f - beta) * verticalSpeedFiltered;

                if      (verticalSpeedFiltered >  MOVEMENT_THRESHOLD) statusStr = "ASCENDING";
                else if (verticalSpeedFiltered < -MOVEMENT_THRESHOLD) statusStr = "DESCENDING";
                else                                                   statusStr = "STABLE";

                prevAltitude = smoothedAltitude;
                bmpPrevTime  = now;
            }
            xSemaphoreGive(xSharedMutex);
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ═════════════════════════════════════════════
//  TASK 2 – MPU / DMP SENSOR  (Priority 5)
//  Polls DMP FIFO as fast as data arrives (~10 ms)
// ═════════════════════════════════════════════
void TaskSensorMPU(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);

    for (;;) {
        if (!dmpReady) {
            vTaskDelayUntil(&xLastWake, xPeriod);
            continue;
        }

        // Local copies of DMP output (obtained outside the mutex to minimise lock time)
        Quaternion    lq;
        VectorInt16   laa, laaReal, lgy;
        VectorFloat   lgravity;
        float         lypr[3];
        bool          gotPacket = false;

        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&lq, fifoBuffer);
            mpu.dmpGetGravity(&lgravity, &lq);
            mpu.dmpGetYawPitchRoll(lypr, &lq, &lgravity);
            mpu.dmpGetAccel(&laa, fifoBuffer);
            mpu.dmpGetLinearAccel(&laaReal, &laa, &lgravity);
            mpu.dmpGetGyro(&lgy, fifoBuffer);
            gotPacket = true;
        }

        if (gotPacket) {
            // ── Compute derived values locally ──
            float verticalAcc   = (laaReal.x * lgravity.x) + (laaReal.y * lgravity.y) + (laaReal.z * lgravity.z);
            float linearMag     = sqrt(pow(laaReal.x, 2) + pow(laaReal.y, 2) + pow(laaReal.z, 2));
            float linearVertical = fabs(verticalAcc);
            float rhythmGyroPower = sqrt(pow(lgy.x, 2) + pow(lgy.y, 2) + pow(lgy.z, 2));

            float lCompositeSignal = (linearMag * 1.0f) + (linearVertical * 1.5f) + ((rhythmGyroPower / 20.0f) * 0.5f);

            float lRoll  = lypr[2] * 180.0f / (float)M_PI;
            float lgx    = lgy.x / 16.4f;
            float lgy_v  = lgy.y / 16.4f;
            float lGyroMag = sqrt(lgx * lgx + lgy_v * lgy_v);

            if (xSemaphoreTake(xSharedMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
                // Publish raw sensor containers
                q        = lq;
                aa       = laa;
                aaReal   = laaReal;
                gy       = lgy;
                gravity  = lgravity;
                ypr[0]   = lypr[0];
                ypr[1]   = lypr[1];
                ypr[2]   = lypr[2];

                compositeSignal  = lCompositeSignal;
                energySum       += lCompositeSignal * lCompositeSignal;
                energySamples++;

                // ── Rhythm / Step Detection ──────────────
                stepSignal       = 0;
                burstDetected    = false;
                bool newStepDetected = false;

                if (lCompositeSignal > COMPOSITE_THRESHOLD && (millis() - lastStepTime > STEP_MIN_DELAY_MS)) {
                    unsigned long currentTime = millis();
                    long currentInterval = currentTime - lastStepTime;
                    if (lastInterval > 0) {
                        irregularityIndex = abs(currentInterval - lastInterval);
                    }
                    lastInterval = currentInterval;
                    lastStepTime = currentTime;

                    // Burst detection
                    stepTimestamps[0] = stepTimestamps[1];
                    stepTimestamps[1] = stepTimestamps[2];
                    stepTimestamps[2] = currentTime;
                    if (stepTimestamps[0] != 0) {
                        unsigned long dur3 = stepTimestamps[2] - stepTimestamps[0];
                        if (dur3 < BURST_THRESHOLD_MS) burstDetected = true;
                    }

                    float stepEnergy = (energySamples > 0) ? sqrt(energySum / energySamples) : 0.0f;
                    if (prevStepEnergy > 0) {
                        energyIrregularity = fabs(stepEnergy - prevStepEnergy) / prevStepEnergy;
                    }
                    prevStepEnergy = stepEnergy;
                    energySum      = 0.0f;
                    energySamples  = 0;

                    stepCount++;
                    stepSignal       = COMPOSITE_THRESHOLD + 2000;
                    newStepDetected  = true;
                }

                // ── Sway Detection ───────────────────────
                rollDelta = fabs(lRoll - rollBaseline);
                gyroEnergy = (gyroEnergy * 0.8f) + (lGyroMag * 0.2f);

                rollBuffer[rollIndex] = rollDelta;
                rollIndex++;
                if (rollIndex >= IRREG_WINDOW) {
                    rollIndex       = 0;
                    rollBufferFilled = true;
                }
                if (rollBufferFilled) {
                    float mean = 0;
                    for (int i = 0; i < IRREG_WINDOW; i++) mean += rollBuffer[i];
                    mean /= IRREG_WINDOW;
                    float var = 0;
                    for (int i = 0; i < IRREG_WINDOW; i++) var += (rollBuffer[i] - mean) * (rollBuffer[i] - mean);
                    rollIrregularity = sqrt(var / IRREG_WINDOW);
                }

                if (calibrationDone && !swayDetected && gyroEnergy < 2.0f) {
                    rollBaseline = rollBaseline * 0.995f + lRoll * 0.005f;
                }

                bool normalSway  = calibrationDone && (rollDelta > MIN_TILT_TRIGGER) &&
                                   (gyroEnergy > MIN_GYRO_TRIGGER) && (rollIrregularity > IRREGULARITY_LIMIT);
                bool hardTrigger = calibrationDone && ((rollDelta > HARD_TILT_LIMIT) || (gyroEnergy > HARD_GYRO_LIMIT));

                if (normalSway || hardTrigger) {
                    if (swayStart == 0) swayStart = millis();
                    if (millis() - swayStart > CONFIRM_MS) swayDetected = true;
                } else {
                    swayStart    = 0;
                    swayDetected = false;
                }

                // ── Alert Trigger Logic (evaluated here for immediacy) ──
                bool triggerNow = false;
                if (!alertActive) {
                    if (swayDetected) {
                        triggerNow  = true;
                        alertReason = 1;
                    } else if (burstDetected) {
                        triggerNow  = true;
                        alertReason = 3;
                    } else if (newStepDetected) {
                        if (stepCount > MIN_STEPS_BEFORE_ALERT) {
                            if (energyIrregularity > RHYTHM_IRREG_LIMIT) {
                                irregularStepCounter++;
                            } else {
                                irregularStepCounter = 0;
                            }
                            if (irregularStepCounter >= IRREGULAR_STEPS_REQUIRED) {
                                triggerNow           = true;
                                alertReason          = 2;
                                irregularStepCounter = 0;
                            }
                        }
                    }
                }
                if (triggerNow) {
                    alertActive  = true;
                    alertEndTime = millis() + ALERT_DURATION_MS;
                }

                xSemaphoreGive(xSharedMutex);
            }
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ═════════════════════════════════════════════
//  TASK 3 – GESTURE SENSOR / COMMUNICATION  (Priority 2)
//  Checks for gesture dismiss; period 50 ms
// ═════════════════════════════════════════════
void TaskGestureComm(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(50);

    for (;;) {
        if (Lpg.ping()) {
            Lpg.getGesture(); // Clear gesture register

            if (xSemaphoreTake(xSharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (alertActive) {
                    alertActive  = false;
                    buzzerState  = false;
                    alertReason  = 0;
                    noTone(BUZZER_PIN);
                    digitalWrite(BUZZER_PIN, LOW);
                    Serial.println("Alarm Dismissed by Gesture");
                    display.clearDisplay();
                    display.display();
                }
                xSemaphoreGive(xSharedMutex);
            }
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ═════════════════════════════════════════════
//  TASK 4 – ALERT / BUZZER MANAGEMENT  (Priority 4)
//  Manages buzzer toggling at 150 ms; period 20 ms
// ═════════════════════════════════════════════
void TaskProcessAlert(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);

    for (;;) {
        if (xSemaphoreTake(xSharedMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
            if (alertActive) {
                if (millis() > alertEndTime && !swayDetected) {
                    alertActive  = false;
                    buzzerState  = false;
                    alertReason  = 0;
                    noTone(BUZZER_PIN);
                    digitalWrite(BUZZER_PIN, LOW);
                } else {
                    if (millis() - buzzerTimer > 150) {
                        buzzerTimer = millis();
                        buzzerState = !buzzerState;
                        digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
                    }
                }
            } else {
                digitalWrite(BUZZER_PIN, LOW);
            }
            xSemaphoreGive(xSharedMutex);
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ═════════════════════════════════════════════
//  TASK 5 – DISPLAY UPDATE  (Priority 1)
//  Updates at 100 ms; cycles screens every 4 s
// ═════════════════════════════════════════════
void TaskDisplay(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);

    for (;;) {
        if (xSemaphoreTake(xSharedMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (alertActive) {
                drawAlertScreen();
            } else {
                unsigned long cycle = millis() % 8000UL;
                if (cycle < 4000UL) {
                    updateCombinedDisplay();
                } else {
                    updateAltitudeDisplay();
                }
            }
            xSemaphoreGive(xSharedMutex);
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ═════════════════════════════════════════════
//  DISPLAY HELPERS
//  (Called with mutex already held by TaskDisplay)
// ═════════════════════════════════════════════

void centerText(String text, int y, int size) {
    int16_t  x1, y1;
    uint16_t w, h;
    display.setTextSize(size);
    display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, y);
    display.print(text);
}

// ── Gait Screen ──────────────────────────────
void updateCombinedDisplay() {
    display.clearDisplay();
    display.setTextColor(WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("Gait Analysis"));

    display.setCursor(0, 12);
    display.print("Roll:"); display.print((int)rollDelta);
    display.print(" Gyro:"); display.print((int)gyroEnergy);

    display.drawLine(0, 24, 128, 24, WHITE);

    display.setCursor(0, 30);
    display.print(F("Steps: "));
    display.setTextSize(2);
    display.print(stepCount);

    display.setTextSize(1);
    display.setCursor(0, 52);
    display.print(F("Irregularity: "));
    display.print(energyIrregularity, 2);

    display.display();
}

// ── Altitude Screen ───────────────────────────
void updateAltitudeDisplay() {
    display.clearDisplay();

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("ALTITUDE DASH");
    display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

    display.setCursor(0, 15);
    display.setTextSize(1);
    display.print("Height");

    display.setCursor(0, 25);
    display.setTextSize(2);
    display.print(relativeHeight, 1);
    display.setTextSize(1);
    display.print(" m");

    display.setCursor(0, 45);
    display.setTextSize(1);
    display.print("Rate");

    display.setCursor(40, 42);
    display.setTextSize(2);
    display.print(verticalSpeedFiltered, 2);
    display.setTextSize(1);
    display.print(" m/s");

    display.fillRect(0, 54, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    centerText(statusStr, 55, 1);
    display.setTextColor(SSD1306_WHITE);

    display.display();
}

// ── Alert Screen ─────────────────────────────
void drawAlertScreen() {
    display.clearDisplay();
    display.setTextColor(WHITE);

    display.drawRect(0, 0, 128, 64, WHITE);

    display.setTextSize(2);
    display.setCursor(20, 10);
    display.print(F("WARNING"));

    display.setTextSize(1);
    display.setCursor(15, 35);

    if (alertReason == 1 || swayDetected) {
        display.print(F("SWAY DETECTED!"));
    } else if (alertReason == 3) {
        display.print(F("RAPID STEPS!"));
    } else {
        display.print(F("HIGH IRREGULARITY"));
    }

    display.setCursor(40, 50);
    if (buzzerState) display.print(F("(BEEP)"));

    display.display();
}
