/*
  Project:        S.H.E.R.P.A. (Smart High-altitude Early Risk Prediction Assistant)
  Team:           Hardcoders
  Author:         Affan Danish
  Platform:       MYOSA (MakeSense EduTech / Pegasus Automation)

  Component:      Multisensor Monitoring & Early-Risk Alert System
  Focus:          Gait irregularity, sway detection, ascent profiling,
                  and contextual alerts related to Acute Mountain Sickness (AMS)

  
  HARDWARE & INTERFACES (MYOSA PLATFORM)
  
  IMU Sensor:     MPU6050 (6-DOF Accelerometer + Gyroscope)
                  I2C Address: 0x69

  Barometric:     BMP180 (Bosch Barometric Pressure Sensor)
                  I2C Address: 0x77
                  Pressure Range: 300–1100 hPa
                  Resolution: ~0.25 m altitude

  Display:        SSD1306 OLED Display (128×64)
                  I2C Address: 0x3C

  Gesture Sensor: APDS9960 (Gesture / Proximity / Light)
                  I2C Address: 0x39

  Alert Output:   Active Buzzer (GPIO-controlled)

  
  SYSTEM PURPOSE
  
  S.H.E.R.P.A. is a wearable, real-time multisensor system designed
  to provide early, objective indicators associated with altitude
  stress and movement instability during high-altitude ascent.

  The system combines:
  - Barometric ascent rate & relative altitude tracking
  - Step rhythm and energy irregularity analysis
  - Roll-based sway detection with gyro confirmation
  - Context-aware alerting with visual and auditory feedback

  The goal is not medical diagnosis, but early risk awareness
  to support timely decisions such as rest, pace adjustment,
  or descent.

  
  SCIENTIFIC CONTEXT
  
  Acute Mountain Sickness (AMS) affects approximately 25–50%
  of individuals ascending above 2,500 meters, with 1–2%
  progressing to severe conditions such as High-Altitude
  Cerebral Edema (HACE).

  Clinical research indicates that objective physiological
  and behavioral markers often precede subjective symptoms
  by 30–60 minutes. This window is critical for preventive
  intervention.

  Unlike conventional altitude trackers that passively
  display elevation, S.H.E.R.P.A. integrates environmental
  and motion-derived indicators to surface early warning
  patterns in real time.

  
  DEVELOPMENT NOTES
  
  - All signal processing is performed onboard
  - Low-pass filtering applied to altitude and ascent rate
  - Event-based alerts prioritized over raw data display
  - Designed for low cognitive load in hypoxic environments

  Date:           December 2025
*/


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include <LightProximityAndGesture.h> // Added Gesture Library

//  PIN DEFINITIONS 
#define INTERRUPT_PIN 2
#define BUZZER_PIN    12  // Connect (+) of buzzer here, (-) to GND

//  DISPLAY CONFIG 
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//  MPU CONFIG 
MPU6050 mpu(0x69);

//  BMP (ALTITUDE) CONFIG 
Adafruit_BMP085 bmp;
// Sensor Variables
float baselineAltitude = 0.0;
float smoothedAltitude = 0.0;
const float alpha = 0.1;          // Altitude LPF
float prevAltitude = 0.0;
unsigned long bmpPrevTime = 0;    
float verticalSpeedRaw = 0.0;
float verticalSpeedFiltered = 0.0;
const float beta = 0.25;          // Speed Filter
float relativeHeight = 0.0;
String statusStr = "STABLE";
const float MOVEMENT_THRESHOLD = 0.3;  // m/s

//  GESTURE SENSOR CONFIG 
LightProximityAndGesture Lpg;

// Shared MPU Variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
volatile bool mpuInterrupt = false;

// Shared Motion Containers
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 gy;
VectorFloat gravity;
float ypr[3];

//  ALERT SYSTEM VARIABLES 
bool alertActive = false;
unsigned long alertEndTime = 0;
unsigned long buzzerTimer = 0;
bool buzzerState = false;
int alertReason = 0; // 0: None, 1: Sway, 2: Irregularity, 3: Rapid Steps (Burst)
#define ALERT_DURATION_MS 3000  // How long the alert lasts after trigger
#define RHYTHM_IRREG_LIMIT 0.30 // 30% irregularity triggers alert

//  CODE 1: RHYTHM VARIABLES 
#define COMPOSITE_THRESHOLD 12000
#define STEP_MIN_DELAY_MS 300

unsigned long lastStepTime = 0;
int stepCount = 0;
int stepSignal = 0;
long lastInterval = 0;
long irregularityIndex = 0;

float energySum = 0.0;
int energySamples = 0;
float prevStepEnergy = 0.0;
float energyIrregularity = 0.0;
float compositeSignal = 0.0; 

// --- NEW: BURST / MULTIPLE STEP DETECTION ---
#define BURST_THRESHOLD_MS 1300 // Max time for 3 steps to be considered a "burst"
unsigned long stepTimestamps[3] = {0, 0, 0};
bool burstDetected = false;

//  CODE 2: SWAY VARIABLES 
#define MIN_TILT_TRIGGER      9.0f
#define MIN_GYRO_TRIGGER      8.0f
#define HARD_TILT_LIMIT       35.0f
#define HARD_GYRO_LIMIT       40.0f
#define CONFIRM_MS            80

float gyroEnergy = 0;
float rollBaseline = 0;
unsigned long swayStart = 0;
bool swayDetected = false;
float rollDelta = 0; 

#define IRREG_WINDOW 20
float rollBuffer[IRREG_WINDOW];
int rollIndex = 0;
bool rollBufferFilled = false;
float rollIrregularity = 0;
#define IRREGULARITY_LIMIT 3.0f

bool calibrationDone = false;

//  TIMERS 
unsigned long previousDisplayMillis = 0;
const long displayInterval = 100; 

//  INTERRUPT 
void dmpDataReady() {
    mpuInterrupt = true;
}

//  SETUP 
void setup() {
    Wire.begin(); // Standard Wire init
    Wire.setClock(400000); // Keep high speed for MPU
    Serial.begin(115200);
    
    // Config Pins
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); 

    delay(100);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;);
    }

    // --- INIT SENSORS SCREEN ---
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println(F("Init Sensors..."));
    display.display();

    // 1. Init BMP
    if (!bmp.begin()) {
        display.println(F("BMP ERROR! Check wiring."));
    } else {
        display.println(F("BMP OK"));
    }
    display.display();
    
    // 2. Init Gesture (With Timeout to prevent hanging)
    display.println(F("Init Gesture..."));
    display.display();
    
    unsigned long gestureStart = millis();
    bool gestureFound = false;
    // Try to connect for up to 2 seconds
    while (millis() - gestureStart < 2000) {
        if(Lpg.begin()) {
            gestureFound = true;
            Serial.println("Gesture Sensor Connected");
            break;
        }
        delay(100);
    }
    
    if (gestureFound) {
        Serial.println("APDS9960 init completed");
        // Enable Gesture (Passed true instead of DISABLE to actually run it)
        if(Lpg.enableGestureSensor(ENABLE)) {
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

    // 3. Init MPU
    mpu.initialize();

    if(mpu.testConnection() == false){
        display.println(F("MPU Failed!"));
        display.display();
        while(true);
    }

    devStatus = mpu.dmpInitialize();

    // Tuned Offsets
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
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Init failed"));
        while(1);
    }

    // --- CALIBRATION SCREEN ---
    display.clearDisplay();
    centerText("CALIBRATING", 20, 1);
    centerText("Keep Still", 40, 1);
    display.display();

    // 4. Calibrate BMP
    float sum = 0;
    for(int i = 0; i < 50; i++) {
        sum += bmp.readAltitude();
        delay(20);
    }
    baselineAltitude = sum / 50.0;
    smoothedAltitude = baselineAltitude;
    prevAltitude = smoothedAltitude;
    bmpPrevTime = millis();

    // 5. Calibrate MPU (Roll Baseline)
    long rSum = 0;
    int validSamples = 0;
    mpu.resetFIFO();
    
    while(validSamples < 100) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            rSum += (ypr[2] * 180/M_PI);   
            validSamples++;
            delay(5);
        }
    }

    rollBaseline = rSum / 100.0;
    calibrationDone = true;
    display.clearDisplay();
}

//  LOOP 
void loop() {
    // --- GESTURE ALARM STOP LOGIC ---
    // Check if gesture is detected
    if(Lpg.ping()) {
        Lpg.getGesture(); // Read gesture to clear it
        
        // If alarm is active, STOP IT immediately
        if (alertActive) {
            alertActive = false;
            buzzerState = false;
            alertReason = 0;
            noTone(BUZZER_PIN);
            digitalWrite(BUZZER_PIN, LOW);
            Serial.println("Alarm Dismissed by Gesture");
            
            // Force display update to clear warning screen
            display.clearDisplay();
        }
    }

    // --- BMP ALTITUDE PROCESSING ---
    unsigned long now = millis();
    float rawAlt = bmp.readAltitude();
    smoothedAltitude = alpha * rawAlt + (1 - alpha) * smoothedAltitude;
    relativeHeight = smoothedAltitude - baselineAltitude;

    float dt = (now - bmpPrevTime) / 1000.0;
    if (dt > 0.25) {
        verticalSpeedRaw = (smoothedAltitude - prevAltitude) / dt;
        verticalSpeedFiltered = beta * verticalSpeedRaw + (1 - beta) * verticalSpeedFiltered;

        if (verticalSpeedFiltered > MOVEMENT_THRESHOLD)
             statusStr = "ASCENDING";
        else if (verticalSpeedFiltered < -MOVEMENT_THRESHOLD)
             statusStr = "DESCENDING";
        else
             statusStr = "STABLE";

        prevAltitude = smoothedAltitude;
        bmpPrevTime = now;
    }

    // --- MPU PROCESSING ---
    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

        // 1. EXTRACT DATA
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
        mpu.dmpGetAccel(&aa, fifoBuffer);          
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); 
        mpu.dmpGetGyro(&gy, fifoBuffer);           

        // 2. LOGIC: RHYTHM ANALYZER
        float verticalAcc = (aaReal.x * gravity.x) + (aaReal.y * gravity.y) + (aaReal.z * gravity.z);
        float linearMag = sqrt(pow(aaReal.x, 2) + pow(aaReal.y, 2) + pow(aaReal.z, 2));
        float linearVertical = abs(verticalAcc);
        float rhythmGyroPower = sqrt(pow(gy.x, 2) + pow(gy.y, 2) + pow(gy.z, 2)); 

        compositeSignal = (linearMag * 1.0) + (linearVertical * 1.5) + ((rhythmGyroPower / 20.0) * 0.5);

        energySum += compositeSignal * compositeSignal;
        energySamples++;
        stepSignal = 0;
        bool newStepDetected = false;
        burstDetected = false;

        if (compositeSignal > COMPOSITE_THRESHOLD && (millis() - lastStepTime > STEP_MIN_DELAY_MS)) {
            unsigned long currentTime = millis();
            long currentInterval = currentTime - lastStepTime;
            if (lastInterval > 0) {
                irregularityIndex = abs(currentInterval - lastInterval);
            }
            lastInterval = currentInterval;
            lastStepTime = currentTime;

            // --- BURST DETECTION LOGIC ---
            stepTimestamps[0] = stepTimestamps[1];
            stepTimestamps[1] = stepTimestamps[2];
            stepTimestamps[2] = currentTime;

            if (stepTimestamps[0] != 0) {
                unsigned long durationFor3Steps = stepTimestamps[2] - stepTimestamps[0];
                if (durationFor3Steps < BURST_THRESHOLD_MS) {
                    burstDetected = true;
                }
            }

            float stepEnergy = sqrt(energySum / energySamples);
            if (prevStepEnergy > 0) {
                energyIrregularity = abs(stepEnergy - prevStepEnergy) / prevStepEnergy;
            }
            prevStepEnergy = stepEnergy;
            energySum = 0;
            energySamples = 0;

            stepCount++;
            stepSignal = COMPOSITE_THRESHOLD + 2000;
            newStepDetected = true;
        }

        // 3. LOGIC: SWAY DETECTOR
        float roll = ypr[2] * 180/M_PI;
        rollDelta = fabs(roll - rollBaseline);

        float gx = gy.x / 16.4;
        float gy_val = gy.y / 16.4;
        float currentGyroMag = sqrt(gx*gx + gy_val*gy_val);
        gyroEnergy = (gyroEnergy * 0.8) + (currentGyroMag * 0.2);

        rollBuffer[rollIndex] = rollDelta;
        rollIndex++;
        if (rollIndex >= IRREG_WINDOW) {
            rollIndex = 0;
            rollBufferFilled = true;
        }

        if (rollBufferFilled) {
            float mean = 0;
            for(int i = 0; i < IRREG_WINDOW; i++) mean += rollBuffer[i];
            mean /= IRREG_WINDOW;
            float var = 0;
            for(int i = 0; i < IRREG_WINDOW; i++) var += (rollBuffer[i] - mean) * (rollBuffer[i] - mean);
            var /= IRREG_WINDOW;
            rollIrregularity = sqrt(var);
        }

        if (calibrationDone && !swayDetected && gyroEnergy < 2.0f) {
            rollBaseline = rollBaseline * 0.995f + roll * 0.005f;
        }

        bool normalSway = calibrationDone && (rollDelta > MIN_TILT_TRIGGER) &&
                          (gyroEnergy > MIN_GYRO_TRIGGER) && (rollIrregularity > IRREGULARITY_LIMIT);
        bool hardTrigger = calibrationDone && ((rollDelta > HARD_TILT_LIMIT) || (gyroEnergy > HARD_GYRO_LIMIT));

        if (normalSway || hardTrigger) {
            if (swayStart == 0) swayStart = millis();
            if (millis() - swayStart > CONFIRM_MS) swayDetected = true;
        } else {
            swayStart = 0;
            swayDetected = false;
        }

        //  4. ALERT TRIGGER SYSTEM 
        bool triggerNow = false;

        // If alert is NOT active, we check triggers
        // If alert IS active, we don't re-trigger, we just wait for it to end or be dismissed (by gesture)
        if (!alertActive) {
            if (swayDetected) {
                triggerNow = true;
                alertReason = 1;
            }
            else if (burstDetected) {
                triggerNow = true;
                alertReason = 3;
            }
            else if (newStepDetected && energyIrregularity > RHYTHM_IRREG_LIMIT) {
                triggerNow = true;
                alertReason = 2;
            }
        }

        if (triggerNow) {
            alertActive = true;
            alertEndTime = millis() + ALERT_DURATION_MS;
        }

        if (alertActive) {
            if (millis() > alertEndTime && !swayDetected) {
                alertActive = false;
                buzzerState = false;
                alertReason = 0;
                noTone(BUZZER_PIN);
                digitalWrite(BUZZER_PIN, LOW);
            } 
            else {
                if (millis() - buzzerTimer > 150) {
                    buzzerTimer = millis();
                    buzzerState = !buzzerState;
                    if (buzzerState) {
                         digitalWrite(BUZZER_PIN, HIGH);
                    } else {
                         digitalWrite(BUZZER_PIN, LOW);
                    }
                }
            }
        } else {
             digitalWrite(BUZZER_PIN, LOW);
        }

        //  5. DISPLAY MANAGER 
        if (millis() - previousDisplayMillis >= displayInterval) {
            previousDisplayMillis = millis();
            
            if (alertActive) {
                drawAlertScreen();
            } else {
                // Cycle screens every 4000ms (4 seconds)
                // 0-4000: Gait Screen
                // 4000-8000: Altitude Screen
                unsigned long cycle = millis() % 8000;
                if (cycle < 4000) {
                    updateCombinedDisplay();
                } else {
                    updateAltitudeDisplay();
                }
            }
        }
    }
}

//  HELPER FUNCTIONS 

void centerText(String text, int y, int size) {
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(size);
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, y);
  display.print(text);
}

// --- GAIT SCREEN ---
void updateCombinedDisplay() {
    display.clearDisplay();
    display.setTextColor(WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("Gait Analysis"));
    
    // Sway Data
    display.setCursor(0, 12);
    display.print("Roll:"); display.print((int)rollDelta);
    display.print(" Gyro:"); display.print((int)gyroEnergy);

    display.drawLine(0, 24, 128, 24, WHITE);

    // Rhythm Data
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

// --- ALTITUDE SCREEN ---
void updateAltitudeDisplay() {
    display.clearDisplay();
    
    // Header
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("ALTITUDE DASH");
    display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

    // Height
    display.setCursor(0, 15);
    display.setTextSize(1);
    display.print("Height");

    display.setCursor(0, 25);
    display.setTextSize(2);
    display.print(relativeHeight, 1);
    display.setTextSize(1);
    display.print(" m");

    // Speed
    display.setCursor(0, 45);
    display.setTextSize(1);
    display.print("Rate");

    display.setCursor(40, 42);
    display.setTextSize(2);
    display.print(verticalSpeedFiltered, 2);
    display.setTextSize(1);
    display.print(" m/s");

    // Status Bar
    display.fillRect(0, 54, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    centerText(statusStr, 55, 1);
    display.setTextColor(SSD1306_WHITE); // Reset color

    display.display();
}

// --- ALERT SCREEN ---
void drawAlertScreen() {
    display.clearDisplay();
    display.setTextColor(WHITE); 
    
    // Draw Border
    display.drawRect(0, 0, 128, 64, WHITE);
    
    display.setTextSize(2);
    display.setCursor(20, 10);
    display.print(F("WARNING"));

    display.setTextSize(1);
    display.setCursor(15, 35);
    
    // Dynamic Alert Message
    if (alertReason == 1 || swayDetected) {
        display.print(F("SWAY DETECTED!"));
    } 
    else if (alertReason == 3) {
        display.print(F("RAPID STEPS!"));
    }
    else {
        display.print(F("HIGH IRREGULARITY"));
    }

    display.setCursor(40, 50);
    // Visual "Beep" indicator
    if(buzzerState) display.print(F("(BEEP)"));

    display.display();
}
