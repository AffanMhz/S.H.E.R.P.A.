/*
 * ******************************************************************************************
 * PROJECT S.H.E.R.P.A.
 * (Smart High-altitude Exertion & Real-time Pathological Analyzer)
 * ******************************************************************************************
 * * MODULE:        Robust Sway Detection & Ataxia Calibration Unit
 * FILE:          Sway_Fusion_Test.ino
 * HARDWARE:      MYOSA Mini Kit (Core: ESP32/AVR | Sensor: MPU6050 DMP @ 0x69)
 * * ------------------------------------------------------------------------------------------
 * PROJECT ABSTRACT
 * ------------------------------------------------------------------------------------------
 * S.H.E.R.P.A. is a lumbar-mounted wearable utilizing context-aware sensor fusion to 
 * detect early markers of Acute Mountain Sickness (AMS). The system distinguishes 
 * healthy physical exertion from pathological instability (Ataxia) in real-time, 
 * operating entirely on the edge without cloud reliance.
 * * ------------------------------------------------------------------------------------------
 * CODE FUNCTIONALITY
 * ------------------------------------------------------------------------------------------
 * This specific module implements the "Hybrid Trigger" logic to isolate pathological sway.
 * It utilizes a running average (EMA) to smooth gyro energy and auto-calibrates the
 * lumbar tilt baseline. It detects:
 * 1. Standard Sway (Correlated Tilt + Gyro Energy)
 * 2. Violent Instability (High-G Movement)
 * 3. Extreme Lean (Pathological Tilt Deviation)
 * * ------------------------------------------------------------------------------------------
 * TEAM 5114 - "THE PEAK PERFORMERS"
 * ------------------------------------------------------------------------------------------
 * 1. Affan Danish ........... Lead Firmware Architect & Sensor Fusion Specialist
 * 2. Zahaib ................. Hardware Integration Lead & PCB Strategist
 * 3. Ammar .................. Algorithmic Optimization Engineer & Data Analyst
 * 4. Maaz ................... Embedded Systems Commander & Prototype Lead
 * * ------------------------------------------------------------------------------------------
 * COPYRIGHT (C) 2025 TEAM S.H.E.R.P.A. - ALL RIGHTS RESERVED
 * ******************************************************************************************
 */
 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MPU6050 mpu(0x69);

// --- MPU Variables ---
bool dmpReady = false;  
uint16_t packetSize;    
uint8_t FIFOBuffer[64]; 
Quaternion q;           
VectorInt16 gy;         
VectorFloat gravity;    
float ypr[3];           

// --- Interrupt ---
volatile bool MPUInterrupt = false;
void dmpDataReady() { MPUInterrupt = true; }

// --- TUNING (UNCHANGED — YOUR TESTED VALUES) ---
#define MIN_TILT_TRIGGER      9.0f
#define MIN_GYRO_TRIGGER      8.0f

#define HARD_TILT_LIMIT       35.0f
#define HARD_GYRO_LIMIT       40.0f

#define CONFIRM_MS            80

// --- STATE ---
float gyroEnergy = 0;
float rollBaseline = 0;
unsigned long swayStart = 0;
bool swayDetected = false;

// ===================== IRREGULARITY VARIABLES =====================
#define IRREG_WINDOW 20
float rollBuffer[IRREG_WINDOW];
int rollIndex = 0;
bool rollBufferFilled = false;

float rollIrregularity = 0;
#define IRREGULARITY_LIMIT 3.0f
// ================================================================

// ===================== SAFETY =====================
bool calibrationDone = false;
// ==================================================

void setup() {
  Wire.begin();
  Wire.setClock(400000); 
  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) for(;;);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  mpu.initialize();
  pinMode(2, INPUT); 
  uint8_t devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    while(1);
  }

  // -------- BASELINE CALIBRATION --------
  Serial.println("Calibrating Baseline (Keep Still)...");
  display.setCursor(0,0);
  display.println("Calibrating...");
  display.display();
  
  long rSum = 0;
  int validSamples = 0;
  
  mpu.resetFIFO();
  delay(1000);

  while(validSamples < 100) {
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      rSum += (ypr[2] * 180/M_PI);   // ROLL ONLY
      validSamples++;
      delay(5);
    }
  }

  rollBaseline = rSum / 100.0;
  calibrationDone = true;
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { 
    
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, FIFOBuffer);

    float roll = ypr[2] * 180/M_PI;

    // -------- ROLL-ONLY DEVIATION --------
    float rollDelta = fabs(roll - rollBaseline);

    // -------- GYRO ENERGY --------
    float gx = gy.x / 16.4; 
    float gy_val = gy.y / 16.4;
    float currentGyroMag = sqrt(gx*gx + gy_val*gy_val);
    gyroEnergy = (gyroEnergy * 0.8) + (currentGyroMag * 0.2);

    // -------- ROLL IRREGULARITY --------
    rollBuffer[rollIndex] = rollDelta;
    rollIndex++;

    if (rollIndex >= IRREG_WINDOW) {
      rollIndex = 0;
      rollBufferFilled = true;
    }

    if (rollBufferFilled) {
      float mean = 0;
      for(int i = 0; i < IRREG_WINDOW; i++)
        mean += rollBuffer[i];
      mean /= IRREG_WINDOW;

      float var = 0;
      for(int i = 0; i < IRREG_WINDOW; i++)
        var += (rollBuffer[i] - mean) * (rollBuffer[i] - mean);
      var /= IRREG_WINDOW;

      rollIrregularity = sqrt(var);
    }

    // -------- BASELINE DRIFT (ONLY WHEN STABLE) --------
    if (calibrationDone &&
        !swayDetected &&
        gyroEnergy < 2.0f) {

      rollBaseline = rollBaseline * 0.995f + roll * 0.005f;
    }

    // -------- SWAY LOGIC (ROLL ONLY) --------
    bool normalSway =
      calibrationDone &&
      (rollDelta > MIN_TILT_TRIGGER) &&
      (gyroEnergy > MIN_GYRO_TRIGGER) &&
      (rollIrregularity > IRREGULARITY_LIMIT);

    bool hardTrigger =
      calibrationDone &&
      ((rollDelta > HARD_TILT_LIMIT) ||
       (gyroEnergy > HARD_GYRO_LIMIT));

    if (normalSway || hardTrigger) {
      if (swayStart == 0) swayStart = millis();
      if (millis() - swayStart > CONFIRM_MS) swayDetected = true;
    } else {
      swayStart = 0;
      swayDetected = false;
    }

    // -------- SERIAL --------
    Serial.print("Gyro:"); Serial.print(gyroEnergy);
    Serial.print(" RollΔ:"); Serial.print(rollDelta);
    Serial.print(" Irreg:"); Serial.print(rollIrregularity);
    Serial.print(" STATE:"); Serial.println(swayDetected ? 40 : 0);

    // -------- OLED --------
    static unsigned long lastDraw = 0;
    if (millis() - lastDraw > 100) {
      lastDraw = millis();
      display.clearDisplay();
      
      display.setCursor(0, 0);
      display.print("G:"); display.print((int)gyroEnergy);
      display.setCursor(64, 0);
      display.print("R:"); display.print((int)rollDelta);

      display.setCursor(0, 12);
      display.print("I:"); display.print(rollIrregularity, 1);
      
      display.setCursor(0, 30);
      display.setTextSize(2);
      if(swayDetected) display.print("SWAY!");
      else display.print("STABLE");
      
      display.display();
    }
  }
}
