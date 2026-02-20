/*
 * ---------------------------------------------------------------------------------------
 * PROJECT S.H.E.R.P.A. 
 * (Smart High-Altitude Emergency Response & Positioning Assistant)
 * 
 * 
 * * SYSTEM DESCRIPTION:
 * S.H.E.R.P.A. is a lumbar-mounted wearable utilizing context-aware sensor fusion to 
 * detect early markers of Acute Mountain Sickness (AMS). The system distinguishes 
 * healthy physical exertion from pathological instability (Ataxia) in real-time
 * completely at the edge, without cloud reliance.

 *
 * MODULE PURPOSE:
 * [UNIT TEST] Arrhythmic Walk & Energy Irregularity Analyzer
 * This module tests the core step-detection engine using DMP Sensor Fusion. It 
 * calculates both "Timing Irregularity" (Step Jitter) and "Energy Irregularity" 
 * to quantify walking rhythm and step-force stability. It also includes Rhythm 
 * Burst Detection, triggering an active buzzer if steps occur too rapidly, 
 * serving as a key metric for detecting the onset of AMS-induced ataxia.
 *
 * HARDWARE CONFIGURATION:
 * - Platform: MyoSa Mini Kit
 * - MCU: ESPRESSIF ESP32-WROOM-32E
 * - IMU: MPU6050 (I2C Address 0x69)
 * - Display: SSD1306 OLED (I2C Address 0x3C)
 * - Alert: Active Buzzer (GPIO 12)

 * ---------------------------------------------------------------------------------------
 * TEAM MEMBERS & ROLES:
 * 1. Affan Danish    | Lead Firmware Architect
 * 2. Zahaib          | Project Specialist
 * 3. Ammar           | Systems Engineer
 * 4. Maaz            | project design
 * ---------------------------------------------------------------------------------------
 *
 * RTOS PORT - VERSION 2.4 (DMP Fusion + Rhythm Analysis + FreeRTOS)
 */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// --- FREE RTOS RESOURCES ---
SemaphoreHandle_t i2cMutex;
TaskHandle_t SensorTaskHandle;
TaskHandle_t DisplayTaskHandle;

// --- OLED CONFIGURATION ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- MPU6050 CONFIGURATION (ADDRESS 0x69) ---
MPU6050 mpu(0x69); 
#define INTERRUPT_PIN 2  

// --- PEDOMETER SETTINGS ---
#define COMPOSITE_THRESHOLD 12000   
#define STEP_MIN_DELAY_MS 300       

// --- BUZZER CONFIGURATION ---
#define BUZZER_PIN 12
#define BURST_THRESHOLD_MS 1300 
#define BUZZER_DURATION 2000    

// MPU Control/Status
bool dmpReady = false;
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint8_t fifoBuffer[64]; 

// Orientation/Motion Variables
Quaternion q;           
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 gy;         
VectorFloat gravity;

// --- STEP & TIMING VARIABLES ---
volatile unsigned long lastStepTime = 0;
volatile int stepCount = 0;
volatile int stepSignal = 0;

// --- TIMING IRREGULARITY ---
volatile long lastInterval = 0;
volatile long irregularityIndex = 0;

// --- ENERGY IRREGULARITY VARIABLES ---
float energySum = 0.0;
int energySamples = 0;
float prevStepEnergy = 0.0;
volatile float energyIrregularity = 0.0;
volatile float currentCompositeSignal = 0.0; // Added for thread-safe display sharing

// --- RHYTHM/BUZZER VARIABLES ---
unsigned long stepTimestamps[3] = {0, 0, 0};
unsigned long buzzerStartTime = 0;
volatile bool buzzerActive = false;

// Interrupt Routine
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

// --- TASK DECLARATIONS ---
void SensorTask(void *pvParameters);
void DisplayTask(void *pvParameters);

void setup() {
    Wire.begin();
    Wire.setClock(400000); 
    Serial.begin(115200);
    
    // Create Mutex for I2C Bus protection
    i2cMutex = xSemaphoreCreateMutex();

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println(F("Init MPU @ 0x69..."));
    display.display();

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if(mpu.testConnection() == false){
        display.println(F("MPU Failed!"));
        display.display();
        while(true);
    }

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        display.clearDisplay();
    }

    // --- CREATE RTOS TASKS ---
    // Sensor Task: High Priority (3), pinned to Core 1 (Application Core)
    xTaskCreatePinnedToCore(
        SensorTask, "SensorTask", 4096, NULL, 3, &SensorTaskHandle, 1);

    // Display Task: Low Priority (1), pinned to Core 0 (Protocol Core)
    xTaskCreatePinnedToCore(
        DisplayTask, "DisplayTask", 4096, NULL, 1, &DisplayTaskHandle, 0);
}

void loop() {
    // Empty. RTOS scheduler takes over automatically in ESP32.
    vTaskDelete(NULL); 
}

// ==========================================
// CORE 1: SENSOR & MATH TASK (HIGH PRIORITY)
// ==========================================
void SensorTask(void *pvParameters) {
    for (;;) {
        if (!dmpReady) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        bool packetAvailable = false;

        // Lock I2C to check MPU6050 FIFO
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            packetAvailable = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
            xSemaphoreGive(i2cMutex);
        }

        if (packetAvailable) { 
            // Lock I2C to read quaternion/accel/gyro data
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                mpu.dmpGetGyro(&gy, fifoBuffer);
                xSemaphoreGive(i2cMutex);
            }

            float verticalAcc = (aaReal.x * gravity.x) + (aaReal.y * gravity.y) + (aaReal.z * gravity.z);
            float linearMag = sqrt(pow(aaReal.x, 2) + pow(aaReal.y, 2) + pow(aaReal.z, 2));
            float linearVertical = abs(verticalAcc);
            float gyroPower = sqrt(pow(gy.x, 2) + pow(gy.y, 2) + pow(gy.z, 2));
            
            float compositeSignal = (linearMag * 1.0) + (linearVertical * 1.5) + ((gyroPower / 20.0) * 0.5);
            
            // Update global for display task
            currentCompositeSignal = compositeSignal;

            // --- ENERGY ACCUMULATION ---
            energySum += compositeSignal * compositeSignal;
            energySamples++;
            stepSignal = 0; 
            
            if (compositeSignal > COMPOSITE_THRESHOLD && (millis() - lastStepTime > STEP_MIN_DELAY_MS)) {

                unsigned long currentTime = millis();

                // --- TIMING IRREGULARITY ---
                long currentInterval = currentTime - lastStepTime;
                if (lastInterval > 0) {
                    irregularityIndex = abs(currentInterval - lastInterval);
                }
                lastInterval = currentInterval;
                lastStepTime = currentTime;

                // --- RHYTHM BURST DETECTION ---
                stepTimestamps[0] = stepTimestamps[1];
                stepTimestamps[1] = stepTimestamps[2];
                stepTimestamps[2] = currentTime;

                if (stepTimestamps[0] != 0) {
                    unsigned long durationFor3Steps = stepTimestamps[2] - stepTimestamps[0];
                    if (durationFor3Steps < BURST_THRESHOLD_MS) {
                        digitalWrite(BUZZER_PIN, HIGH);
                        buzzerStartTime = millis();
                        buzzerActive = true;
                    }
                }

                // --- ENERGY IRREGULARITY COMPUTATION ---
                float stepEnergy = sqrt(energySum / energySamples);
                if (prevStepEnergy > 0) {
                    energyIrregularity = abs(stepEnergy - prevStepEnergy) / prevStepEnergy;
                }

                prevStepEnergy = stepEnergy;
                energySum = 0;
                energySamples = 0;

                stepCount++;
                stepSignal = COMPOSITE_THRESHOLD + 2000;
            }

            // --- HANDLE BUZZER TIMEOUT ---
            if (buzzerActive && (millis() - buzzerStartTime > BUZZER_DURATION)) {
                digitalWrite(BUZZER_PIN, LOW);
                buzzerActive = false;
            }

            // Serial output remains inline with step processing for accurate plotter rendering
            Serial.print("Signal:"); Serial.print(compositeSignal);
            Serial.print(",Thresh:"); Serial.print(COMPOSITE_THRESHOLD);
            Serial.print(",Step:"); Serial.print(stepSignal);
            Serial.print(",TimeIrreg:"); Serial.print(irregularityIndex);
            Serial.print(",EnergyIrreg:"); Serial.println(energyIrregularity, 4);
        }
        
        // Yield 1 tick to prevent Watchdog Timeout on Core 1
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// ==========================================
// CORE 0: DISPLAY TASK (LOW PRIORITY)
// ==========================================
void DisplayTask(void *pvParameters) {
    const TickType_t xDisplayFrequency = pdMS_TO_TICKS(150); // 150ms interval from original code
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Grab local copies of volatile globals to ensure data stability during drawing
        int localStepCount = stepCount;
        float localIrreg = energyIrregularity;
        bool localBuzzer = buzzerActive;
        float localMag = currentCompositeSignal;

        // Lock I2C Bus to update Display
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            display.clearDisplay();
            display.setTextColor(WHITE);

            display.setTextSize(1);
            display.setCursor(0, 0);
            display.print(F("RHYTHM ANALYZER"));

            display.setCursor(0, 16);
            display.print(F("Steps: "));
            display.setTextSize(2); 
            display.print(localStepCount);

            display.setTextSize(1);
            display.setCursor(0, 40);
            display.print(F("Irreg(E): "));
            display.print(localIrreg, 2);

            if(localIrreg > 0.20) {
                display.setCursor(90, 40);
                display.print(F("!!"));
            }

            if(localBuzzer) {
                display.fillCircle(120, 5, 4, WHITE);
            }

            display.setCursor(0, 54);
            display.print(F("Sig: "));
            display.print((int)localMag);

            display.display();
            
            // Unlock I2C Bus
            xSemaphoreGive(i2cMutex);
        }

        // Sleep task precisely until the next 150ms interval
        vTaskDelayUntil(&xLastWakeTime, xDisplayFrequency);
    }
}