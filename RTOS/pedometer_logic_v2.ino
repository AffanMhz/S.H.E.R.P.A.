/*
 * =======================================================================================
 * PROJECT S.H.E.R.P.A. 
 * (Smart High-Altitude Emergency Response & Positioning Assistant)
 * =======================================================================================
 * * SYSTEM DESCRIPTION:
 * S.H.E.R.P.A. is a lumbar-mounted wearable utilizing context-aware sensor fusion to 
 * detect early markers of Acute Mountain Sickness (AMS). 
 *
 * RTOS VERSION: 2.2 (DMP Fusion + Rhythm Analysis + FreeRTOS Multitasking)
 * =======================================================================================
 */
 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

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

// --- ALGORITHM VARIABLES (Used only in Sensor Task) ---
unsigned long lastStepTime = 0;
int stepCount = 0;
int stepSignal = 0;
long lastInterval = 0;
long irregularityIndex = 0;
float energySum = 0.0;
int energySamples = 0;
float prevStepEnergy = 0.0;
float energyIrregularity = 0.0;

// --- RTOS HANDLES ---
SemaphoreHandle_t dmpSemaphore; // Triggers processing when MPU data is ready
SemaphoreHandle_t i2cMutex;     // Protects the I2C bus from collisions
SemaphoreHandle_t dataMutex;    // Protects shared variables between tasks

// --- SHARED DATA (For Display Task) ---
float shared_compositeSignal = 0.0;
int shared_stepCount = 0;
float shared_energyIrregularity = 0.0;

// --- INTERRUPT SERVICE ROUTINE (ISR) ---
void IRAM_ATTR dmpDataReady() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Give the semaphore to unblock the sensor task immediately
    xSemaphoreGiveFromISR(dmpSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// --- FUNCTION DECLARATIONS ---
void sensorTask(void *pvParameters);
void displayTask(void *pvParameters);
void updateDisplay(float mag, int steps, float irreg);

void setup() {
    Wire.begin();
    Wire.setClock(400000); 
    Serial.begin(115200);
    delay(100); 

    // Initialize RTOS primitives
    dmpSemaphore = xSemaphoreCreateBinary();
    i2cMutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
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
    // Core 1 is generally preferred for Arduino loops on ESP32
    xTaskCreatePinnedToCore(
        sensorTask,       // Task function
        "SensorTask",     // Name
        8192,             // Stack size (Words)
        NULL,             // Parameters
        2,                // Priority (Higher = more urgent)
        NULL,             // Task handle
        1                 // Core ID
    );

    xTaskCreatePinnedToCore(
        displayTask,      
        "DisplayTask",    
        4096,             
        NULL,             
        1,                // Priority (Lower than sensor)
        NULL,             
        1                 
    );
}

void loop() {
    // In FreeRTOS, the loop() is just another task. We delete it so it doesn't waste CPU.
    vTaskDelete(NULL);
}

// ==============================================================================
// TASK 1: IMU ACQUISITION & RHYTHM ALGORITHM (High Priority)
// ==============================================================================
void sensorTask(void *pvParameters) {
    for (;;) {
        // Wait for interrupt from MPU6050
        if (xSemaphoreTake(dmpSemaphore, portMAX_DELAY) == pdTRUE) {
            if (!dmpReady) continue;

            bool packetAvailable = false;
            
            // Safely use I2C to read MPU FIFO
            if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
                packetAvailable = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
                xSemaphoreGive(i2cMutex);
            }

            if (packetAvailable) { 
                // --- KINEMATICS MATH (Unchanged) ---
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                mpu.dmpGetGyro(&gy, fifoBuffer);

                float verticalAcc = (aaReal.x * gravity.x) + (aaReal.y * gravity.y) + (aaReal.z * gravity.z);
                float linearMag = sqrt(pow(aaReal.x, 2) + pow(aaReal.y, 2) + pow(aaReal.z, 2));
                float linearVertical = abs(verticalAcc);
                float gyroPower = sqrt(pow(gy.x, 2) + pow(gy.y, 2) + pow(gy.z, 2));

                float compositeSignal = (linearMag * 1.0) + (linearVertical * 1.5) + ((gyroPower / 20.0) * 0.5);

                // --- ENERGY ACCUMULATION ---
                energySum += compositeSignal * compositeSignal;
                energySamples++;
                stepSignal = 0; 
                
                // --- STEP DETECTION ---
                if (compositeSignal > COMPOSITE_THRESHOLD && (millis() - lastStepTime > STEP_MIN_DELAY_MS)) {
                    unsigned long currentTime = millis();
                    long currentInterval = currentTime - lastStepTime;
                    
                    if (lastInterval > 0) {
                        irregularityIndex = abs(currentInterval - lastInterval);
                    }
                    lastInterval = currentInterval;
                    lastStepTime = currentTime;

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

                // Serial Output
                Serial.print("Signal:"); Serial.print(compositeSignal);
                Serial.print(",Thresh:"); Serial.print(COMPOSITE_THRESHOLD);
                Serial.print(",Step:"); Serial.print(stepSignal);
                Serial.print(",TimeIrreg:"); Serial.print(irregularityIndex);
                Serial.print(",EnergyIrreg:"); Serial.println(energyIrregularity, 4);

                // Safely update shared variables for the display task
                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                    shared_compositeSignal = compositeSignal;
                    shared_stepCount = stepCount;
                    shared_energyIrregularity = energyIrregularity;
                    xSemaphoreGive(dataMutex);
                }
            }
        }
    }
}

// ==============================================================================
// TASK 2: OLED UPDATE ENGINE (Low Priority, runs every 150ms)
// ==============================================================================
void displayTask(void *pvParameters) {
    float local_mag;
    int local_steps;
    float local_irreg;

    const TickType_t xDelay = pdMS_TO_TICKS(150);

    for (;;) {
        // Safely fetch the latest variables
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            local_mag = shared_compositeSignal;
            local_steps = shared_stepCount;
            local_irreg = shared_energyIrregularity;
            xSemaphoreGive(dataMutex);
        }

        // Safely write to I2C Display
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            updateDisplay(local_mag, local_steps, local_irreg);
            xSemaphoreGive(i2cMutex);
        }

        // Delay task for 150ms (allows CPU to do other things)
        vTaskDelay(xDelay);
    }
}

// --- DISPLAY RENDERER (Now accepts parameters instead of relying on globals) ---
void updateDisplay(float mag, int steps, float irreg) {
    display.clearDisplay();
    display.setTextColor(WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("RHYTHM ANALYZER"));

    display.setCursor(0, 16);
    display.print(F("Steps: "));
    display.setTextSize(2); 
    display.print(steps);

    display.setTextSize(1);
    display.setCursor(0, 40);
    display.print(F("Irreg(E): "));
    display.print(irreg, 2);

    if(irreg > 0.20) {
        display.setCursor(90, 40);
        display.print(F("!!"));
    }

    display.setCursor(0, 54);
    display.print(F("Sig: "));
    display.print((int)mag);

    display.display();
}
