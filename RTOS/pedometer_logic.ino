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
 * [UNIT TEST] Pedometer & Gait Irregularity Analyzer
 * This module tests the core step-detection engine using DMP Sensor Fusion. It 
 * calculates the "Irregularity Index" (Step Jitter) to quantify walking rhythm 
 * stability, a key metric for detecting the onset of AMS-induced ataxia.
 *

 * HARDWARE CONFIGURATION:
 * - Platform: MyoSa Mini Kit
 * - MCU: ESP32 (WROOM-32)
 * - IMU: MPU6050 (Address 0x69)
 * - Display: SSD1306 OLED (I2C)
 *

 * ---------------------------------------------------------------------------------------
 * TEAM MEMBERS & ROLES:
 * 1. Affan Danish    | Lead Firmware Architect
 * 2. Zahaib          | Project Specialist
 * 3. Ammar           | Systems Engineer
 * 4. Maaz            | project design
 * ---------------------------------------------------------------------------------------
 *
 * RTOS PORT - VERSION 2.2 (DMP Fusion + Rhythm Analysis + FreeRTOS)
 */



#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

// --- STEP & TIMING VARIABLES ---
unsigned long lastStepTime = 0;
int stepCount = 0;
int stepSignal = 0;
long lastInterval = 0;
long irregularityIndex = 0;

// --- ENERGY IRREGULARITY VARIABLES ---
float energySum = 0.0;
int energySamples = 0;
float prevStepEnergy = 0.0;
float energyIrregularity = 0.0;

// --- RTOS HANDLES ---
SemaphoreHandle_t i2cMutex;      // Protects the shared I2C bus (Wire)
SemaphoreHandle_t dataMutex;     // Protects shared variables between tasks
SemaphoreHandle_t dmpSemaphore;  // Unblocks the sensor task when data is ready

// Shared display data struct
struct {
    int steps;
    float energyIrreg;
    float signal;
} currentDisplayData;

// --- INTERRUPT SERVICE ROUTINE (ISR) ---
void IRAM_ATTR dmpDataReady() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Give semaphore to unblock the sensor task immediately
    xSemaphoreGiveFromISR(dmpSemaphore, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// --- TASK PROTOTYPES ---
void vSensorTask(void *pvParameters);
void vDisplayTask(void *pvParameters);

void setup() {
    Wire.begin();
    Wire.setClock(400000); 
    Serial.begin(115200);
    delay(100); 

    // Initialize RTOS Primitives
    i2cMutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    dmpSemaphore = xSemaphoreCreateBinary();

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
        
        // Create Tasks only after everything is successfully initialized
        // Core 1 is generally better for user tasks on ESP32
        xTaskCreatePinnedToCore(vSensorTask, "Sensor Task", 8192, NULL, 2, NULL, 1);
        xTaskCreatePinnedToCore(vDisplayTask, "Display Task", 4096, NULL, 1, NULL, 1);
    }
}

void loop() {
    // Empty. RTOS tasks handle everything now.
    vTaskDelete(NULL); 
}

// --- SENSOR PROCESSING TASK (Priority 2 - High) ---
void vSensorTask(void *pvParameters) {
    for (;;) {
        // Wait indefinitely for the ISR to signal that DMP data is ready
        if (xSemaphoreTake(dmpSemaphore, portMAX_DELAY) == pdTRUE) {
            
            bool packetReady = false;
            
            // Take I2C Mutex exclusively to read MPU FIFO safely
            if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
                packetReady = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
                xSemaphoreGive(i2cMutex); // Release I2C immediately
            }

            if (packetReady) { 
                // FIFO parsing does NOT require I2C, safe to do outside mutex
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

                energySum += compositeSignal * compositeSignal;
                energySamples++;
                stepSignal = 0; 
                
                unsigned long currentTime = millis(); // Using millis() is fine here, or xTaskGetTickCount()

                if (compositeSignal > COMPOSITE_THRESHOLD && (currentTime - lastStepTime > STEP_MIN_DELAY_MS)) {

                    long currentInterval = currentTime - lastStepTime;
                    if (lastInterval > 0) {
                        irregularityIndex = abs(currentInterval - lastInterval);
                    }
                    lastInterval = currentInterval;
                    lastStepTime = currentTime;

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

                // Serial printing is generally thread-safe on ESP32, but good to keep lean
                Serial.print("Signal:"); Serial.print(compositeSignal);
                Serial.print(",Thresh:"); Serial.print(COMPOSITE_THRESHOLD);
                Serial.print(",Step:"); Serial.print(stepSignal);
                Serial.print(",TimeIrreg:"); Serial.print(irregularityIndex);
                Serial.print(",EnergyIrreg:"); Serial.println(energyIrregularity, 4);

                // Safely update the shared display variables
                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                    currentDisplayData.signal = compositeSignal;
                    currentDisplayData.steps = stepCount;
                    currentDisplayData.energyIrreg = energyIrregularity;
                    xSemaphoreGive(dataMutex);
                }
            }
        }
    }
}

// --- DISPLAY UPDATE TASK (Priority 1 - Low) ---
void vDisplayTask(void *pvParameters) {
    float localSignal = 0;
    int localSteps = 0;
    float localEnergyIrreg = 0;

    for (;;) {
        // Run exactly every 150ms (replaces the millis() timer)
        vTaskDelay(pdMS_TO_TICKS(150));

        // Safely fetch the latest calculation data
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            localSignal = currentDisplayData.signal;
            localSteps = currentDisplayData.steps;
            localEnergyIrreg = currentDisplayData.energyIrreg;
            xSemaphoreGive(dataMutex);
        }

        // Take I2C Mutex exclusively to write to OLED safely
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            display.clearDisplay();
            display.setTextColor(WHITE);

            display.setTextSize(1);
            display.setCursor(0, 0);
            display.print(F("RHYTHM ANALYZER"));

            display.setCursor(0, 16);
            display.print(F("Steps: "));
            display.setTextSize(2); 
            display.print(localSteps);

            display.setTextSize(1);
            display.setCursor(0, 40);
            display.print(F("Irreg(E): "));
            display.print(localEnergyIrreg, 2);

            if(localEnergyIrreg > 0.20) {
                display.setCursor(90, 40);
                display.print(F("!!"));
            }

            display.setCursor(0, 54);
            display.print(F("Sig: "));
            display.print((int)localSignal);

            display.display();
            xSemaphoreGive(i2cMutex); // Done with I2C
        }
    }
}
