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

// --- NEW: BUZZER CONFIGURATION ---
#define BUZZER_PIN 12
#define BURST_THRESHOLD_MS 1300 // Max time for 3 steps to be considered a "burst"
#define BUZZER_DURATION 2000    // How long buzzer stays on (ms)

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

// --- TIMING IRREGULARITY (already present) ---
long lastInterval = 0;
long irregularityIndex = 0;

// --- NEW: ENERGY IRREGULARITY VARIABLES ---
float energySum = 0.0;
int energySamples = 0;
float prevStepEnergy = 0.0;
float energyIrregularity = 0.0;

// --- NEW: RHYTHM/BUZZER VARIABLES ---
unsigned long stepTimestamps[3] = {0, 0, 0}; // Stores time of last 3 steps
unsigned long buzzerStartTime = 0;
bool buzzerActive = false;

// Display Timer
unsigned long previousDisplayMillis = 0;
const long displayInterval = 150; 

// Interrupt Routine
volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Wire.begin();
    Wire.setClock(400000); 
    Serial.begin(115200);
    delay(100); 

    // --- NEW: BUZZER SETUP ---
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

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
}

void loop() {
    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetGyro(&gy, fifoBuffer);

        float verticalAcc =
            (aaReal.x * gravity.x) +
            (aaReal.y * gravity.y) +
            (aaReal.z * gravity.z);

        float linearMag =
            sqrt(pow(aaReal.x, 2) + pow(aaReal.y, 2) + pow(aaReal.z, 2));

        float linearVertical = abs(verticalAcc);

        float gyroPower =
            sqrt(pow(gy.x, 2) + pow(gy.y, 2) + pow(gy.z, 2));

        float compositeSignal =
            (linearMag * 1.0) +
            (linearVertical * 1.5) +
            ((gyroPower / 20.0) * 0.5);

        // --- ENERGY ACCUMULATION ---
        energySum += compositeSignal * compositeSignal;
        energySamples++;

        stepSignal = 0; 
        
        if (compositeSignal > COMPOSITE_THRESHOLD &&
            (millis() - lastStepTime > STEP_MIN_DELAY_MS)) {

            unsigned long currentTime = millis();

            // --- TIMING IRREGULARITY ---
            long currentInterval = currentTime - lastStepTime;
            if (lastInterval > 0) {
                irregularityIndex = abs(currentInterval - lastInterval);
            }
            lastInterval = currentInterval;
            lastStepTime = currentTime;

            // --- NEW: RHYTHM BURST DETECTION ---
            // Shift register for timestamps: [0]<-[1], [1]<-[2], [2]<-New
            stepTimestamps[0] = stepTimestamps[1];
            stepTimestamps[1] = stepTimestamps[2];
            stepTimestamps[2] = currentTime;

            // Check if we have 3 recorded steps and they happened fast
            if (stepTimestamps[0] != 0) {
                unsigned long durationFor3Steps = stepTimestamps[2] - stepTimestamps[0];
                
                // If 3 steps took less than BURST_THRESHOLD_MS
                if (durationFor3Steps < BURST_THRESHOLD_MS) {
                    digitalWrite(BUZZER_PIN, HIGH);
                    buzzerStartTime = millis();
                    buzzerActive = true;
                }
            }

            // --- ENERGY IRREGULARITY COMPUTATION ---
            float stepEnergy = sqrt(energySum / energySamples);

            if (prevStepEnergy > 0) {
                energyIrregularity =
                    abs(stepEnergy - prevStepEnergy) / prevStepEnergy;
            }

            prevStepEnergy = stepEnergy;
            energySum = 0;
            energySamples = 0;

            stepCount++;
            stepSignal = COMPOSITE_THRESHOLD + 2000;
        }

        // --- NEW: HANDLE BUZZER TIMEOUT (Non-blocking) ---
        if (buzzerActive && (millis() - buzzerStartTime > BUZZER_DURATION)) {
            digitalWrite(BUZZER_PIN, LOW);
            buzzerActive = false;
        }

        Serial.print("Signal:"); Serial.print(compositeSignal);
        Serial.print(",");
        Serial.print("Thresh:"); Serial.print(COMPOSITE_THRESHOLD);
        Serial.print(",");
        Serial.print("Step:"); Serial.print(stepSignal);
        Serial.print(",");
        Serial.print("TimeIrreg:"); Serial.print(irregularityIndex);
        Serial.print(",");
        Serial.print("EnergyIrreg:");
        Serial.println(energyIrregularity, 4);

        if (millis() - previousDisplayMillis >= displayInterval) {
            previousDisplayMillis = millis();
            updateDisplay(compositeSignal);
        }
    }
}

void updateDisplay(float mag) {
    display.clearDisplay();
    display.setTextColor(WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("RHYTHM ANALYZER"));

    display.setCursor(0, 16);
    display.print(F("Steps: "));
    display.setTextSize(2); 
    display.print(stepCount);

    display.setTextSize(1);
    display.setCursor(0, 40);
    display.print(F("Irreg(E): "));
    display.print(energyIrregularity, 2);

    if(energyIrregularity > 0.20) {
        display.setCursor(90, 40);
        display.print(F("!!"));
    }

    // Visual indicator for buzzer
    if(buzzerActive) {
        display.fillCircle(120, 5, 4, WHITE);
    }

    display.setCursor(0, 54);
    display.print(F("Sig: "));
    display.print((int)mag);

    display.display();
}