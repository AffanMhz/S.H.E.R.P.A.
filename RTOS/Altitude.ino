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
 * [UNIT TEST] Altitude & Vertical Speed Monitor
 * This module tests the core barometric pressure sensing engine. It utilizes 
 * a low-pass filter to calculate relative height and vertical speed (rate of 
 * ascent/descent). Monitoring rapid elevation changes is a critical environmental 
 * metric for correlating with the onset of Acute Mountain Sickness (AMS).
 *
 * HARDWARE CONFIGURATION:
 * - Platform: MyoSa Mini Kit
 * - MCU: ESPRESSIF ESP32-WROOM-32E
 * - Sensor: BMP085 / BMP180 Barometric Pressure Sensor (I2C)
 * - Display: SSD1306 OLED (I2C Address 0x3C)
 *

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


#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/* --- CONFIGURATION --- */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 
#define I2C_SDA       21
#define I2C_SCL       22

// Objects
Adafruit_BMP085 bmp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- FREE RTOS RESOURCES ---
SemaphoreHandle_t i2cMutex;
TaskHandle_t SensorTaskHandle;
TaskHandle_t DisplayTaskHandle;

// --- SENSOR VARIABLES (Volatile for cross-task sharing) ---
volatile float baselineAltitude = 0.0;
volatile float smoothedAltitude = 0.0;
const float alpha = 0.1; 
volatile float prevAltitude = 0.0;
volatile unsigned long prevTime = 0;
volatile float verticalSpeed = 0.0;
volatile float relativeHeight = 0.0;
String statusStr = "STABLE"; 

// Thresholds
const float MOVEMENT_THRESHOLD = 0.2; 

// --- TASK DECLARATIONS ---
void SensorTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void centerText(String text, int y, int size); 

void setup() {
  Serial.begin(115200);
  
  // Create Mutex for I2C Bus protection
  i2cMutex = xSemaphoreCreateMutex();

  // 1. Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // 2. Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed. Check wiring or try 0x3D"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Show Boot Screen
  centerText("INIT SENSORS", 25, 1);
  display.display();
  delay(1000);

  // 3. Initialize BMP Sensor
  if (!bmp.begin()) {
    display.clearDisplay();
    centerText("BMP ERROR", 20, 2);
    centerText("Check Wiring", 45, 1);
    display.display();
    while (1);
  }

  // 4. Calibration (Tare)
  display.clearDisplay();
  centerText("CALIBRATING", 20, 1);
  centerText("Don't Move...", 40, 1);
  display.display();

  float sum = 0;
  int samples = 50;
  for(int i = 0; i < samples; i++) {
    sum += bmp.readAltitude();
    delay(20); 
  }
  baselineAltitude = sum / samples;
  smoothedAltitude = baselineAltitude;
  prevAltitude = smoothedAltitude;
  prevTime = millis();

  // --- CREATE RTOS TASKS ---
  // Sensor Task: High Priority (3), pinned to Core 1
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 3, &SensorTaskHandle, 1);

  // Display Task: Low Priority (1), pinned to Core 0
  xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 4096, NULL, 1, &DisplayTaskHandle, 0);
}

void loop() {
  // Empty. RTOS scheduler takes over automatically.
  vTaskDelete(NULL); 
}

// ==========================================
// CORE 1: SENSOR LOGIC TASK (HIGH PRIORITY)
// ==========================================
void SensorTask(void *pvParameters) {
  // Run sensor logic at a clean 20ms interval (50Hz) for smooth filtering
  const TickType_t xFrequency = pdMS_TO_TICKS(20); 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    float currentRawAltitude = 0;
    bool readSuccess = false;
    
    // Lock I2C to read BMP085 safely
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
      currentRawAltitude = bmp.readAltitude();
      readSuccess = true;
      xSemaphoreGive(i2cMutex);
    }

    if (readSuccess) {
      unsigned long currentMillis = millis();

      // Low-Pass Filter
      smoothedAltitude = (currentRawAltitude * alpha) + (smoothedAltitude * (1.0 - alpha));
      
      // Relative Height
      relativeHeight = smoothedAltitude - baselineAltitude;

      // Vertical Speed Calculation
      float dt = (currentMillis - prevTime) / 1000.0;

      if (dt > 0.2) { // Update speed calc every 200ms to avoid jitter
        verticalSpeed = (smoothedAltitude - prevAltitude) / dt;
        
        if (verticalSpeed > MOVEMENT_THRESHOLD) statusStr = "ASCENDING";
        else if (verticalSpeed < -MOVEMENT_THRESHOLD) statusStr = "DESCENDING";
        else statusStr = "STABLE";

        prevAltitude = smoothedAltitude;
        prevTime = currentMillis;
      }
    }

    // Delay precisely to maintain timing behavior
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==========================================
// CORE 0: DISPLAY TASK (LOW PRIORITY)
// ==========================================
void DisplayTask(void *pvParameters) {
  const TickType_t xDisplayFrequency = pdMS_TO_TICKS(100); // Original 100ms refresh rate
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Create thread-safe local copies of variables before drawing
    float localHeight = relativeHeight;
    float localSpeed = verticalSpeed;
    String localStatus = statusStr;

    // Lock I2C Bus to update Display
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      display.clearDisplay();

      // --- Header ---
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("BMP DASHBOARD");
      display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

      // --- Row 1: Relative Height ---
      display.setCursor(0, 15);
      display.setTextSize(1);
      display.print("Hgt:"); 
      
      display.setCursor(30, 12);
      display.setTextSize(2);
      display.print(localHeight, 1); 
      display.setTextSize(1);
      display.print(" m");

      // --- Row 2: Vertical Speed ---
      display.setCursor(0, 38);
      display.setTextSize(1);
      display.print("Spd:");
      
      display.setCursor(30, 35);
      display.setTextSize(2);
      display.print(localSpeed, 2); 
      display.setTextSize(1);
      display.print(" m/s");

      // --- Row 3: Status Bar (Bottom) ---
      display.fillRect(0, 54, 128, 10, SSD1306_WHITE); 
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); 
      centerText(localStatus, 55, 1); 
      display.setTextColor(SSD1306_WHITE); 

      display.display();
      
      // Unlock I2C Bus
      xSemaphoreGive(i2cMutex);
    }

    // Sleep task until the next 100ms interval
    vTaskDelayUntil(&xLastWakeTime, xDisplayFrequency);
  }
}

// --- VISUAL FUNCTIONS ---
// Note: This helper function is only ever called from thread-safe contexts 
// (either in setup() or while the I2C mutex is held in the DisplayTask)
void centerText(String text, int y, int size) {
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(size);
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, y);
  display.print(text);
}