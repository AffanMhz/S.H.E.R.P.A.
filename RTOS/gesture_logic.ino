/*
 * ******************************************************************************************
 * PROJECT S.H.E.R.P.A.
 * (Smart High-altitude Exertion & Real-time Pathological Analyzer)
 * ******************************************************************************************
 * * MODULE:        Gesture Control & Touchless UI Interface
 * FILE:            Gesture_Test.ino
 * HARDWARE:        MYOSA Mini Kit (Core: ESP32/AVR | Sensor: APDS9960 I2C)
 * * ------------------------------------------------------------------------------------------
 * PROJECT ABSTRACT
 * ------------------------------------------------------------------------------------------
 * S.H.E.R.P.A. is a lumbar-mounted wearable utilizing context-aware sensor fusion to 
 * detect early markers of Acute Mountain Sickness (AMS). This module provides a 
 * touchless interface, allowing a trekker to dismiss alarms or interact with the UI 
 * using hand gestures (via the APDS9960) without needing to remove heavy winter gloves.
 * * ------------------------------------------------------------------------------------------
 * CODE FUNCTIONALITY
 * ------------------------------------------------------------------------------------------
 * This specific module initializes and continuously polls the APDS9960 sensor engine.
 * It waits for a valid gesture "ping" and decodes the directional movement.
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
 /* Library Inclusion */
#include <LightProximityAndGesture.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/* Creating Object of LightProximityAndGesture Class */
LightProximityAndGesture Lpg;

// --- FREE RTOS RESOURCES ---
SemaphoreHandle_t i2cMutex;
TaskHandle_t GestureTaskHandle;

// --- TASK DECLARATION ---
void GestureTask(void *pvParameters);

/* Setup Function */
void setup() {

  /* Setting up communication */
  Serial.begin(115200);
  
  // Create Mutex for I2C Bus protection BEFORE initializing I2C devices
  i2cMutex = xSemaphoreCreateMutex();

  Wire.begin();
  Wire.setClock(100000);
  
  /* Setting up the LightProximityAndGesture Board. */
  for(;;)
  {
    if(Lpg.begin())
    {
      Serial.println("Proximity, Ambient Light, RGB & Gesture sensor is connected...");
      break;
    }
    Serial.println("Proximity, Ambient Light, RGB & Gesture sensor is disconnected...");
    delay(500u);
  }
  Serial.println("APDS9960 initialization completed");

  /* Start running the gesture sensor engine */
  // Preserving your exact configuration argument (DISABLE)
  if( Lpg.enableGestureSensor(DISABLE) )
  {
    Serial.println("Gesture sensor is now running");
  }
  else
  {
    Serial.println("Something went wrong during gesture sensor init!");
  }

  /* Wait for initialization and calibration to finish */
  delay(500u);

  /* --- CREATE RTOS TASK --- */
  // Gesture Task: Medium Priority (2), pinned to Core 1
  xTaskCreatePinnedToCore(
      GestureTask, 
      "GestureTask", 
      4096, 
      NULL, 
      2, 
      &GestureTaskHandle, 
      1
  );
}

/* Loop Function */
void loop() {
  // RTOS Scheduler overrides loop()
  vTaskDelete(NULL);
}

// ==========================================
// CORE 1: GESTURE POLLING TASK (MEDIUM PRIORITY)
// ==========================================
void GestureTask(void *pvParameters) {
  // Poll the gesture sensor at 50Hz (every 20ms) to ensure no missed swipes
  const TickType_t xFrequency = pdMS_TO_TICKS(20); 
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    
    // Lock the I2C Bus to communicate with the APDS9960
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      
      /* Task continuously reads gesture data and prints */
      if(Lpg.ping())
      {
        Lpg.getGesture();
      }
      
      // Unlock the I2C Bus immediately after use
      xSemaphoreGive(i2cMutex);
    }

    // Delay deterministically to preserve timing behavior and feed the Watchdog
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}