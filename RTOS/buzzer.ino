/*
 * ******************************************************************************************
 * PROJECT S.H.E.R.P.A.
 * (Smart High-altitude Exertion & Real-time Pathological Analyzer)
 * ******************************************************************************************
 * * MODULE:        Audio Alert Module & Serial Override
 * FILE:            Buzzer_Test.ino
 * HARDWARE:        MYOSA Mini Kit (Core: ESP32/AVR | Actuator: Active Buzzer)
 * * ------------------------------------------------------------------------------------------
 * PROJECT ABSTRACT
 * ------------------------------------------------------------------------------------------
 * S.H.E.R.P.A. is a lumbar-mounted wearable utilizing context-aware sensor fusion to 
 * detect early markers of Acute Mountain Sickness (AMS). This module handles the audio 
 * feedback system, ensuring that life-critical alarms (Ataxia detection) can be triggered 
 * deterministically.
 * * ------------------------------------------------------------------------------------------
 * CODE FUNCTIONALITY
 * ------------------------------------------------------------------------------------------
 * This specific unit test initializes the buzzer GPIO and continuously polls the Serial 
 * interface for manual override commands ('1' for ON, '0' for OFF). It operates completely 
 * asynchronously to ensure UI commands do not block sensor polling.
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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Buzzer pin
const int buzzerPin = 12;

// --- FREE RTOS RESOURCES ---
TaskHandle_t BuzzerTaskHandle;

// --- TASK DECLARATION ---
void BuzzerControlTask(void *pvParameters);

void setup() {
  // Initialize buzzer pin as output
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);  // Buzzer OFF by default

  // Start serial communication (Keeping your original 9600 baud rate)
  Serial.begin(9600);
  Serial.println("Buzzer Control Ready");
  Serial.println("Send 1 to turn ON, 0 to turn OFF");

  /* --- CREATE RTOS TASK --- */
  // Buzzer Task: Low Priority (1), pinned to Core 0 to keep Core 1 free for sensors
  xTaskCreatePinnedToCore(
      BuzzerControlTask,   // Task function
      "BuzzerControlTask", // Task name for debugging
      2048,                // Stack size
      NULL,                // Parameters
      1,                   // Priority (1 is standard/low)
      &BuzzerTaskHandle,   // Task handle
      0                    // Pin to Core 0
  );
}

void loop() {
  // RTOS Scheduler overrides loop()
  vTaskDelete(NULL);
}

// ==========================================
// CORE 0: BUZZER & SERIAL POLLING TASK
// ==========================================
void BuzzerControlTask(void *pvParameters) {
  // Check the serial buffer every 50ms. 
  // This is fast enough for human input, but slow enough to yield CPU time.
  const TickType_t xFrequency = pdMS_TO_TICKS(50); 
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Check if data is available on Serial
    if (Serial.available() > 0) {
      char command = Serial.read();  // Read one character

      if (command == '1') {
        digitalWrite(buzzerPin, HIGH);
        Serial.println("Buzzer ON");
      }
      else if (command == '0') {
        digitalWrite(buzzerPin, LOW);
        Serial.println("Buzzer OFF");
      }
    }

    // Delay deterministically to preserve timing behavior and feed the Watchdog
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}