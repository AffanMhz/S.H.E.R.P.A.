/* Library Inclusion */
#include <LightProximityAndGesture.h>

/* Creating Object of LightProximityAndGesture Class */
LightProximityAndGesture Lpg;

/* Buzzer Configuration */
#define BUZZER_PIN 12

/* Variables for "Back and Forth" detection */
unsigned long lastGestureTime = 0;
String firstGesture = ""; // An empty string means no gesture has been recorded yet
const unsigned long GESTURE_TIMEOUT = 2000; // Time in milliseconds (2 seconds)

void setup() {
  /* Setting up communication */
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  
  /* Setting up the Buzzer Pin */
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure the buzzer starts turned off
  
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
}

void loop() {
  /* Check if there is gesture data available */
  if(Lpg.ping())
  {
    // Capture the gesture as a String
    String currentGesture = String(Lpg.getGesture()); 
    
    // Print the gesture to the Serial Monitor so we can see exactly what text it uses
    Serial.print("Gesture detected: ");
    Serial.println(currentGesture);
    
    // We only care about Left and Right swipes for a "back and forth" motion.
    // NOTE: If your sensor outputs "Left" instead of "LEFT", change the text below!
    if (currentGesture == "LEFT" || currentGesture == "RIGHT" || currentGesture == "Left" || currentGesture == "Right") {
      
      unsigned long currentTime = millis();
      
      // Step 1: Detect the first wave
      if (firstGesture == "") {
        firstGesture = currentGesture;
        lastGestureTime = currentTime;
        Serial.println("First wave detected. Waiting for return wave...");
      } 
      // Step 2: Check for the return wave
      else {
        // Did the return wave happen within the time limit?
        if (currentTime - lastGestureTime <= GESTURE_TIMEOUT) {
          
          // Is it the opposite direction of the first wave?
          if (currentGesture != firstGesture) {
            Serial.println("Back and forth gesture completed!");
            
            /* --- TRIGGER THE BUZZER --- */
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500); // Wait 500 milliseconds
            digitalWrite(BUZZER_PIN, LOW);
            
            // Reset the gesture state so it can happen again
            firstGesture = ""; 
          } else {
            // Swiped the same direction twice; restart the timer
            lastGestureTime = currentTime;
          }
        } else {
          // Time ran out! Treat this new swipe as the start of a new sequence
          firstGesture = currentGesture;
          lastGestureTime = currentTime;
          Serial.println("Too slow! Starting a new sequence...");
        }
      }
    }
  }
  
  // Step 3: Automatically clear the memory if too much time passes with no second wave
  if (firstGesture != "" && (millis() - lastGestureTime > GESTURE_TIMEOUT)) {
    firstGesture = "";
    Serial.println("Timeout: Gesture reset.");
  }
}
