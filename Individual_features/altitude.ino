#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

/* --- CONFIGURATION --- */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 
#define I2C_SDA       21
#define I2C_SCL       22

// Objects
Adafruit_BMP085 bmp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- SENSOR VARIABLES ---
float baselineAltitude = 0.0;
float smoothedAltitude = 0.0;
const float alpha = 0.1; // Smoothing factor
float prevAltitude = 0.0;
unsigned long prevTime = 0;
float verticalSpeed = 0.0;
float relativeHeight = 0.0;
String statusStr = "STABLE";

// Thresholds
const float MOVEMENT_THRESHOLD = 0.2; 

// --- DISPLAY TIMING ---
unsigned long lastScreenRefresh = 0;
const int refreshRate = 100;    // Update numbers every 100ms

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // 2. Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for most OLEDs
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
}

void loop() {
  unsigned long currentMillis = millis();

  // --- 1. SENSOR LOGIC (Run continuously) ---
  float currentRawAltitude = bmp.readAltitude();
  
  // Low-Pass Filter
  smoothedAltitude = (currentRawAltitude * alpha) + (smoothedAltitude * (1.0 - alpha));
  
  // Relative Height
  relativeHeight = smoothedAltitude - baselineAltitude;

  // Vertical Speed Calculation
  // Calculate time difference in seconds (dt)
  float dt = (currentMillis - prevTime) / 1000.0;

  if (dt > 0.2) { // Update speed calc every 200ms to avoid jitter
    verticalSpeed = (smoothedAltitude - prevAltitude) / dt;
    
    if (verticalSpeed > MOVEMENT_THRESHOLD) statusStr = "ASCENDING";
    else if (verticalSpeed < -MOVEMENT_THRESHOLD) statusStr = "DESCENDING";
    else statusStr = "STABLE";

    prevAltitude = smoothedAltitude;
    prevTime = currentMillis;
  }

  // --- 2. DISPLAY LOGIC ---
  // Screen Refresh Logic (Every 100ms - Makes it look "Live")
  if (currentMillis - lastScreenRefresh >= refreshRate) {
    lastScreenRefresh = currentMillis;
    updateDisplayContent();
  }
}

// --- VISUAL FUNCTIONS ---

void centerText(String text, int y, int size) {
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(size);
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, y);
  display.print(text);
}

void updateDisplayContent() {
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
  display.print(relativeHeight, 1); // 1 decimal place
  display.setTextSize(1);
  display.print(" m");

  // --- Row 2: Vertical Speed ---
  display.setCursor(0, 38);
  display.setTextSize(1);
  display.print("Spd:");
  
  display.setCursor(30, 35);
  display.setTextSize(2);
  display.print(verticalSpeed, 2); // 2 decimal places
  display.setTextSize(1);
  display.print(" m/s");

  // --- Row 3: Status Bar (Bottom) ---
  display.fillRect(0, 54, 128, 10, SSD1306_WHITE); // White background
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Black text on white
  centerText(statusStr, 55, 1); // Centered status text
  display.setTextColor(SSD1306_WHITE); // Reset to White text

  display.display();
}