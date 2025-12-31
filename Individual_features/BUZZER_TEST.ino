// Buzzer pin
const int buzzerPin = 12;

void setup() {
  // Initialize buzzer pin as output
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);  // Buzzer OFF by default

  // Start serial communication
  Serial.begin(9600);
  Serial.println("Buzzer Control Ready");
  Serial.println("Send 1 to turn ON, 0 to turn OFF");
}

void loop() {
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
}
