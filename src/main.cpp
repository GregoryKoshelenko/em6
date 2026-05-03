// Police Flasher - ESP32 HW13

#include <Arduino.h>

const int RED_LED = 2;
const int BLUE_LED = 21;
const int BUTTON_PIN = 7;

unsigned long redTimer = 0;
unsigned long blueTimer = 0;
unsigned long lastButtonPress = 0;

const unsigned long DEBOUNCE_DELAY = 300;  // 300ms debounce
bool redState = false;
bool blueState = false;
bool flasherEnabled = false;  // true=ON, false=OFF
bool showRed = true;

// Interrupt handler for button press
void IRAM_ATTR buttonPressed() {
  unsigned long currentTime = millis();
  
  // Debounce check - ignore if pressed too soon
  if (currentTime - lastButtonPress < DEBOUNCE_DELAY) {
    return;
  }
  
  lastButtonPress = currentTime;
  flasherEnabled = !flasherEnabled;  // Toggle ON/OFF
  
  // Turn off both LEDs when toggled off
  if (!flasherEnabled) {
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    redState = false;
    blueState = false;
  }
  
  if (flasherEnabled) {
    Serial.println("Flasher ON");
  } else {
    Serial.println("Flasher OFF");
  }
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Attach interrupt to button pin (RISING when released)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, RISING);
  
  Serial.println("\n\nSetup complete - LEDs initialized");
  Serial.flush();
  delay(100);
  Serial.println("Police Flasher Ready!");
  Serial.flush();
  delay(100);
  Serial.println("Button on pin 7 - Press to toggle ON/OFF");
  Serial.flush();
}

void loop() {
  if (!flasherEnabled) {
    // Flasher is off - all LEDs stay off
    return;
  }
  
  if (millis() - redTimer >= 500) {
    redState = !redState;
    blueState = !blueState;
    
    // Alternate between red and blue
    if (redState) {
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, LOW);
      Serial.println("Red LED ON");
      Serial.flush();
    } else {
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
      Serial.println("Blue LED ON");
      Serial.flush();
    }
    
    redTimer = millis();
  }
}
