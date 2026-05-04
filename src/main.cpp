#include <Arduino.h>

const int RED_LED = 2;
const int BLUE_LED = 21;
const int BUTTON_PIN = 7;
const int BOOT_BUTTON_PIN = 0;

// Blinking mode timings
const unsigned long FAST_BLINK_INTERVAL = 200;  // 200ms
const unsigned long SLOW_BLINK_INTERVAL = 800;  // 800ms

unsigned long ledTimer = 0;
unsigned long lastButtonPress = 0;
unsigned long lastBootButtonPress = 0;

const unsigned long DEBOUNCE_DELAY = 50;  // 50ms debounce
bool ledState = false;
unsigned long currentBlinkInterval = SLOW_BLINK_INTERVAL;

// Interrupt handler for external button press (fast mode)
void IRAM_ATTR buttonPressed() {
  unsigned long currentTime = millis();
  
  // Debounce check
  if (currentTime - lastButtonPress < DEBOUNCE_DELAY) {
    return;
  }
  
  lastButtonPress = currentTime;
  currentBlinkInterval = FAST_BLINK_INTERVAL;
  Serial.println("Fast blink mode activated");
  Serial.flush();
}

// Interrupt handler for BOOT button press (slow mode)
void IRAM_ATTR bootButtonPressed() {
  unsigned long currentTime = millis();
  
  // Debounce check
  if (currentTime - lastBootButtonPress < DEBOUNCE_DELAY) {
    return;
  }
  
  lastBootButtonPress = currentTime;
  currentBlinkInterval = SLOW_BLINK_INTERVAL;
  Serial.println("Slow blink mode activated");
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  
  // Attach interrupts to buttons (RISING for button release detection)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, RISING);
  attachInterrupt(digitalPinToInterrupt(BOOT_BUTTON_PIN), bootButtonPressed, RISING);
  
  Serial.println("\n\nSetup complete - LEDs initialized");
  Serial.flush();
  delay(100);
  Serial.println("Dual LED Flasher Ready!");
  Serial.flush();
  delay(100);
  Serial.println("Button on pin 7 - Press for fast blink");
  Serial.println("BOOT button (GPIO0) - Press for slow blink");
  Serial.flush();
}

void loop() {
  if (millis() - ledTimer >= currentBlinkInterval / 2) {
    ledState = !ledState;
    
    if (ledState) {
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, HIGH);
      Serial.println("LEDs ON");
    } else {
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      Serial.println("LEDs OFF");
    }
    
    Serial.flush();
    ledTimer = millis();
  }
}