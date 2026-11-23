#include <Arduino.h>

const int RED_LED = 2;
const int BLUE_LED = 21;
const int BUTTON_PIN = 7;
const int BOOT_BUTTON_PIN = 0;

// Blinking mode timings
const unsigned long FAST_BLINK_INTERVAL_MS = 200;
const unsigned long SLOW_BLINK_INTERVAL_MS = 800;

unsigned long ledTimer = 0;
unsigned long lastButtonPress = 0;
unsigned long lastBootButtonPress = 0;

const unsigned long DEBOUNCE_DELAY_MS = 50;
bool ledState = false;
volatile unsigned long currentBlinkInterval = SLOW_BLINK_INTERVAL_MS;
volatile bool modeChanged = false;  // set in ISR, reported in loop()

// Interrupt handler for external button press (fast mode)
void IRAM_ATTR buttonPressed() {
  unsigned long currentTime = millis();

  // Debounce check
  if (currentTime - lastButtonPress < DEBOUNCE_DELAY_MS) {
    return;
  }

  lastButtonPress = currentTime;
  currentBlinkInterval = FAST_BLINK_INTERVAL_MS;
  modeChanged = true;
}

// Interrupt handler for BOOT button press (slow mode)
void IRAM_ATTR bootButtonPressed() {
  unsigned long currentTime = millis();

  // Debounce check
  if (currentTime - lastBootButtonPress < DEBOUNCE_DELAY_MS) {
    return;
  }

  lastBootButtonPress = currentTime;
  currentBlinkInterval = SLOW_BLINK_INTERVAL_MS;
  modeChanged = true;
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
  Serial.println("Dual LED Flasher Ready!");
  Serial.println("Button on pin 7 - Press for fast blink");
  Serial.println("BOOT button (GPIO0) - Press for slow blink");
}

void loop() {
  // Serial output only here, not in the ISR
  if (modeChanged) {
    modeChanged = false;
    Serial.println(currentBlinkInterval == FAST_BLINK_INTERVAL_MS
                       ? "Fast blink mode activated"
                       : "Slow blink mode activated");
  }

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

    ledTimer = millis();
  }
}
