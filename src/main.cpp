#include <Arduino.h>

const uint8_t ldrPin = 4; // LDR sensor connected to GPIO 4
const uint8_t adcResolution = 12; //-bit ADC resolution
const adc_attenuation_t adcAttenuation = ADC_11db; // ADC attenuation for full range (0-3.3V)



void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  analogReadResolution(adcResolution); // Set ADC resolution
  analogSetPinAttenuation(ldrPin, adcAttenuation); // Set ADC attenuation for the LDR pin
}

static const int32_t VREF_MV = 3300;
static const int32_t ADC_MAX = (1 << 12) - 1; // 4095

void loop() {
  int32_t raw     = analogRead(ldrPin);
  int32_t calcMv  = (raw * VREF_MV) / ADC_MAX;
  int32_t directMv = analogReadMilliVolts(ldrPin);

  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print(" | Calc: ");
  Serial.print(calcMv);
  Serial.print(" mV | Direct: ");
  Serial.print(directMv);
  Serial.print(" mV | Error: ");
  if (directMv == 0) {
      Serial.println("N/A");
  } else {
      int32_t errorX10 = ((directMv - calcMv) * 1000) / directMv; // tenths of a percent
      Serial.print(errorX10 / 10);
      Serial.print(".");
      Serial.print(abs(errorX10 % 10));
      Serial.println("%");
  }
  delay(2000);
}
