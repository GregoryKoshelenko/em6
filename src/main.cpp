#include <Arduino.h>

const uint8_t ldrPin = 4; // LDR sensor connected to GPIO 4
const uint8_t adcResolution = 12; //-bit ADC resolution
const adc_attenuation_t adcAttenuation = ADC_11db; // ADC attenuation for full range (0-3.3V)



void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  analogReadResolution(adcResolution); // Set ADC resolution
  analogSetPinAttenuation(ldrPin, adcAttenuation); // Set ADC attenuation for the LDR pin
}

void loop() {
  int32_t raw = analogRead(ldrPin); // Read raw ADC value from the LDR sensor
  float Vref = 3300.0; // Reference voltage in millivolts (3.3V — full range with ADC_11db)
  float Vcalc = (raw * Vref) / ((1 << adcResolution) - 1); // Calculate voltage from raw ADC value
  Vcalc /= 1000.0; // Convert millivolts to volts

  int32_t mv = analogReadMilliVolts(ldrPin); // Read voltage in millivolts directly from the LDR sensor
  float readVoltage = mv / 1000.0; // Convert millivolts to volts
  float error = (readVoltage != 0.0f) ? ((readVoltage - Vcalc) / readVoltage) * 100.0f : NAN;

  Serial.print("Raw ADC Value: ");
  Serial.print(raw);
  Serial.print(" | Calculated Voltage: ");
  Serial.print(Vcalc, 3);
  Serial.print(" V | Direct Voltage: ");
  Serial.print(readVoltage, 3);
  Serial.print(" V | Error: ");
  if (isnan(error)) {
      Serial.println("N/A");
  } else {
      Serial.print(error, 2);
      Serial.println(" %");
  }
  delay(2000); // Wait for 2 seconds before the next reading
}
