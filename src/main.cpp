#include <Arduino.h>

const int ldrPin = 35; // LDR sensor connected to GPIO 35
const int adcResolution = 12; //-bit ADC resolution
const adc_attenuation_t adcAttenuation = ADC_11db; // ADC attenuation for full range (0-3.3V)



void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  analogReadResolution(adcResolution); // Set ADC resolution
  analogSetPinAttenuation(ldrPin, adcAttenuation); // Set ADC attenuation for the LDR pin
}

void loop() {
  int raw = analogRead(ldrPin); // Read raw ADC value from the LDR sensor
  float Vref = 1100.0; // Reference voltage in millivolts (1.1V)
  float Vcalc = (raw * Vref) / ((1 << adcResolution) - 1); // Calculate voltage from raw ADC value
  Vcalc /= 1000.0; // Convert millivolts to volts

  int mv = analogReadMilliVolts(ldrPin); // Read voltage in millivolts directly from the LDR sensor
  float readVoltage = mv / 1000.0; // Convert millivolts to volts
  float error = ((readVoltage - Vcalc) / readVoltage) * 100.0; // Calculate percentage error between calculated voltage and direct reading
  Serial.print("Raw ADC Value: ");
  Serial.print(raw);
  Serial.print(" | Calculated Voltage: ");
  Serial.print(Vcalc, 3); // Print calculated voltage with 3 decimal places
  Serial.print(" V | Direct Voltage: ");
  Serial.print(readVoltage, 3); // Print direct voltage reading with 3 decimal places
  Serial.print(" V | Error: ");
  Serial.print(error, 2); // Print percentage error with 2 decimal places
  Serial.println(" %");
  delay(2000); // Wait for 2 seconds before the next reading
}