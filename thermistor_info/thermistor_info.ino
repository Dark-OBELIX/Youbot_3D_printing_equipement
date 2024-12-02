#include <math.h>

#define THERMISTOR_PIN A13         // Pin where the thermistor is connected
#define HEATER_PIN 10              // Pin for the heater element (D10 on RAMPS 1.4)
#define SERIES_RESISTOR 4700       // 4.7kΩ series resistor

#define THERMISTOR_NOMINAL 1000      
#define TEMPERATURE_NOMINAL 25     // Nominal temperature 25°C
#define B_COEFFICIENT 3950         // B coefficient
#define ADC_MAX 1023               // ADC resolution
#define SUPPLY_VOLTAGE 5           // Supply voltage

float targetTemperature = -1;      // Initialize without a target temperature
#define TEMPERATURE_HYSTERESIS 2   // Hysteresis to avoid frequent toggling

void setup() {
  Serial.begin(9600);
  pinMode(HEATER_PIN, OUTPUT);     // Set heater pin as output
}

void loop() {
  // Read data sent via serial port (e.g., from a Python script)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    targetTemperature = input.toFloat();  // Update target temperature
    Serial.println(targetTemperature);
  }

  int adcValue = analogRead(THERMISTOR_PIN);  // Read raw ADC value

  // Convert ADC value to resistance
  float resistance = SERIES_RESISTOR / ((SUPPLY_VOLTAGE / (adcValue / (float)ADC_MAX)) - 1);

  // Calculate temperature
  float temperature;
  temperature = resistance / THERMISTOR_NOMINAL;
  temperature = log(temperature);
  temperature /= B_COEFFICIENT;
  temperature += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  temperature = 1.0 / temperature;
  temperature -= 273.15;

  // Display the current temperature
  Serial.println(temperature);

  // If a target temperature is set, control the heater element
  if (targetTemperature > 0) {
    if (temperature < targetTemperature - TEMPERATURE_HYSTERESIS) {
      digitalWrite(HEATER_PIN, HIGH);  // Turn the heater on
    } else if (temperature >= targetTemperature + TEMPERATURE_HYSTERESIS) {
      digitalWrite(HEATER_PIN, LOW);   // Turn the heater off
    }
  } else {
    digitalWrite(HEATER_PIN, LOW);   // Turn the heater off if no target is set
  }

  delay(1000);  // Wait one second before the next reading
}
