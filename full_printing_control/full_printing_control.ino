#include <math.h>

// Heating configuration
#define THERMISTOR_PIN A13
#define HEATER_PIN 10
#define SERIES_RESISTOR 4700
#define THERMISTOR_NOMINAL 1000
#define TEMPERATURE_NOMINAL 25
#define B_COEFFICIENT 3950
#define ADC_MAX 1023
#define SUPPLY_VOLTAGE 5
#define TEMPERATURE_HYSTERESIS 2

// Motor configuration
#define DIR_PIN 28
#define STEP_PIN 26
#define ENABLE_PIN 24

// Control variables
float targetTemperature = -1;    // Target temperature initialized to -1 (disabled)
bool motorActive = false;        // Motor status indicator

void setup() {
  Serial.begin(9600);

  // Initialize pins
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH); // Disable motor by default
}

void loop() {
  // Read commands from the serial port
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove spaces or line breaks

    if (input.equalsIgnoreCase("start")) {
      startMotor();
    } 
    else if (input.equalsIgnoreCase("stop")) {
      stopMotor();
    } 
    else {
      float temp = input.toFloat();
      if (temp > 0) {
        targetTemperature = temp;
        Serial.print("Target temperature set to: ");
        Serial.println(targetTemperature);
      }
    }
  }

  // Temperature control
  manageHeating();

  delay(100);  // Small delay to avoid overload
}

// Heating control function
void manageHeating() {
  int adcValue = analogRead(THERMISTOR_PIN);
  float resistance = SERIES_RESISTOR / ((SUPPLY_VOLTAGE / (adcValue / (float)ADC_MAX)) - 1);

  // Calculate temperature
  float temperature = resistance / THERMISTOR_NOMINAL;
  temperature = log(temperature);
  temperature /= B_COEFFICIENT;
  temperature += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  temperature = 1.0 / temperature;
  temperature -= 273.15;

  Serial.print("Current temperature: ");
  Serial.println(temperature);

  if (targetTemperature > 0) {
    if (temperature < targetTemperature - TEMPERATURE_HYSTERESIS) {
      digitalWrite(HEATER_PIN, HIGH);  // Turn on the heater
    } else if (temperature >= targetTemperature + TEMPERATURE_HYSTERESIS) {
      digitalWrite(HEATER_PIN, LOW);   // Turn off the heater
    }
  } else {
    digitalWrite(HEATER_PIN, LOW);     // Disable if no target
  }
}

// Function to start the motor
void startMotor() {
  if (!motorActive) {
    motorActive = true;
    digitalWrite(ENABLE_PIN, LOW);  // Activate the motor
    Serial.println("Motor started.");
  }

  // Run the motor continuously while active
  while (motorActive) {
    digitalWrite(DIR_PIN, HIGH);    // Set direction
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);        // Adjust this value for speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);

    // Check if a "stop" command is received
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.equalsIgnoreCase("stop")) {
        stopMotor();
      }
    }
  }
}

// Function to stop the motor
void stopMotor() {
  motorActive = false;
  digitalWrite(ENABLE_PIN, HIGH);  // Deactivate the motor
  Serial.println("Motor stopped.");
}
