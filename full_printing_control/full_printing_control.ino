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
float targetTemperature = 185;  // Default target temperature in Celsius
unsigned long motorRunDuration = 5000; // Default motor run duration in milliseconds
bool motorActive = false;       // Motor status indicator
bool startCommandReceived = false; // Indicates if the "Start" command has been received

void setup() {
  Serial.begin(9600);

  // Initialize pins
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH); // Disable motor by default
  Serial.println("[INFO] : System initialized. Send 'Start <temperature> <duration>' to begin.");
}

void loop() {
  // Check for user command
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace and newline characters

    if (input.startsWith("Start")) {
      parseStartCommand(input);
    }
  }

  // Proceed only if the start command is received
  if (startCommandReceived) {
    // Temperature control
    if (manageHeating()) {
      Serial.println("[INFO] : Temperature OK");

      // Run the motor for a defined duration
      startMotor(motorRunDuration);

      // Stop the motor and start cooling
      Serial.println("[INFO] : Motor Off");
      coolDown();

      // Reset the start command to prevent looping
      startCommandReceived = false;
      Serial.println("[INFO] : Process complete. Send 'Start <temperature> <duration>' to begin again.");
    }
  }

  delay(100);  // Small delay to avoid overload
}

// Function to parse the Start command and extract parameters
void parseStartCommand(String command) {
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);

  if (firstSpace > 0 && secondSpace > firstSpace) {
    String tempStr = command.substring(firstSpace + 1, secondSpace);
    String durationStr = command.substring(secondSpace + 1);

    float temp = tempStr.toFloat();
    unsigned long duration = durationStr.toInt();

    if (temp > 0 && duration > 0) {
      targetTemperature = temp;
      motorRunDuration = duration;
      startCommandReceived = true;

      Serial.print("[INFO] : Start command received with target temperature: ");
      Serial.print(targetTemperature);
      Serial.print("[INFO] :  and motor run duration: ");
      Serial.println(motorRunDuration);
    } else {
      Serial.println("[ERROR] : Invalid parameters. Use 'Start <temperature> <duration>'.");
    }
  } else {
    Serial.println("[ERROR] : Invalid command format. Use 'Start <temperature> <duration>'.");
  }
}

// Heating control function
bool manageHeating() {
  int adcValue = analogRead(THERMISTOR_PIN);
  float resistance = SERIES_RESISTOR / ((SUPPLY_VOLTAGE / (adcValue / (float)ADC_MAX)) - 1);

  // Calculate temperature
  float temperature = resistance / THERMISTOR_NOMINAL;
  temperature = log(temperature);
  temperature /= B_COEFFICIENT;
  temperature += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  temperature = 1.0 / temperature;
  temperature -= 273.15;

  Serial.print("[TEMPERATURE] : ");
  Serial.println(temperature);

  if (temperature < targetTemperature - TEMPERATURE_HYSTERESIS) {
    digitalWrite(HEATER_PIN, HIGH);  // Turn on the heater
  } else if (temperature >= targetTemperature) {
    digitalWrite(HEATER_PIN, LOW);   // Turn off the heater
    Serial.println("[INFO] : Temperature reached."); // Confirmation for C++
    return true;  // Temperature reached
  }

  return false;  // Temperature not yet reached
}

// Function to start the motor
void startMotor(unsigned long duration) {
  motorActive = true;
  digitalWrite(ENABLE_PIN, LOW);  // Activate the motor
  Serial.println("[INFO] : Motor started.");

  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(DIR_PIN, HIGH);    // Set direction
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);        // Adjust this value for speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }

  motorActive = false;
  digitalWrite(ENABLE_PIN, HIGH);  // Deactivate the motor
}

// Cooling function
void coolDown() {
  Serial.println("[INFO] : Cooling extruder...");
  digitalWrite(HEATER_PIN, LOW); // Ensure heater is off
  delay(30000); // Cool down for 30 seconds (adjust if needed)
  Serial.println("[INFO] : Cooling complete.");
}
