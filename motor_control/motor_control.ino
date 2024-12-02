#define DIR_PIN 28    // Direction pin (DIR)
#define STEP_PIN 26   // Step pin (STEP)
#define ENABLE_PIN 24 // Enable pin (ENABLE)

// Variables for extrusion adjustment
float theoreticalLength = 100.0;  // Filament length to extrude (mm)
float actualLength = 110.0;       // Measured output length (mm)

// Correction factor calculation
float correctionFactor = theoreticalLength / actualLength;

void setup() {
  // Initialize pins as outputs
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Activate the motor
  digitalWrite(ENABLE_PIN, LOW); // LOW activates the motor
  digitalWrite(DIR_PIN, HIGH);   // Set extrusion direction
}

void loop() {
  extrude(theoreticalLength);
  delay(5000); // Pause before the next (simulated) extrusion
}

void extrude(float lengthInMM) {
  // Convert length to steps based on the correction factor
  int steps = convertMMToSteps(lengthInMM * correctionFactor);
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000); // Adjust this value for speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
}

int convertMMToSteps(float length) {
  float stepsPerMM = 100.0; // Adjust based on motor and extruder characteristics
  return int(length * stepsPerMM);
}
