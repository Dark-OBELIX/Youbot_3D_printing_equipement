// Configuration des broches pour l'extrudeur E0 avec un A4988 sur RAMPS 1.4
#define DIR_PIN 28    // Broche de direction (DIR)
#define STEP_PIN 26   // Broche de pas (STEP)
#define ENABLE_PIN 24 // Broche d'activation (ENABLE)

void setup() {
  // Initialisation des broches comme sorties
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Activation du moteur
  digitalWrite(ENABLE_PIN, LOW); // LOW active le moteur
  digitalWrite(DIR_PIN, HIGH);   // Définir la direction en avant
}

void loop() {
  // Faire tourner le moteur en continu dans une direction
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(1000); // Ajustez cette valeur pour contrôler la vitesse
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(1000); // Ajustez cette valeur pour contrôler la vitesse
}
