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
}

void loop() {
  // Faire tourner le moteur dans une direction
  digitalWrite(DIR_PIN, HIGH); // Définir la direction
  for (int i = 0; i < 200; i++) { // 200 pas pour un tour complet (à ajuster selon votre moteur)
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000); // Ajustez cette valeur pour la vitesse
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000); // Ajustez cette valeur pour la vitesse
  }
  
  delay(1000); // Pause de 1 seconde entre les rotations

  // Faire tourner le moteur dans l'autre direction
  digitalWrite(DIR_PIN, LOW); // Inverser la direction
  for (int i = 0; i < 200; i++) { // 200 pas pour un tour complet
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
  
  delay(1000); // Pause de 1 seconde entre les rotations
}
