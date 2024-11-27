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
  // Inversion de la direction
  digitalWrite(DIR_PIN, HIGH);   // Tourne dans un sens
  tournerMoteur(1000);           // Faire tourner le moteur pendant 1000 pas

  delay(1000);                   // Pause de 1 seconde

  digitalWrite(DIR_PIN, LOW);    // Tourne dans l'autre sens
  tournerMoteur(1000);           // Faire tourner le moteur pendant 1000 pas

  delay(1000);                   // Pause de 1 seconde
}

void tournerMoteur(int nombreDePas) {
  for (int i = 0; i < nombreDePas; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000); // Ajustez cette valeur pour la vitesse
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000); // Ajustez cette valeur pour la vitesse
  }
}
