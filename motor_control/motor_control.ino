#define DIR_PIN 28    // Broche de direction (DIR)
#define STEP_PIN 26   // Broche de pas (STEP)
#define ENABLE_PIN 24 // Broche d'activation (ENABLE)

// Variables pour l'ajustement de l'extrusion
float longueurTheorique = 100.0;  // Longueur de filament à extruder (mm)
float longueurReelle = 110;      // Longueur mesurée en sortie (mm)

// Calcul du facteur de correction
float facteurCorrection = longueurTheorique / longueurReelle;

void setup() {
  // Initialisation des broches comme sorties
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Activation du moteur
  digitalWrite(ENABLE_PIN, LOW); // LOW active le moteur
  digitalWrite(DIR_PIN, HIGH);   // Sens de l'extrusion
}

void loop() {
  extruder(longueurTheorique);
  delay(5000); // Pause avant la prochaine extrusion (simulée)
}

void extruder(float longueurEnMM) {
  // Convertir la longueur en nombre de pas en fonction du facteur de correction
  int nombreDePas = convertirMMEnPas(longueurEnMM * facteurCorrection);
  
  for (int i = 0; i < nombreDePas; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000); // Ajustez cette valeur pour la vitesse
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
}

int convertirMMEnPas(float longueur) {
  float pasParMM = 100.0; // Ajustez en fonction des caractéristiques du moteur et de l'extrudeur
  return int(longueur * pasParMM);
}
