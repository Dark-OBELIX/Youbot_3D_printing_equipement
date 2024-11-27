#include <math.h>

// Configuration de la chauffe
#define THERMISTOR_PIN A13
#define HEATER_PIN 10
#define SERIES_RESISTOR 4700
#define THERMISTOR_NOMINAL 1000
#define TEMPERATURE_NOMINAL 25
#define B_COEFFICIENT 3950
#define ADC_MAX 1023
#define SUPPLY_VOLTAGE 5
#define TEMPERATURE_HYSTERESIS 2

// Configuration du moteur
#define DIR_PIN 28
#define STEP_PIN 26
#define ENABLE_PIN 24

// Variables de contrôle
float targetTemperature = -1;    // Température cible initialisée à -1 (désactivée)
bool moteurActif = false;        // Indicateur du statut du moteur

void setup() {
  Serial.begin(9600);

  // Initialisation des broches
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH); // Désactive le moteur par défaut
}

void loop() {
  // Lecture des commandes depuis le port série
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Supprime les espaces ou retours à la ligne

    if (input.equalsIgnoreCase("start")) {
      startMoteur();
    } 
    else if (input.equalsIgnoreCase("stop")) {
      stopMoteur();
    } 
    else {
      float temp = input.toFloat();
      if (temp > 0) {
        targetTemperature = temp;
        Serial.print("Température cible définie à : ");
        Serial.println(targetTemperature);
      }
    }
  }

  // Contrôle de la température
  gererChauffe();

  delay(100);  // Petit délai pour éviter une surcharge
}

// Fonction de contrôle du chauffage
void gererChauffe() {
  int adcValue = analogRead(THERMISTOR_PIN);
  float resistance = SERIES_RESISTOR / ((SUPPLY_VOLTAGE / (adcValue / (float)ADC_MAX)) - 1);

  // Calcul de la température
  float temperature = resistance / THERMISTOR_NOMINAL;
  temperature = log(temperature);
  temperature /= B_COEFFICIENT;
  temperature += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  temperature = 1.0 / temperature;
  temperature -= 273.15;

  Serial.print("Température actuelle : ");
  Serial.println(temperature);

  if (targetTemperature > 0) {
    if (temperature < targetTemperature - TEMPERATURE_HYSTERESIS) {
      digitalWrite(HEATER_PIN, HIGH);  // Allumer l'élément chauffant
    } else if (temperature >= targetTemperature + TEMPERATURE_HYSTERESIS) {
      digitalWrite(HEATER_PIN, LOW);   // Éteindre l'élément chauffant
    }
  } else {
    digitalWrite(HEATER_PIN, LOW);     // Désactiver si aucune cible
  }
}

// Fonction pour démarrer le moteur
void startMoteur() {
  if (!moteurActif) {
    moteurActif = true;
    digitalWrite(ENABLE_PIN, LOW);  // Activer le moteur
    Serial.println("Moteur démarré.");
  }

  // Faire tourner le moteur en continu tant qu'il est actif
  while (moteurActif) {
    digitalWrite(DIR_PIN, HIGH);    // Configurer la direction
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);        // Ajustez cette valeur pour la vitesse
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);

    // Vérifier si une commande "stop" arrive
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.equalsIgnoreCase("stop")) {
        stopMoteur();
      }
    }
  }
}

// Fonction pour arrêter le moteur
void stopMoteur() {
  moteurActif = false;
  digitalWrite(ENABLE_PIN, HIGH);  // Désactiver le moteur
  Serial.println("Moteur arrêté.");
}
