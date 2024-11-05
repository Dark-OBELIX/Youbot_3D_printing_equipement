#include <math.h>

#define THERMISTOR_PIN A13         // Pin où le thermistor est connecté
#define HEATER_PIN 10              // Pin de l'élément chauffant (D10 sur RAMPS 1.4)
#define SERIES_RESISTOR 4700       // Résistance série 4.7kΩ

#define THERMISTOR_NOMINAL 1000      
#define TEMPERATURE_NOMINAL 25     // Température nominale 25°C
#define B_COEFFICIENT 3950         // Coefficient B
#define ADC_MAX 1023               // Résolution ADC
#define SUPPLY_VOLTAGE 5           // Tension d'alimentation

float targetTemperature = -1;      // Initialisation sans cible de température
#define TEMPERATURE_HYSTERESIS 2   // Hystérésis pour éviter de toggler fréquemment

void setup() {
  Serial.begin(9600);
  pinMode(HEATER_PIN, OUTPUT);     // Définir la broche de l'élément chauffant comme sortie
}

void loop() {
  // Lire les données envoyées par le port série (depuis le script Python)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    targetTemperature = input.toFloat();  // Met à jour la température cible
    Serial.println(targetTemperature);
  }

  int adcValue = analogRead(THERMISTOR_PIN);  // Lire l'ADC brut

  // Convertir la lecture en résistance
  float resistance = SERIES_RESISTOR / ((SUPPLY_VOLTAGE / (adcValue / (float)ADC_MAX)) - 1);

  // Calculer la température
  float temperature;
  temperature = resistance / THERMISTOR_NOMINAL;
  temperature = log(temperature);
  temperature /= B_COEFFICIENT;
  temperature += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  temperature = 1.0 / temperature;
  temperature -= 273.15;

  // Afficher la température actuelle
  Serial.println(temperature);

  // Si une température cible a été définie, contrôler l'élément chauffant
  if (targetTemperature > 0) {
    if (temperature < targetTemperature - TEMPERATURE_HYSTERESIS) {
      digitalWrite(HEATER_PIN, HIGH);  // Allumer l'élément chauffant
    } else if (temperature >= targetTemperature + TEMPERATURE_HYSTERESIS) {
      digitalWrite(HEATER_PIN, LOW);   // Éteindre l'élément chauffant
    }
  } else {
    digitalWrite(HEATER_PIN, LOW);   // Éteindre l'élément chauffant si aucune cible n'est définie
  }

  delay(1000);  // Attendre une seconde avant la prochaine lecture
}
