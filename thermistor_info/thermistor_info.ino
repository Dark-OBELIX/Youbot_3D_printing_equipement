#include <math.h>

#define THERMISTOR_PIN A0        // Pin pour le thermistor
#define SERIES_RESISTOR 4700     // Résistance série en ohms
#define THERMISTOR_NOMINAL 1000
#define TEMPERATURE_NOMINAL 25   // Température nominale en °C
#define B_COEFFICIENT 3950       // Coefficient B de la thermistance
#define ADC_MAX 1023             // Résolution ADC
#define SUPPLY_VOLTAGE 5.0       // Tension d'alimentation



float calculateTemperature(int adcValue) {
    // Calculer la résistance
    float resistance = SERIES_RESISTOR / ((SUPPLY_VOLTAGE / (adcValue / (float)ADC_MAX)) - 1);
    // Calculer la température
    float temperature = resistance / THERMISTOR_NOMINAL;
    temperature = log(temperature);
    temperature /= B_COEFFICIENT;
    temperature += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
    temperature = 1.0 / temperature;
    temperature -= 273.15; // Convertir de Kelvin à Celsius
    return temperature;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
    int adcValue = analogRead(THERMISTOR_PIN); // Lire la valeur de l'ADC
    float temperature = calculateTemperature(adcValue); // Calculer la température
    Serial.println(temperature);
    delay(1000); // Attendre 1 seconde avant la prochaine lecture
}
