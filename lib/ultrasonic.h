#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// Initialisation du capteur
void ultrasonic_init(int trigPin, int echoPin);

// Initialisation du 2e capteur (optionnel)
void ultrasonic_init2(int trigPin, int echoPin);

// Lecture de la distance en cm
int ultrasonic_readDistance();

// Lecture distance du 2e capteur (cm). Renvoie -1 si pas configuré / pas de mesure.
int ultrasonic_readDistance2();

// Test si obstacle sous un seuil
bool ultrasonic_isObstacle(int distance, int threshold);

#endif