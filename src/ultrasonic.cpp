#include "../lib/ultrasonic.h"

static int PIN_TRIG_1;
static int PIN_ECHO_1;
static int PIN_TRIG_2;
static int PIN_ECHO_2;
static bool g_hasSecond = false;

static int measureCm(int trigPin, int echoPin)
{
    // Sécurité: s'assurer que l'ECHO est bas avant de déclencher.
    // Si l'entrée est déjà HIGH (bruit/couplage/câblage), pulseIn peut mesurer un faux pulse très court.
    unsigned long t0 = micros();
    while (digitalRead(echoPin) == HIGH)
    {
        if ((unsigned long)(micros() - t0) > 2000UL)
            break;
    }

    // Envoi de l'impulsion
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Lecture du retour
    // 20ms = ~3.4m max, conforme à ton sketch
    long duree = pulseIn(echoPin, HIGH, 20000UL);

    if (duree == 0)
    {
        return -1; // Pas de mesure valide
    }

    return (int)(duree / 58UL);
}

void ultrasonic_init(int trigPin, int echoPin) {
    PIN_TRIG_1 = trigPin;
    PIN_ECHO_1 = echoPin;

    pinMode(PIN_TRIG_1, OUTPUT);
    pinMode(PIN_ECHO_1, INPUT);
}

void ultrasonic_init2(int trigPin, int echoPin)
{
    PIN_TRIG_2 = trigPin;
    PIN_ECHO_2 = echoPin;
    g_hasSecond = true;

    pinMode(PIN_TRIG_2, OUTPUT);
    pinMode(PIN_ECHO_2, INPUT);
}

int ultrasonic_readDistance() {
    return measureCm(PIN_TRIG_1, PIN_ECHO_1);
}

int ultrasonic_readDistance2()
{
    if (!g_hasSecond)
        return -1;
    return measureCm(PIN_TRIG_2, PIN_ECHO_2);
}

bool ultrasonic_isObstacle(int distance, int threshold) {
    return (distance > 0 && distance <= threshold);
}