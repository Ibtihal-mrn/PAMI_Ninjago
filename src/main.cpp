#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

#include "../lib/motors.h"
#include "../lib/encoders.h"
#include "../lib/ultrasonic.h"
#include "../lib/emergencyButton.h"
#include "../lib/life.h"
#include "../lib/safety.h"
#include "../lib/robot.h"

// ===== tes constantes =====
const unsigned long DUREE_VIE_MS = 100000;

const int PIN_SERVO = 9;
const int SERVO_POS_INIT = 0;
const int SERVO_POS_FIN  = 90;

const int TRIG_PIN = 6;
const int ECHO_PIN = 7;
const float DISTANCE_SEUIL = 10.0;

const int PIN_URGENCE = 8;

// mouvements
#define TICKS_5CM     30
#define TICKS_15CM    54
#define TICKS_90DEG   16
#define TICKS_180DEG  34

// PID (gardé)
#define BASE_SPEED  70
#define KP          1.0
#define KI          0.03
#define MAX_CORR    12

// ===== objets =====
Adafruit_MotorShield AFMS;

Motors motors(AFMS, 1, 2);
Encoders encoders(2, 3);
Ultrasonic us(TRIG_PIN, ECHO_PIN, DISTANCE_SEUIL);
EmergencyButton btn(PIN_URGENCE);

Servo servoFin;
Life life(DUREE_VIE_MS, servoFin, SERVO_POS_INIT, SERVO_POS_FIN);

Safety safety(us, btn, life, motors);
Robot robot(motors, encoders, safety);

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  Serial.println("=== PAMI - REGULATION VITESSE (PI) + ARRET URGENCE ===");

  if (!motors.begin()) {
    Serial.println("❌ ERREUR SHIELD");
    while (1) {}
  }

  encoders.begin();
  us.begin();
  btn.begin();

  servoFin.attach(PIN_SERVO);
  servoFin.write(SERVO_POS_INIT);
  life.begin();

  delay(3000);
}

void loop() {
  // Serial.println("➡️ Avance 5 cm");
  // robot.avancer_ticks(TICKS_5CM);

  // Serial.println("↪️ Rotation 90° gauche");
  // robot.tourner_gauche_ticks(TICKS_90DEG);

  // Serial.println("➡️ Avance 15 cm");
  // robot.avancer_ticks(TICKS_15CM);

  // Serial.println("🔁 Rotation 180° (retour)");
  // robot.tourner_gauche_ticks(TICKS_180DEG);

  // Serial.println("➡️ Avance 5 cm");
  // robot.avancer_ticks(50);

  // Serial.println("↪️ Rotation 90° gauche");
  // robot.tourner_gauche_ticks(TICKS_90DEG);

  // Serial.println("➡️ Avance 15 cm");
  // robot.avancer_ticks(130);

  // Serial.println("↪️ Rotation 90° gauche");
  // robot.tourner_gauche_ticks(TICKS_90DEG);

  // Serial.println("➡️ Avance 5 cm");
  // robot.avancer_ticks(TICKS_5CM + 10);

  // Serial.println("↪️ Rotation 90° gauche");
  // robot.tourner_gauche_ticks(TICKS_90DEG + 2);

  // Serial.println("➡️ Avance 15 cm");
  // robot.avancer_ticks(170);

  // Serial.println("✅ FIN SEQUENCE");
  // while (1) {}

  robot.avancer_cm(50);
  delay(1000);
  robot.tourner_gauche_deg(90);
  delay(1000);
}


