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
#include "../lib/startSwitch.h"
#include "../lib/teamSwitch.h"
#include "../lib/fsm.h"

// ===== tes constantes =====
const unsigned long DUREE_VIE_MS = 90000;
const int PIN_SERVO = 9;
const int SERVO_POS_INIT = 0;
const int SERVO_POS_FIN = 90;
const int TRIG_PIN_1 = 6;
const int ECHO_PIN_1 = 7;
const int TRIG_PIN_2 = 10;
const int ECHO_PIN_2 = 11;
const float DISTANCE_SEUIL = 10.0;
const int PIN_START = 4;
const int PIN_TEAM = 5;
const int PIN_URGENCE = 8;

// PID : les constantes réelles sont dans robot.cpp (les #define ici n'y sont pas visibles)

// ===== objets =====
Adafruit_MotorShield AFMS;
Motors motors(AFMS, 1, 2);
Encoders encoders(2, 3);
Ultrasonic us1(TRIG_PIN_1, ECHO_PIN_1, DISTANCE_SEUIL);
Ultrasonic us2(TRIG_PIN_2, ECHO_PIN_2, DISTANCE_SEUIL);
EmergencyButton btn(PIN_URGENCE);
Servo servoFin;
Life life(DUREE_VIE_MS, servoFin, SERVO_POS_INIT, SERVO_POS_FIN);
Safety safety(us1, us2, btn, life, motors);
Robot robot(motors, encoders, safety);
StartSwitch startSwitch(PIN_START);
TeamSwitch teamSwitch(PIN_TEAM);
Fsm fsm(robot);
Team team = Team::A; // valeur par défaut, sera mise à jour dans setup()

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
  }

  Serial.println("=== PAMI - REGULATION VITESSE (PI) + ARRET URGENCE ===");

  // ===== TIRETTE =====
  startSwitch.begin();
  startSwitch.waitForStart();

  // ===== TEAM SWITCH =====
  teamSwitch.begin();
  team = teamSwitch.readTeam();
  if (team == Team::A)
  {
    Serial.println("Equipe A sélectionnée");
  }
  else
  {
    Serial.println("Equipe B sélectionnée");
  }

  if (!motors.begin())
  {
    Serial.println("❌ ERREUR SHIELD");
    while (1)
    {
    }
  }

  encoders.begin();
  us1.begin();
  us2.begin();
  btn.begin();

  servoFin.attach(PIN_SERVO);
  servoFin.write(SERVO_POS_INIT);
  life.begin();

  fsm.begin(team);

  delay(3000);
}

void loop()
{
  fsm.update();

  if (fsm.isFinished())
  {
    while (1)
    {
    }
  }
}
