#include "../lib/robot.h"
#include "../lib/RobotGeometry.h"   // ✅ AJOUT : conversion cm/deg -> ticks

// paramètres PID = tes #define (on les laisse dans main.cpp ou ici)
#ifndef BASE_SPEED
#define BASE_SPEED 70
#endif
#ifndef KP
#define KP 1.0
#endif
#ifndef KI
#define KI 0.03
#endif
#ifndef MAX_CORR
#define MAX_CORR 12
#endif

// ✅ AJOUT : instance géométrie (valeurs que tu as données)
// Rayon 2.94cm => diamètre 5.88cm
// Entrâxe 9.38cm
// ticks/tour mesurés ~G=43, D=41
static RobotGeometry GEOM(5.88f, 9.38f, 43.0f, 41.0f);

// (optionnel) calibration fine : à ajuster après tests
// tu peux aussi exposer des setters dans Robot si tu veux les régler depuis main
// GEOM.setDriveCalibration(1.0f);
// GEOM.setTurnCalibration(1.0f);

Robot::Robot(Motors& motors, Encoders& enc, Safety& safety)
: _motors(motors), _enc(enc), _safety(safety) {}

// ✅ AJOUT : avancer en cm (sans changer la logique PID/ticks)
void Robot::avancer_cm(float distance_cm) {
  long ticks = GEOM.cmToTicks(distance_cm);
  avancer_ticks(ticks);
}

// ✅ AJOUT : tourner à gauche en degrés (sans changer la logique ticks)
void Robot::tourner_gauche_deg(float deg) {
  long ticks = GEOM.degToTicks(deg);
  tourner_gauche_ticks(ticks);
}

void Robot::avancer_ticks(long ticks_cible) {
  int vitesseG = 0;
  int vitesseD = 0;

  _enc.reset();
  _lastTicksG = 0;
  _lastTicksD = 0;
  _erreurI = 0;
  _lastPID = millis();

  _motors.runForward();

  while (((_enc.ticksG() + _enc.ticksD()) / 2) < ticks_cible) {

    // ✅ EXACTEMENT ton intention : dans le while on appelle Safety
    if (_safety.check()) return;

    if (millis() - _lastPID >= 50) {
      long dG = _enc.ticksG() - _lastTicksG;
      long dD = _enc.ticksD() - _lastTicksD;

      long erreur = dD - dG;

      _erreurI += erreur;
      _erreurI = constrain(_erreurI, -100, 100);

      int correction = (int)(KP * erreur + KI * _erreurI);
      correction = constrain(correction, -MAX_CORR, MAX_CORR);

      vitesseG = constrain(BASE_SPEED - correction, 40, 120);
      vitesseD = constrain(BASE_SPEED + correction, 40, 120);

      _motors.setSpeed(vitesseD, vitesseG);

      _lastTicksG = _enc.ticksG();
      _lastTicksD = _enc.ticksD();
      _lastPID = millis();
    }

    if (millis() - _lastDebug > 100) {
      Serial.print(_enc.ticksG()); Serial.print("\t");
      Serial.print(_enc.ticksD()); Serial.print("\t");
      Serial.print(_enc.ticksD() - _enc.ticksG());
      Serial.print("\tVG="); Serial.print(vitesseG);
      Serial.print("\tVD="); Serial.println(vitesseD);
      _lastDebug = millis();
    }
  }

  _motors.stop();
  delay(300);
}

void Robot::tourner_gauche_ticks(long ticks_cible) {
  _enc.reset();

  _motors.runTurnLeft();
  _motors.setSpeed(65, 65);

  while (((_enc.ticksG() + _enc.ticksD()) / 2) < ticks_cible) {
    if (_safety.check()) return; // ✅ appelé dans le while aussi
  }

  _motors.stop();
  delay(1000);
}
