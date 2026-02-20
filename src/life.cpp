#include "../lib/life.h"

Life::Life(unsigned long dureeMs, Servo& servo, int posInit, int posFin)
: _duree(dureeMs), _servo(servo), _posInit(posInit), _posFin(posFin) {}

void Life::begin() {
  _servo.write(_posInit);
  _startTime = millis();
  _declenchee = false;
  _lastPrint = 0;
}

bool Life::check() {
  unsigned long tempsEcoule = millis() - _startTime;

  if (millis() - _lastPrint >= 200) {
    Serial.print("Temps écoulé : ");
    Serial.print(tempsEcoule);
    Serial.println(" ms");
    _lastPrint = millis();
  }

  if (!_declenchee && tempsEcoule >= _duree) {
    Serial.println("===== SEUIL ATTEINT =====");
    _declenchee = true;
    _servo.write(_posFin);
    Serial.println("⏳ FIN DE VIE: moteurs coupes + servo actionne");
    return true;
  }

  return false;
}
