#include "../lib/safety.h"

Safety::Safety(Ultrasonic& us, EmergencyButton& btn, Life& life, Motors& motors)
: _us(us), _btn(btn), _life(life), _motors(motors) {}

bool Safety::check() {
  // 1) urgence
  if (_btn.urgenceActive()) {
    _motors.stop();
    Serial.println("🛑 ARRET D'URGENCE ACTIVE");
    return true;
  }

  // 2) fin de vie
  if (_life.check()) {
    _motors.stop();
    while (1) {} // comme ton code: stop définitif
  }

  // 3) obstacle (même logique que ton gererObstacle)
  float distance = _us.mesurerDistance();
  if (distance > 0 && distance <= _us.seuil()) {

    if (!_obstaclePresent) {
      Serial.print("🚨 OBSTACLE DETECTE A ");
      Serial.print(distance);
      Serial.println(" cm -> ARRET");
      _obstaclePresent = true;
    }

    _motors.stop();

    while (distance > 0 && distance <= _us.seuil()) {
      // urgence prioritaire pendant l’attente
      if (_btn.urgenceActive()) {
        _motors.stop();
        Serial.println("🛑 ARRET D'URGENCE ACTIVE");
        return true;
      }
      // fin de vie pendant l’attente
      if (_life.check()) {
        _motors.stop();
        while (1) {}
      }

      distance = _us.mesurerDistance();
      Serial.print("⏳ Obstacle toujours present : ");
      Serial.print(distance);
      Serial.println(" cm");
      delay(200);
    }

    Serial.println("✅ OBSTACLE DISPARU -> REPRISE");
    _motors.runForward(); // relance “dans le bon sens” comme toi
    _obstaclePresent = false;
  }

  return false;
}
