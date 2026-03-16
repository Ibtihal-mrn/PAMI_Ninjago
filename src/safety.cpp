#include "../lib/safety.h"

Safety::Safety(Ultrasonic &us1, Ultrasonic &us2, EmergencyButton &btn, Life &life, Motors &motors)
    : _us1(us1), _us2(us2), _btn(btn), _life(life), _motors(motors) {}

bool Safety::check()
{
  // 1) urgence
  if (_btn.urgenceActive())
  {
    _motors.stop();
    Serial.println("🛑 ARRET D'URGENCE ACTIVE");
    return true;
  }

  // 2) fin de vie
  if (_life.check())
  {
    _motors.stop();
    while (1)
    {
    } // comme ton code: stop définitif
  }

  // 3) obstacle (même logique que ton gererObstacle)
  float distance1 = _us1.mesurerDistance();
  float distance2 = _us2.mesurerDistance();
  bool obstacle1 = (distance1 > 0 && distance1 <= _us1.seuil());
  bool obstacle2 = (distance2 > 0 && distance2 <= _us2.seuil());

  if (obstacle1 || obstacle2)
  {
    float distance = obstacle1 && (!obstacle2 || distance1 <= distance2) ? distance1 : distance2;
    const char *capteur = obstacle1 && (!obstacle2 || distance1 <= distance2) ? "US1" : "US2";

    if (!_obstaclePresent)
    {
      Serial.print("🚨 OBSTACLE DETECTE A ");
      Serial.print(distance);
      Serial.print(" cm (capteur ");
      Serial.print(capteur);
      Serial.println(") -> ARRET");
      _obstaclePresent = true;
    }

    _motors.stop();

    while (obstacle1 || obstacle2)
    {
      // urgence prioritaire pendant l’attente
      // if (_btn.urgenceActive()) {
      //   _motors.stop();
      //   Serial.println("🛑 ARRET D'URGENCE ACTIVE");
      //   return true;
      // }
      // fin de vie pendant l’attente
      if (_life.check())
      {
        _motors.stop();
        while (1)
        {
        }
      }

      distance1 = _us1.mesurerDistance();
      distance2 = _us2.mesurerDistance();
      obstacle1 = (distance1 > 0 && distance1 <= _us1.seuil());
      obstacle2 = (distance2 > 0 && distance2 <= _us2.seuil());

      distance = obstacle1 && (!obstacle2 || distance1 <= distance2) ? distance1 : distance2;
      capteur = obstacle1 && (!obstacle2 || distance1 <= distance2) ? "US1" : "US2";

      Serial.print("⏳ Obstacle toujours present : ");
      Serial.print(distance);
      if (obstacle1 || obstacle2)
      {
        Serial.print(" cm (capteur ");
        Serial.print(capteur);
        Serial.println(")");
      }
      else
      {
        Serial.println(" cm");
      }
      delay(200);
    }

    Serial.println("✅ OBSTACLE DISPARU -> REPRISE");
    _motors.runForward(); // relance “dans le bon sens” comme toi
    _obstaclePresent = false;
  }

  return false;
}
