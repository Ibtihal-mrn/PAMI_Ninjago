#include "../lib/motors.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = NULL;
Adafruit_DCMotor *motorR = NULL;

// inverser si nécessaire
bool invertLeft = false;
bool invertRight = true;

// ---------- PARAMETRES ----------
int baseSpeed = 140; // vitesse de base
float Kp = 0.5;      // gain correction trajectoire

int trimL = 0;
int trimR = -16;

static uint8_t applyInvert(bool invert, uint8_t dir)
{
  if (dir == FORWARD)
    return invert ? BACKWARD : FORWARD;
  if (dir == BACKWARD)
    return invert ? FORWARD : BACKWARD;
  return dir;
}

static void runLeft(uint8_t dir)
{
  motorL->run(applyInvert(invertLeft, dir));
}

static void runRight(uint8_t dir)
{
  motorR->run(applyInvert(invertRight, dir));
}

void motors_init(void)
{
  Wire.begin();
  AFMS.begin();

  motorL = AFMS.getMotor(2); // moteur gauche
  motorR = AFMS.getMotor(1); // moteur droit

  // moteurs initialisés à l'arrêt
  // motorL->setSpeed(0);
  // motorL->run(FORWARD);

  // motorR->setSpeed(0);
  // motorR->run(FORWARD);
  motors_stop();
}

void motors_applySpeeds(int speedL, int speedR)
{ // On va normalement plus l'utiliser
  runLeft(FORWARD);
  runRight(FORWARD);

  motorL->setSpeed(speedL);
  motorR->setSpeed(speedR);
}

void motors_stop()
{
  motorL->setSpeed(0);
  motorR->setSpeed(0);
  motorL->run(RELEASE); // Release permet de couper le pont H, arrêt propre
  motorR->run(RELEASE);
}

void motors_forward(int speedL, int speedR)
{
  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);

  motorL->setSpeed(speedL);
  motorR->setSpeed(speedR);

  runLeft(FORWARD);
  runRight(FORWARD);
}

void motors_rotateRight(int speed)
{
  speed = constrain(speed, 0, 255);
  motorL->setSpeed(speed);
  motorR->setSpeed(speed);
  runLeft(BACKWARD);
  runRight(FORWARD);
}

void motors_rotateLeft(int speed)
{
  speed = constrain(speed, 0, 255);
  motorL->setSpeed(speed);
  motorR->setSpeed(speed);
  runLeft(FORWARD);
  runRight(BACKWARD);
}

void motors_backward(int speedL, int speedR)
{
  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);

  motorL->setSpeed(speedL);
  motorR->setSpeed(speedR);

  runLeft(BACKWARD);
  runRight(BACKWARD);
}