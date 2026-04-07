#include "../lib/motors.h"

static bool g_motorShieldOk = false;

static void i2c_scan_bus(Print &out)
{
  out.println(F("I2C scan..."));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0)
    {
      out.print(F("  - 0x"));
      if (addr < 16)
        out.print('0');
      out.println(addr, HEX);
      found++;
    }
  }
  if (found == 0)
    out.println(F("  (no device found)"));
}

// Tente de libérer un bus I2C bloqué (SDA maintenue à 0 par un périphérique)
static void i2c_bus_recover()
{
#if defined(SDA) && defined(SCL)
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);

  // Clock out up to 9 pulses while SDA is low
  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; i++)
  {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }

  // Generate a STOP condition
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
#endif
}

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
  i2c_bus_recover();
  Wire.begin();
  Wire.setClock(100000);

  g_motorShieldOk = AFMS.begin();
  if (!g_motorShieldOk)
  {
    Serial.println(F("[ERR] Motor Shield V2 non detecte sur I2C (adresse attendue 0x60)."));
    Serial.println(F("      Causes frequentes: SDA/SCL inverses, peripherique I2C cable sur mauvais pins, capteur HS qui bloque SDA/SCL, ou GND manquant."));
    i2c_scan_bus(Serial);
    // On stoppe ici pour eviter une sequence qui 'tourne' sans moteurs.
    while (1)
      delay(1000);
  }

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