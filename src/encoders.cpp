#include "../lib/encoders.h"

volatile long ticksL = 0;
volatile long ticksR = 0;

// Filtre anti-glitch logiciel: ignore les fronts trop rapproches.
// Pour encodeur simple, garder une valeur faible pour ne pas rater des impulsions.
// 1200us reste compatible avec une vitesse de roue elevee tout en filtrant
// les parasites de rebranchement de cable.
static const unsigned long ENC_MIN_PULSE_US = 1200;
volatile unsigned long lastEdgeUsL = 0;
volatile unsigned long lastEdgeUsR = 0;

long prevL = 0;
long prevR = 0;

// ---------- INTERRUPTIONS ----------
void ISR_left(void)
{
  unsigned long now = micros();
  if ((unsigned long)(now - lastEdgeUsL) >= ENC_MIN_PULSE_US)
  {
    ticksL++;
    lastEdgeUsL = now;
  }
}

void ISR_right(void)
{
  unsigned long now = micros();
  if ((unsigned long)(now - lastEdgeUsR) >= ENC_MIN_PULSE_US)
  {
    ticksR++;
    lastEdgeUsR = now;
  }
}

void encoders_init(void)
{
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);

  ticksL = 0;
  ticksR = 0;
  lastEdgeUsL = micros();
  lastEdgeUsR = micros();

  // Pour un encodeur simple en INPUT_PULLUP, le front FALLING est souvent
  // le plus propre (impulsion active vers GND).
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), ISR_left, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), ISR_right, FALLING);
}

void encoders_read(long *left, long *right)
{
  noInterrupts();
  *left = ticksL;
  *right = ticksR;
  interrupts();
}

void encoders_computeDelta(long left, long right, long *dL, long *dR)
{
  *dL = left - prevL;
  *dR = right - prevR;

  prevL = left;
  prevR = right;
}