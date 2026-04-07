#include "../lib/robot.h"
#include "../lib/encoders.h"
#include "../lib/motors.h"
#include "../lib/control.h"
#include "../lib/kinematics.h"
#include "../lib/safety.h"
#include "../lib/ultrasonic.h"
#include "../lib/emergencyButton.h"

#define DRIVE_PI_DEBUG 1

// ------------------------------------------------------------
// Pause/reprise de la séquence via le bouton d'arrêt d'urgence
// - Toggle sur front montant de emergencyButton_isPressed()
// - En pause: moteurs stop + attente (polling + debounce)
// ------------------------------------------------------------
static bool g_pauseEnabled = true;
static bool g_isPaused = false;
static bool g_prevBtn = false;
static unsigned long g_lastToggleMs = 0;

static void pause_updateToggle()
{
  if (!g_pauseEnabled)
    return;

  bool btn = emergencyButton_isPressed();
  unsigned long now = millis();

  // Debounce simple + edge detect
  const unsigned long DEBOUNCE_MS = 80;
  if (btn && !g_prevBtn && (unsigned long)(now - g_lastToggleMs) >= DEBOUNCE_MS)
  {
    g_isPaused = !g_isPaused;
    g_lastToggleMs = now;
  }

  g_prevBtn = btn;
}

// Retourne la durée passée en pause (ms). 0 => pas de pause.
static unsigned long pause_waitIfNeeded()
{
  pause_updateToggle();
  if (!g_isPaused)
    return 0;

  motors_stop();
  unsigned long pauseStart = millis();

  while (true)
  {
    pause_updateToggle();
    if (!g_isPaused)
      break;
    delay(5);
  }

  motors_stop();
  return (unsigned long)(millis() - pauseStart);
}

void robot_init()
{
  motors_init();
  encoders_init();
  ultrasonic_init(13, 10); // trig, echo
  safety_init(40, 50);     // 40cm seuil, sonar toutes les 50ms
}

void robot_step()
{ // On va aussi plus l'utiliser normalement
  long left, right;
  encoders_read(&left, &right);

  long dL, dR;
  encoders_computeDelta(left, right, &dL, &dR);

  int speedL, speedR;
  control_computeSpeeds(dL, dR, speedL, speedR);

  motors_applySpeeds(speedL, speedR);
}

void robot_stop()
{
  motors_stop();
}

void robot_rotate(float angle_deg, int speed)
{
  //   long targetTicks = ticks_for_rotation_deg(angle_deg);

  //   long startL, startR;
  //   encoders_read(&startL, &startR);

  //   // choix du sens
  //   if (angle_deg > 0) {
  //       motors_rotateRight(speed);   // droite = angle positif
  //   } else {
  //       motors_rotateLeft(speed);    // gauche = angle négatif
  //   }

  //   while (true) {
  //       long curL, curR;
  //       encoders_read(&curL, &curR);

  //       long dL = labs(curL - startL);
  //       long dR = labs(curR - startR);

  //       if ((dL + dR) / 2 >= labs(targetTicks)) {
  //       break;
  //       }
  //   }

  // motors_stop();
  const uint16_t DT_MS = 10;
  unsigned long tPrev = micros();

  long targetTicks = ticks_for_rotation_deg(angle_deg);

  long startL, startR;
  encoders_read(&startL, &startR);

  if (angle_deg > 0)
    motors_rotateRight(speed);
  else
    motors_rotateLeft(speed);

  while (true)
  {
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL)
    {
      unsigned long pausedMs = pause_waitIfNeeded();
      if (pausedMs > 0)
        tPrev = micros();
      continue;
    }
    tPrev += (unsigned long)DT_MS * 1000UL;

    {
      unsigned long pausedMs = pause_waitIfNeeded();
      if (pausedMs > 0)
      {
        tPrev = micros();
        continue;
      }
    }

    safety_update();
    if (safety_isTriggered())
    {
      motors_stop();
      return; // arrêt immédiat
    }

    long curL, curR;
    encoders_read(&curL, &curR);

    long dL = labs(curL - startL);
    long dR = labs(curR - startR);

    if ((dL + dR) / 2 >= labs(targetTicks))
      break;
  }

  motors_stop();
}


void robot_move_distance(float dist_mm, int pwmBaseTarget)
{
  // // on garde ton système : control_computeSpeeds utilise baseSpeed
  // int oldBase = baseSpeed;
  // baseSpeed = speed;

  // long target = ticks_for_distance_mm(abs(dist_mm));

  // long startL, startR;
  // encoders_read(&startL, &startR);

  // // IMPORTANT : repartir propre pour les deltas de vitesse
  // prevL = startL;
  // prevR = startR;

  // while (true) {
  //   long curL, curR;
  //   encoders_read(&curL, &curR);

  //   // arrêt basé sur la distance (position totale)
  //   long distTicksL = labs(curL - startL);
  //   long distTicksR = labs(curR - startR);
  //   if ((distTicksL + distTicksR) / 2 >= target) break;

  //   // correction basée sur la vitesse instantanée (comme avant)
  //   long dL, dR;
  //   encoders_computeDelta(curL, curR, &dL, &dR);

  //   int speedL, speedR;
  //   control_computeSpeeds(dL, dR, speedL, speedR);

  //   motors_applySpeeds(speedL, speedR);

  //   delay(40);
  // }

  // motors_stop();
  // baseSpeed = oldBase;

  const uint16_t DT_MS = 10;
  const float dt = DT_MS / 1000.0f;

  long target = ticks_for_distance_mm(fabs(dist_mm));
  int dir = (dist_mm >= 0.0f) ? 1 : -1;

#if DRIVE_PI_DEBUG
  Serial.print("[move] dist_mm=");
  Serial.print(dist_mm);
  Serial.print(" pwm=");
  Serial.print(pwmBaseTarget);
  Serial.print(" mm_per_tick=");
  Serial.print(mm_per_tick(), 4);
  Serial.print(" targetTicks=");
  Serial.println(target);
#endif

  // Deceleration douce en fin de course pour limiter le depassement.
  // (En "RELEASE", le robot peut encore rouler un peu -> on ralentit avant.)
  const long BRAKE_TICKS_MIN = 10;
  const long BRAKE_TICKS_MAX = 60;
  long brakeTicks = (long)lround((float)target * 0.35f);
  brakeTicks = constrain(brakeTicks, BRAKE_TICKS_MIN, BRAKE_TICKS_MAX);
  const int PWM_NEAR = 45; // pwm vise dans la zone de freinage (a ajuster)

  // Important pour la répétabilité: s'assurer que le robot est immobile
  // avant de figer les ticks de départ (sinon 2 essais peuvent démarrer différemment).
  motors_stop();
  delay(60);

  long startL, startR;
  encoders_read(&startL, &startR);

  // reset deltas encodeurs pour la vitesse
  prevL = startL;
  prevR = startR;

  DrivePIState st;
  control_reset(st);

  unsigned long tPrev = micros();
  unsigned long startMs = millis();
  unsigned long dbgLastMs = 0;

  // Garde-fous: évite de rouler indéfiniment si un encodeur décroche
  const unsigned long MOVE_TIMEOUT_MS = 12000;
  unsigned long oneSideDeadSinceMs = 0;
  const long ONE_SIDE_DEAD_TICKS = 80;

  // Phase d'arrêt/settle: attend que les encodeurs ne bougent plus
  // pour que l'étape suivante de la séquence parte toujours d'un état stable.
  const uint16_t SETTLE_MAX_MS = 260;
  const uint8_t SETTLE_GOOD_COUNT = 4;
  const long SETTLE_D_TICKS_TOL = 1;
  bool settling = false;
  unsigned long settleStartMs = 0;
  uint8_t settleGood = 0;

  while (true)
  {
    unsigned long pausedMs = pause_waitIfNeeded();
    if (pausedMs > 0)
    {
      startMs += pausedMs;
      if (settling)
        settleStartMs += pausedMs;

      // Repart proprement (évite dt énorme / rattrapage / intégrale PI)
      long curL, curR;
      encoders_read(&curL, &curR);
      prevL = curL;
      prevR = curR;
      control_reset(st);
      tPrev = micros();
      continue;
    }

    // période fixe
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL)
      continue;
    tPrev += (unsigned long)DT_MS * 1000UL;

    // safety_update();
    // if (safety_isTriggered()) {
    //   motors_stop();

    //   while(safety_isTriggered()){
    //     safety_update();
    //     safety_clearIfSafe();
    //     delay(20);
    //   }
    // }

    long curL, curR;
    encoders_read(&curL, &curR);

    long distTicksL = labs(curL - startL);
    long distTicksR = labs(curR - startR);
    long avgTicks = (distTicksL + distTicksR) / 2;

    // deltas (vitesse)
    long dL, dR;
    encoders_computeDelta(curL, curR, &dL, &dR);

    // Si on est en phase settle, on attend que le robot s'immobilise.
    if (settling)
    {
      if (labs(dL) <= SETTLE_D_TICKS_TOL && labs(dR) <= SETTLE_D_TICKS_TOL)
        settleGood++;
      else
        settleGood = 0;

      if (settleGood >= SETTLE_GOOD_COUNT)
        break;
      if (millis() - settleStartMs >= SETTLE_MAX_MS)
        break;

#if DRIVE_PI_DEBUG
      unsigned long nowMs = millis();
      if (nowMs - dbgLastMs >= 100)
      {
        dbgLastMs = nowMs;
        Serial.print("[settle] dL=");
        Serial.print(dL);
        Serial.print(" dR=");
        Serial.println(dR);
      }
#endif
      continue;
    }

    // Si une roue avance beaucoup et l'autre reste quasi a zero, on stoppe.
    long maxDistTicks = max(distTicksL, distTicksR);
    long minDistTicks = min(distTicksL, distTicksR);
    if (maxDistTicks >= ONE_SIDE_DEAD_TICKS && minDistTicks <= 2)
    {
      if (oneSideDeadSinceMs == 0)
        oneSideDeadSinceMs = millis();
      if (millis() - oneSideDeadSinceMs > 250)
      {
        Serial.print("[ERR] Encodeur asymetrique: arret securite (distTicksL=");
        Serial.print(distTicksL);
        Serial.print(" distTicksR=");
        Serial.print(distTicksR);
        Serial.println(")");
        break;
      }
    }
    else
    {
      oneSideDeadSinceMs = 0;
    }

    if (millis() - startMs > MOVE_TIMEOUT_MS)
    {
      Serial.print("[ERR] Timeout deplacement: arret securite (avgTicks=");
      Serial.print(avgTicks);
      Serial.print(" target=");
      Serial.print(target);
      Serial.println(")");
      break;
    }

    if (avgTicks >= target)
    {
#if DRIVE_PI_DEBUG
      Serial.print("[move] target reached -> settle (avgTicks=");
      Serial.print(avgTicks);
      Serial.print(" target=");
      Serial.print(target);
      Serial.println(")");
#endif
      motors_stop();
      settling = true;
      settleStartMs = millis();
      settleGood = 0;
      continue;
    }

    // Profil simple: on baisse la consigne PWM quand on approche la cible
    long remaining = target - avgTicks;
    int pwmCmd = pwmBaseTarget;
    if (remaining <= brakeTicks)
    {
      // Interpolation lineaire: remaining=brakeTicks => pwmBaseTarget, remaining=0 => PWM_NEAR
      float ratio = (brakeTicks > 0) ? ((float)remaining / (float)brakeTicks) : 0.0f;
      ratio = constrain(ratio, 0.0f, 1.0f);
      pwmCmd = PWM_NEAR + (int)lround((float)(pwmBaseTarget - PWM_NEAR) * ratio);
      pwmCmd = constrain(pwmCmd, 0, pwmBaseTarget);
    }

    // erreur de cap cumulée (position)
    long headingErr = (curL - startL) - (curR - startR);

    int pwmL, pwmR;
    control_driveStraight_PI(st, headingErr, dL, dR, pwmCmd, dt, pwmL, pwmR);

    if (dir > 0)
      motors_forward(pwmL, pwmR);
    else
      motors_backward(pwmL, pwmR);

#if DRIVE_PI_DEBUG
    // Log periodique pour tuning PID sans saturer le port serie.
    unsigned long nowMs = millis();
    if (nowMs - dbgLastMs >= 100)
    {
      dbgLastMs = nowMs;
      Serial.print("ticksL=");
      Serial.print(curL - startL);
      Serial.print(" ticksR=");
      Serial.print(curR - startR);
      Serial.print(" dL=");
      Serial.print(dL);
      Serial.print(" dR=");
      Serial.print(dR);
      Serial.print(" pwmCmd=");
      Serial.print(pwmCmd);
      Serial.print(" err=");
      Serial.println(headingErr);
    }
#endif
  }

  motors_stop();

#if DRIVE_PI_DEBUG
  long endL, endR;
  encoders_read(&endL, &endR);
  long distTicksL = labs(endL - startL);
  long distTicksR = labs(endR - startR);
  long avgTicks = (distTicksL + distTicksR) / 2;
  float distEst = avgTicks * mm_per_tick();

  Serial.print("[move] done ticksL=");
  Serial.print(distTicksL);
  Serial.print(" ticksR=");
  Serial.print(distTicksR);
  Serial.print(" avg=");
  Serial.print(avgTicks);
  Serial.print(" distEst_mm=");
  Serial.println(distEst, 1);
#endif
}


void robot_pauseable_delay(uint32_t ms)
{
  unsigned long start = millis();
  while ((unsigned long)(millis() - start) < ms)
  {
    unsigned long pausedMs = pause_waitIfNeeded();
    if (pausedMs > 0)
    {
      start += pausedMs; // le temps en pause ne compte pas
      continue;
    }
    delay(5);
  }
}