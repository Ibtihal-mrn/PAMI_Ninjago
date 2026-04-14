#include "../lib/robot.h"
#include "../lib/encoders.h"
#include "../lib/motors.h"
#include "../lib/control.h"
#include "../lib/kinematics.h"
#include "../lib/imu.h"
#include "../lib/safety.h"
#include "../lib/ultrasonic.h"
#include "../lib/emergencyButton.h"

#define DRIVE_PI_DEBUG 1

static bool robot_handleSafetyStop()
{
  safety_update();
  if (!safety_isTriggered())
    return false;

  // 1) Urgence: priorité, on ABORT l'action en cours.
  if (safety_isEmergencyActive())
  {
    motors_stop();
    Serial.println("[EMERGENCY] STOP");

    while (safety_isEmergencyActive())
    {
      safety_update();
      safety_clearIfSafe();
      delay(20);
    }
    return true;
  }

  // 2) Obstacle ultrason: PAUSE + prints, puis reprise automatique (comme ton sketch).
  if (safety_isObstacleActive())
  {
    static bool obstaclePresent = false;
    motors_stop();

    if (!obstaclePresent)
    {
      obstaclePresent = true;
      Serial.println("\xF0\x9F\x9A\xA8 OBSTACLE DETECTE (2 capteurs) -> ARRET");
    }

    const int threshold = safety_getObstacleThresholdCm();
    while (true)
    {
      // Pendant la pause obstacle: on ne fait qu'une mise à jour sécurité toutes les ~200ms.
      // Comme safety_update() est throttlé, ça évite de repinger trop vite les ultrasons.
      safety_update();

      if (safety_isEmergencyActive())
      {
        motors_stop();
        Serial.println("[EMERGENCY] STOP");
        while (safety_isEmergencyActive())
        {
          safety_update();
          delay(20);
        }
        return true;
      }

      int d1, d2;
      safety_getLastDistancesCm(d1, d2);

      bool obstacleEncore = safety_isObstacleActive();
      (void)threshold; // threshold reste la ref, mais l'état obstacle est filtré côté safety.

      // prints identiques au sketch
      Serial.print("US1:");
      Serial.print(d1);
      Serial.print(" US2:");
      Serial.println(d2);

      if (!obstacleEncore)
        break;

      delay(200);
    }

    Serial.println("\xE2\x9C\x85 OBSTACLE DISPARU -> REPRISE");
    obstaclePresent = false;
    safety_clearIfSafe();
    return false; // reprise
  }

  // Fallback
  motors_stop();
  Serial.println("[SAFETY] STOP");
  while (safety_isTriggered())
  {
    safety_update();
    safety_clearIfSafe();
    delay(20);
  }
  return true;
}

bool robot_pauseable_delay(uint16_t ms)
{
  unsigned long start = millis();
  while ((unsigned long)(millis() - start) < (unsigned long)ms)
  {
    if (robot_handleSafetyStop())
      return false;
    delay(5);
  }
  return true;
}

void robot_init()
{
  motors_init();
  encoders_init();
  ultrasonic_init(6, 7);   // US1: trig, echo
  ultrasonic_init2(10, 11); // US2: trig, echo
  safety_init(10, 50);     // comme ton sketch: obstacle <=10cm, sonar toutes les 50ms

  // IMU
  if (!imu_init())
  {
    // optionnel: print erreur
    // Serial.println("IMU init FAIL");
  }
  else
  {
    delay(200);
    imu_calibrate(600, 2); // ~1.2s, robot immobile
    // Serial.println("IMU calibrated");
  }
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
      continue;
    tPrev += (unsigned long)DT_MS * 1000UL;

    if (robot_handleSafetyStop())
      return;

    long curL, curR;
    encoders_read(&curL, &curR);

    long dL = labs(curL - startL);
    long dR = labs(curR - startR);

    if ((dL + dR) / 2 >= labs(targetTicks))
      break;
  }

  motors_stop();
}

void robot_rotate_gyro(float target_deg, int pwmMax)
{
  const uint16_t DT_MS = 10;

  // Debug (0/1): affiche angle/err/rate périodiquement.
  const bool DEBUG_GYRO = false;

  // Gains robustes pour petite plateforme
  const float KP = 2.0f;  // PWM / deg
  const float KD = 0.55f; // PWM / (deg/s) dans le sens de la rotation

  const int PWM_MIN = 50;
  const int PWM_MIN_NEAR = 30;
  const int DEAD_PWM = 0;

  const float ANGLE_TOL = 1.8f;
  const float RATE_TOL = 6.0f;
  const uint16_t STABLE_MS = 140;
  const float SLOW_ZONE_DEG = 14.0f;
  const float COAST_ZONE_DEG = 1.8f;
  // const float BRAKE_START_DEG = 40.0f;
  const float RATE_FILT_ALPHA = 0.25f;
  const uint16_t TIMEOUT_MS = 5000;
  const int RAMP_STEP = 8;

  // Deceleration estimate (deg/s^2) used to anticipate stopping distance.
  const float DECEL_DPS2 = 900.0f;

  float absTargetDeg = fabs(target_deg);
  float brakeStartDeg = constrain(absTargetDeg * 0.55f, 28.0f, 95.0f);

  // Recalage court du bias avant chaque rotation pour stabiliser la repetabilite.
  motors_stop();
  if (!robot_pauseable_delay(120))
    return;
  (void)imu_calibrate(120, 2);

  float angle = 0.0f;
  float rateFilt = 0.0f;
  float prevErr = target_deg;

  int gyroSign = 1;               // auto-correction si l'axe/signe est inversé
  bool gyroSignLocked = false;
  float signTowardSum = 0.0f;
  uint16_t signSamples = 0;

  unsigned long tPrev = micros();
  unsigned long startMs = millis();
  unsigned long stableStart = 0;

  int pwmLimit = 0;

  while (true)
  {
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL)
      continue;
    float dt = (float)(now - tPrev) / 1000000.0f;
    tPrev = now;

    if (robot_handleSafetyStop())
      return;

    float rateRaw = 0.0f;
    if (!imu_readGyroZ_dps_checked(rateRaw))
    {
      if (imu_getConsecutiveReadFailures() >= 15) // ~150ms
      {
        Serial.println("[ERR] IMU read fail (I2C) -> stop rotate_gyro");
        break;
      }
      // on saute cette itération (évite d'intégrer du 0 arbitraire)
      continue;
    }

    // Applique la correction de signe si nécessaire
    rateRaw *= (float)gyroSign;
    rateFilt += RATE_FILT_ALPHA * (rateRaw - rateFilt);
    angle += rateFilt * dt;

    float err = target_deg - angle;
    float absErr = fabs(err);

    if (pwmLimit < pwmMax)
      pwmLimit = min(pwmLimit + RAMP_STEP, pwmMax);

    // Sens commande uniquement base sur l'erreur cible -> evite le ping-pong aleatoire.
    int dir = (err >= 0.0f) ? 1 : -1;

    // Vitesse projetee dans le sens utile (+ = on se rapproche).
    float rateToward = rateFilt * (float)dir;

    // Auto-détection du signe: au début, si on "s'éloigne" (rateToward fortement négatif), on inverse.
    if (!gyroSignLocked)
    {
      if (pwmLimit >= min(80, pwmMax))
      {
        signTowardSum += rateToward;
        signSamples++;
      }

      if (millis() - startMs >= 280 && signSamples >= 8)
      {
        float avgToward = signTowardSum / (float)signSamples;
        // Si la rotation mesurée est globalement opposée au sens utile, on inverse.
        if (avgToward < -8.0f)
        {
          gyroSign = -gyroSign;
          angle = -angle;
          rateFilt = -rateFilt;
          prevErr = target_deg - angle;
          Serial.println("[IMU] gyro sign auto-inverse");
        }
        gyroSignLocked = true;
      }
    }
    float u = KP * absErr - KD * rateToward;

    int pwm = (int)fabs(u);
    pwm = constrain(pwm, 0, pwmLimit);

    // float decelRatio = (absErr - ANGLE_TOL) / (BRAKE_START_DEG - ANGLE_TOL);
    float decelRatio = (absErr - ANGLE_TOL) / (brakeStartDeg - ANGLE_TOL);
    decelRatio = constrain(decelRatio, 0.0f, 1.0f);
    int pwmCapErr = PWM_MIN_NEAR + (int)((pwmLimit - PWM_MIN_NEAR) * decelRatio);
    pwm = min(pwm, pwmCapErr);

    float stopDistDeg = (rateToward > 0.0f) ? (rateToward * rateToward) / (2.0f * DECEL_DPS2) : 0.0f;
    if (absErr <= stopDistDeg + 2.5f)
      pwm = min(pwm, PWM_MIN_NEAR + 5);
    if (absErr <= stopDistDeg)
      pwm = DEAD_PWM;

    if (absErr < COAST_ZONE_DEG)
    {
      pwm = DEAD_PWM;
      motors_stop();
    }
    else
    {
      int pwmMinLocal = (absErr < SLOW_ZONE_DEG) ? PWM_MIN_NEAR : PWM_MIN;
      if (pwm > 0)
        pwm = max(pwm, pwmMinLocal);
      else
        pwm = DEAD_PWM;

      if (dir > 0)
        motors_rotateRight(pwm);
      else
        motors_rotateLeft(pwm);
    }

    // Stop robuste: stable dans la fenetre OU croisement propre de la cible.
    bool crossedTarget = ((prevErr > 0.0f && err <= 0.0f) || (prevErr < 0.0f && err >= 0.0f));
    if ((absErr < ANGLE_TOL && fabs(rateFilt) < RATE_TOL) || (crossedTarget && absErr < 3.0f && fabs(rateFilt) < 20.0f))
    {
      if (stableStart == 0)
        stableStart = millis();
      if (millis() - stableStart >= STABLE_MS)
        break;
    }
    else
    {
      stableStart = 0;
    }

    prevErr = err;

    if (DEBUG_GYRO)
    {
      static unsigned long lastDbg = 0;
      unsigned long nowMs = millis();
      if (nowMs - lastDbg >= 120)
      {
        lastDbg = nowMs;
        Serial.print("[gyro] angle=");
        Serial.print(angle, 1);
        Serial.print(" err=");
        Serial.print(err, 1);
        Serial.print(" rate=");
        Serial.print(rateFilt, 1);
        Serial.print(" pwmLim=");
        Serial.println(pwmLimit);
      }
    }

    if (millis() - startMs > TIMEOUT_MS)
    {
      Serial.print("[WARN] rotate_gyro timeout angle=");
      Serial.print(angle, 1);
      Serial.print(" err=");
      Serial.print(err, 1);
      Serial.print(" rate=");
      Serial.println(rateFilt, 1);
      break;
    }
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
  if (!robot_pauseable_delay(60))
    return;

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
    // période fixe
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL)
      continue;
    tPrev += (unsigned long)DT_MS * 1000UL;

    if (robot_handleSafetyStop())
      return;

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
      if (robot_handleSafetyStop())
        return;

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
        Serial.println("[ERR] Encodeur asymetrique: arret securite");
        break;
      }
    }
    else
    {
      oneSideDeadSinceMs = 0;
    }

    if (millis() - startMs > MOVE_TIMEOUT_MS)
    {
      Serial.println("[ERR] Timeout deplacement: arret securite");
      break;
    }

    if (avgTicks >= target)
    {
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
}

void robot_move_distance_gyro(float dist_mm, int pwmBaseTarget)
{
  const uint16_t DT_MS = 10;
  const float KP_HEAD = 2.2f;  // PWM/deg
  const float KD_HEAD = 0.45f; // PWM/(deg/s)
  const float RATE_FILT_ALPHA = 0.25f;

  const int PWM_MIN = 45;
  const int PWM_MAX = 255;
  const int CORR_MAX = 60;
  const unsigned long MOVE_TIMEOUT_MS = 12000;

  long target = ticks_for_distance_mm(fabs(dist_mm));
  int dir = (dist_mm >= 0.0f) ? 1 : -1;

  long startL, startR;
  encoders_read(&startL, &startR);

  // Recalage gyro court au depart (robot immobile)
  motors_stop();
  delay(120);
  (void)imu_calibrate(120, 2);

  float headingDeg = 0.0f;
  float rateFilt = 0.0f;

  unsigned long tPrev = micros();
  unsigned long startMs = millis();

  while (true)
  {
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL)
      continue;

    float dt = (float)(now - tPrev) / 1000000.0f;
    tPrev = now;

    long curL, curR;
    encoders_read(&curL, &curR);

    long distTicksL = labs(curL - startL);
    long distTicksR = labs(curR - startR);
    long avgTicks = (distTicksL + distTicksR) / 2;
    if (avgTicks >= target)
      break;

    if (millis() - startMs > MOVE_TIMEOUT_MS)
    {
      Serial.println("[WARN] move_distance_gyro timeout");
      break;
    }

    // Estimation de cap via gyro (integration du yaw rate)
    float rateRaw = imu_readGyroZ_dps();
    rateFilt += RATE_FILT_ALPHA * (rateRaw - rateFilt);
    headingDeg += rateFilt * dt;

    // Si heading > 0, la correction freine la roue gauche et accelere la droite.
    float corr = KP_HEAD * headingDeg + KD_HEAD * rateFilt;
    corr = constrain(corr, -CORR_MAX, CORR_MAX);

    int pwmBase = constrain(pwmBaseTarget, 0, PWM_MAX);
    int pwmL = constrain((int)(pwmBase - corr + trimL), 0, PWM_MAX);
    int pwmR = constrain((int)(pwmBase + corr + trimR), 0, PWM_MAX);

    if (pwmL > 0)
      pwmL = max(pwmL, PWM_MIN);
    if (pwmR > 0)
      pwmR = max(pwmR, PWM_MIN);

    if (dir > 0)
      motors_forward(pwmL, pwmR);
    else
      motors_backward(pwmL, pwmR);
  }

  motors_stop();
}