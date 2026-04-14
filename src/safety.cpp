#include "../lib/safety.h"
#include "../lib/emergencyButton.h"
#include "../lib/ultrasonic.h"

static int g_obstacle_cm;
static uint16_t g_sonar_period;

static bool g_triggered = false;
static int g_lastDist1 = -1;
static int g_lastDist2 = -1;
static unsigned long g_lastSonarMs = 0;
static unsigned long g_sonarWarmupUntilMs = 0;
static bool g_emergencyActive = false;
static bool g_obstacleActive = false;

static int g_hist1[3] = {-1, -1, -1};
static int g_hist2[3] = {-1, -1, -1};
static uint8_t g_histIndex = 0;
static uint8_t g_obstacleCount = 0;
static uint8_t g_clearCount = 0;

static int filteredMedian3(const int hist[3])
{
  int vals[3];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 3; i++)
  {
    int v = hist[i];
    if (v > 0)
      vals[n++] = v;
  }

  if (n == 0)
    return -1;
  if (n == 1)
    return vals[0];
  if (n == 2)
    return (vals[0] + vals[1]) / 2;

  // n == 3 -> sort 3 values
  if (vals[0] > vals[1])
  {
    int t = vals[0];
    vals[0] = vals[1];
    vals[1] = t;
  }
  if (vals[1] > vals[2])
  {
    int t = vals[1];
    vals[1] = vals[2];
    vals[2] = t;
  }
  if (vals[0] > vals[1])
  {
    int t = vals[0];
    vals[0] = vals[1];
    vals[1] = t;
  }
  return vals[1];
}

void safety_init(int obstacle_cm, uint16_t sonar_period_ms) {
  g_obstacle_cm = obstacle_cm;
  g_sonar_period = sonar_period_ms;
  g_triggered = false;
  g_lastDist1 = -1;
  g_lastDist2 = -1;
  g_emergencyActive = false;
  g_obstacleActive = false;
  g_hist1[0] = g_hist1[1] = g_hist1[2] = -1;
  g_hist2[0] = g_hist2[1] = g_hist2[2] = -1;
  g_histIndex = 0;
  g_obstacleCount = 0;
  g_clearCount = 0;

  // Comme ton sketch: laisse le temps aux ultrasons/stabilisation après démarrage.
  // Le bouton d'urgence reste actif immédiatement.
  g_sonarWarmupUntilMs = millis() + 3000UL;

  emergencyButton_init();
}

void safety_update() {
  // bouton (rapide)
  {
    static bool lastPressed = false;
    bool pressed = emergencyButton_isPressed();
    g_emergencyActive = pressed;
    if (pressed != lastPressed)
    {
      lastPressed = pressed;
      Serial.print("[EMERGENCY] pressed=");
      Serial.print(pressed ? "1" : "0");
      Serial.print(" raw=");
      Serial.println(emergencyButton_rawRead() ? "1" : "0");
    }
  }

  // ultrason (lent → throttling)
  unsigned long now = millis();
  if (now - g_lastSonarMs >= g_sonar_period) {
    g_lastSonarMs = now;

    // Comme dans ton sketch: lire 2 capteurs et déclencher si l'un voit un obstacle.
    int raw1 = ultrasonic_readDistance();
    // Délai volontairement plus long pour limiter le crosstalk entre 2 HC-SR04.
    delay(20);
    int raw2 = ultrasonic_readDistance2();

    g_hist1[g_histIndex] = raw1;
    g_hist2[g_histIndex] = raw2;
    g_histIndex = (uint8_t)((g_histIndex + 1) % 3);

    // On expose des distances filtrées (plus stables) et on s'en sert pour la logique obstacle.
    g_lastDist1 = filteredMedian3(g_hist1);
    g_lastDist2 = filteredMedian3(g_hist2);

    // Pendant le warmup, on met à jour les distances mais on n'arme pas le stop sur sonar.
    if (now >= g_sonarWarmupUntilMs)
    {
      const bool rawObstacle = ((g_lastDist1 > 0 && g_lastDist1 <= g_obstacle_cm) ||
                                (g_lastDist2 > 0 && g_lastDist2 <= g_obstacle_cm));

      // Hystérésis simple: évite de bloquer sur 1 lecture bruitée.
      const uint8_t TRIG_N = 2;
      const uint8_t CLEAR_N = 2;

      if (rawObstacle)
      {
        if (g_obstacleCount < TRIG_N)
          g_obstacleCount++;
        g_clearCount = 0;
      }
      else
      {
        if (g_clearCount < CLEAR_N)
          g_clearCount++;
        g_obstacleCount = 0;
      }

      if (g_obstacleCount >= TRIG_N)
        g_obstacleActive = true;
      else if (g_clearCount >= CLEAR_N)
        g_obstacleActive = false;
    }
    else
    {
      g_obstacleActive = false;
      g_obstacleCount = 0;
      g_clearCount = 0;
    }
  }

  g_triggered = (g_emergencyActive || g_obstacleActive);
}

bool safety_isTriggered() {
  return g_triggered;
}

void safety_clearIfSafe() {
  bool us1Safe = (g_lastDist1 <= 0 || g_lastDist1 > g_obstacle_cm);
  bool us2Safe = (g_lastDist2 <= 0 || g_lastDist2 > g_obstacle_cm);
  if (!emergencyButton_isPressed() && us1Safe && us2Safe) {
    g_triggered = false;
    g_emergencyActive = false;
    g_obstacleActive = false;
  }
}

bool safety_isEmergencyActive() {
  return g_emergencyActive;
}

bool safety_isObstacleActive() {
  return g_obstacleActive;
}

int safety_getObstacleThresholdCm() {
  return g_obstacle_cm;
}

void safety_getLastDistancesCm(int &out_us1, int &out_us2) {
  out_us1 = g_lastDist1;
  out_us2 = g_lastDist2;
}