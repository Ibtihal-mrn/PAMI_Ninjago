#include <Arduino.h>
#include "../lib/encoders.h"
#include "../lib/motors.h"
#include "../lib/control.h"
#include "../lib/robot.h"
#include "../lib/teamSwitch.h"

static const uint8_t PIN_TIRETTE = 4;
static const uint8_t PIN_TEAM_SWITCH = 5;

static Team g_team = Team::A;

void setup()
{
  Serial.begin(9600);
  robot_init();

  // Lecture du switch équipe (comme dans ton sketch): LOW => équipe B, HIGH => équipe A.
  TeamSwitch teamSwitch(PIN_TEAM_SWITCH);
  teamSwitch.begin();
  g_team = teamSwitch.readTeam();
  if (g_team == Team::B)
    Serial.println("Equipe B selectionnee");
  else
    Serial.println("Equipe A selectionnee");

  pinMode(PIN_TIRETTE, INPUT_PULLUP);
  Serial.println("En attente de la tirette...");

  // Comme dans ton code qui marche: on attend que la broche passe a LOW.
  // Avec INPUT_PULLUP, LOW = broche tiree vers GND (tirette retiree / contact ferme selon cablage).
  while (digitalRead(PIN_TIRETTE) == HIGH)
  {
    delay(10);
  }

  Serial.println("Tirette detectee -> DEMARRAGE");
  delay(300);

  // // Smoke test moteur (bypass encodeurs/regulation) : doit bouger comme ton sketch minimal.
  // Serial.println("[smoke] moteurs AVANT 1s");
  // motors_forward(120, 120);
  // delay(1000);
  // motors_stop();
  // delay(300);

  // Serial.println("[smoke] moteurs ARRIERE 1s");
  // motors_backward(120, 120);
  // delay(1000);
  // motors_stop();
  // delay(500);

  // // Diagnostic encodeurs: sans bouger, les ticks ne doivent PAS monter.
  // ticksL = 0;
  // ticksR = 0;
  // Serial.println("[diag] encodeurs a l'arret (2s)");
  // unsigned long t0 = millis();
  // while (millis() - t0 < 2000)
  // {
  //   static unsigned long last = 0;
  //   if (millis() - last >= 200)
  //   {
  //     last = millis();
  //     Serial.print("ticksL=");
  //     Serial.print((long)ticksL);
  //     Serial.print(" ticksR=");
  //     Serial.println((long)ticksR);
  //   }
  //   delay(5);
  // }
}



void loop()
{
  static bool runSequence = true;  

  if (!runSequence) {
    return; 
  }

  // -----------------------------------
  // --------- Séquence (sans gyro) -----
  // -----------------------------------

  if (g_team == Team::A)
  {
    Serial.println("=== Sequence equipe A ===");
    robot_move_distance(120, 120);
    delay(500);
    robot_move_distance(120, 120);
    // robot_pauseable_delay(500);
    // robot_pauseable_delay(500);
    // robot_move_distance(320, 80);
    // robot_pauseable_delay(1000);
    // robot_pauseable_delay(1000);
    // robot_move_distance(270, 70);
    // robot_pauseable_delay(500);
    // robot_pauseable_delay(1000);
    // robot_move_distance(280, 70);
    // robot_pauseable_delay(500);
  }
  else
  {
    Serial.println("=== Sequence equipe B ===");
    // Variante legere pour valider que le switch est bien pris en compte.
    robot_move_distance(200, 70);
    robot_pauseable_delay(500);
    robot_pauseable_delay(500);
    robot_move_distance(300, 80);
    robot_pauseable_delay(1000);
    robot_pauseable_delay(1000);
    robot_move_distance(250, 70);
    robot_pauseable_delay(500);
    robot_pauseable_delay(1000);
    robot_move_distance(260, 70);
    robot_pauseable_delay(500);
  }
  robot_stop();


 



  runSequence = false; 
} 


// Print 
// Serial.print("ticksL="); Serial.print(ticksL);
// Serial.print(" ticksR="); Serial.println(ticksR);

// Serial.print(digitalRead(ENC_R_A));
// Serial.print(" ");
// Serial.println(digitalRead(ENC_R_B));
// delay(50);


// long l, r ; 
// encoders_read(&l, &r);

// static long lastL = 0, lastR = 0;

// Serial.print("dTicksL="); Serial.print(l - lastL);
// Serial.print(" dTicksR="); Serial.println(r - lastR);


// lastL = ticksL;
// lastR = ticksR;