#pragma once
#include <Arduino.h>

void robot_init();
void robot_step();
void robot_move_distance(float dist_mm, int speed);
void robot_move_distance_gyro(float dist_mm, int pwmBaseTarget);
void robot_rotate(float angle_deg, int speed);
void robot_stop();
void robot_rotate_gyro(float target_deg, int pwmMax);

// Délai qui garde l'arrêt d'urgence actif (via safety).
// Renvoie false si l'arrêt d'urgence / sécurité s'est déclenché pendant l'attente.
bool robot_pauseable_delay(uint16_t ms);