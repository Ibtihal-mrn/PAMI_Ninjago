#pragma once
#include <Arduino.h>

void robot_init();
void robot_step();
void robot_move_distance(float dist_mm, int speed);
void robot_move_distance_gyro(float dist_mm, int pwmBaseTarget);
void robot_rotate(float angle_deg, int speed);
void robot_stop();
void robot_rotate_gyro(float target_deg, int pwmMax);
void robot_move_timed(int pwmL, int pwmR, uint32_t ms);
void robot_back_timed(int pwmL, int pwmR, uint32_t ms);
void robot_move_timed_distance_cm(float dist_cm);
void robot_back_timed_distance_cm(float dist_cm);


// Remplace delay() quand tu veux que la séquence puisse se mettre en pause
// et reprendre via le bouton d'arrêt d'urgence (toggle).
bool robot_pauseable_delay(uint16_t ms);