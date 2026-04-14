#pragma once
#include <Arduino.h>

bool imu_init();
bool imu_calibrate(uint16_t samples = 500, uint16_t delay_ms = 3);

// Renvoie la vitesse angulaire Z (yaw rate) en deg/s, bias déjà retiré
float imu_readGyroZ_dps();

// Variante robuste: renvoie false si la lecture I2C a échoué.
bool imu_readGyroZ_dps_checked(float &out_dps);

// Nombre d'échecs consécutifs de lecture I2C (gyro).
uint16_t imu_getConsecutiveReadFailures();