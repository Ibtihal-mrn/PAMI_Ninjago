#include "../lib/imu.h"
#include <Wire.h>

static const uint8_t MPU_ADDR = 0x68;

// Registres MPU6050
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG= 0x1C;
static const uint8_t REG_CONFIG      = 0x1A;
static const uint8_t REG_SMPLRT_DIV  = 0x19;
static const uint8_t REG_GYRO_ZOUT_H = 0x47;

// Plage gyro: si le robot tourne vite, ±250°/s peut saturer et fausser l'integration.
// Choix: ±500°/s (FS_SEL=1) => 65.5 LSB/(deg/s)
static const uint8_t GYRO_FS_SEL_500DPS = 0x08;
static const float GYRO_SENS = 65.5f;

static float gyroZ_bias_dps = 0.0f;

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  uint8_t n = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len, (uint8_t)true);
  if (n != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool imu_init() {
  Wire.begin();

  // Réveil
  if (!writeReg(REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);

  // Filtre passe-bas (DLPF). 3 ~ 44Hz (souvent bien pour robot)
  if (!writeReg(REG_CONFIG, 0x03)) return false;

  // Sample rate divider (optionnel). Avec DLPF actif, base 1kHz -> /9 => 100Hz
  if (!writeReg(REG_SMPLRT_DIV, 9)) return false;

  // Gyro ±500°/s (evite la saturation pendant les rotations rapides)
  if (!writeReg(REG_GYRO_CONFIG, GYRO_FS_SEL_500DPS)) return false;

  // Accel (pas indispensable ici). Laisse par défaut ±2g
  if (!writeReg(REG_ACCEL_CONFIG, 0x00)) return false;

  return true;
}

bool imu_calibrate(uint16_t samples, uint16_t delay_ms) {
  // Le robot doit être IMMOBILE pendant la calibration
  float sum = 0.0f;
  uint16_t ok = 0;

  for (uint16_t i = 0; i < samples; i++) {
    uint8_t b[2];
    if (readBytes(REG_GYRO_ZOUT_H, b, 2)) {
      int16_t raw = (int16_t)((b[0] << 8) | b[1]);
      float dps = raw / GYRO_SENS;
      sum += dps;
      ok++;
    }
    delay(delay_ms);
  }

  if (ok < samples / 2) return false;

  gyroZ_bias_dps = sum / ok;
  return true;
}

float imu_readGyroZ_dps() {
  uint8_t b[2];
  if (!readBytes(REG_GYRO_ZOUT_H, b, 2)) return 0.0f;

  int16_t raw = (int16_t)((b[0] << 8) | b[1]);
  float dps = raw / GYRO_SENS;
  return dps - gyroZ_bias_dps;
}