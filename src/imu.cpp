#include "../lib/imu.h"
#include <Wire.h>

static uint8_t g_mpuAddr = 0x68;
static bool g_inited = false;
static uint16_t g_consecutiveReadFails = 0;

// Registres MPU6050
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG= 0x1C;
static const uint8_t REG_CONFIG      = 0x1A;
static const uint8_t REG_SMPLRT_DIV  = 0x19;
static const uint8_t REG_GYRO_ZOUT_H = 0x47;
static const uint8_t REG_WHO_AM_I    = 0x75;

// Plage gyro: si le robot tourne vite, ±250°/s peut saturer et fausser l'integration.
// Choix: ±500°/s (FS_SEL=1) => 65.5 LSB/(deg/s)
static const uint8_t GYRO_FS_SEL_500DPS = 0x08;
static const float GYRO_SENS = 65.5f;

static float gyroZ_bias_dps = 0.0f;

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(g_mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(g_mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  uint8_t n = Wire.requestFrom((uint8_t)g_mpuAddr, (uint8_t)len, (uint8_t)true);
  if (n != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool detectAddress(uint8_t &addrOut)
{
  const uint8_t candidates[2] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2; i++)
  {
    Wire.beginTransmission(candidates[i]);
    if (Wire.endTransmission() == 0)
    {
      addrOut = candidates[i];
      return true;
    }
  }
  return false;
}

bool imu_init() {
  Wire.begin();
  Wire.setClock(100000);

  // Detect 0x68 / 0x69 automatically
  uint8_t addr = 0x68;
  if (!detectAddress(addr))
  {
    Serial.println("[IMU] MPU6050 detect FAIL (0x68/0x69)");
    g_inited = false;
    g_consecutiveReadFails = 0;
    return false;
  }
  g_mpuAddr = addr;

  Serial.print("[IMU] MPU6050 detecte a 0x");
  Serial.println(g_mpuAddr, HEX);

  // Réveil
  if (!writeReg(REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);

  // Optional: WHO_AM_I read (non-bloquant)
  {
    uint8_t who = 0;
    if (readBytes(REG_WHO_AM_I, &who, 1))
    {
      Serial.print("[IMU] WHO_AM_I=0x");
      Serial.println(who, HEX);
    }
    else
    {
      Serial.println("[IMU] WHO_AM_I read FAIL");
    }
  }

  // Filtre passe-bas (DLPF). 3 ~ 44Hz (souvent bien pour robot)
  if (!writeReg(REG_CONFIG, 0x03)) return false;

  // Sample rate divider (optionnel). Avec DLPF actif, base 1kHz -> /9 => 100Hz
  if (!writeReg(REG_SMPLRT_DIV, 9)) return false;

  // Gyro ±500°/s (evite la saturation pendant les rotations rapides)
  if (!writeReg(REG_GYRO_CONFIG, GYRO_FS_SEL_500DPS)) return false;

  // Accel (pas indispensable ici). Laisse par défaut ±2g
  if (!writeReg(REG_ACCEL_CONFIG, 0x00)) return false;

  g_consecutiveReadFails = 0;
  g_inited = true;
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
  float out = 0.0f;
  if (!imu_readGyroZ_dps_checked(out))
    return 0.0f;
  return out;
}

bool imu_readGyroZ_dps_checked(float &out_dps)
{
  if (!g_inited)
  {
    g_consecutiveReadFails++;
    return false;
  }

  uint8_t b[2];
  if (!readBytes(REG_GYRO_ZOUT_H, b, 2))
  {
    g_consecutiveReadFails++;
    return false;
  }

  g_consecutiveReadFails = 0;
  int16_t raw = (int16_t)((b[0] << 8) | b[1]);
  float dps = raw / GYRO_SENS;
  out_dps = dps - gyroZ_bias_dps;
  return true;
}

uint16_t imu_getConsecutiveReadFailures()
{
  return g_consecutiveReadFails;
}