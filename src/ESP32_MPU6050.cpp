#include <Arduino.h>
#include "ESP32_MPU6050.h"

// Constructor: Initialize I2C address and zero out offsets and readings.
ESP32_MPU6050::ESP32_MPU6050(int8_t address) : i2cAddress(address)
{
  gyroscope_offset = {0, 0, 0};
  accelerometer_offset = {0, 0, 0};
  readings = {{0, 0, 0}, {0, 0, 0}, 0}; // IMPORTANT: Initialize readings to prevent garbage data on first run.

  // Set default sensitivity to prevent division by zero. These are for the default ranges (2000DPS and 16G).
  gyroscope_sensitivity = GYRO_SENSITIVITY_2000DPS;
  accelerometer_sensitivity = ACCEL_SENSITIVITY_16G;
}

// begin(): Initialize the MPU6050 sensor.
bool ESP32_MPU6050::begin(GyroRange gyroRange, AccelRange accelRange, LpfBandwidth lpfBandwidth)
{
  Wire.begin();

  // Verify sensor connection by checking the WHO_AM_I register.
  uint8_t who_am_i_val = readRegister(MPU6050_WHO_AM_I);
  Serial.print("MPU6050 WHO_AM_I register value: 0x");
  Serial.println(who_am_i_val, HEX);
  if (who_am_i_val != MPU6050_WHO_AM_I_EXPECTED_VALUE)
  {
    return false;
  }

  // Wake up the sensor by writing 0 to the power management register.
  if (!writeRegister(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_WAKE))
  {
    return false;
  }

  // Set the full-scale ranges for gyroscope and accelerometer. This also sets the sensitivity values.
  if (!setGyroscopeRange(gyroRange))
    return false;
  if (!setAccelerometerRange(accelRange))
    return false;

  // Set the Digital Low Pass Filter (DLPF) bandwidth.
  if (!setLpfBandwidth(lpfBandwidth))
    return false;

  // Ensure FIFO is disabled, as we are reading registers directly.
  writeRegister(MPU6050_USER_CTRL, 0); // Clear all bits, especially FIFO_EN
  writeRegister(MPU6050_FIFO_EN, 0);   // Disable all FIFO writes

  return true;
}

// setLpfBandwidth(): Set the DLPF configuration.
bool ESP32_MPU6050::setLpfBandwidth(LpfBandwidth bandwidth)
{
  return writeRegister(MPU6050_CONFIG, bandwidth);
}

// setGyroscopeRange(): Set the gyro range and its corresponding sensitivity.
bool ESP32_MPU6050::setGyroscopeRange(GyroRange range)
{
  switch (range)
  {
  case GYRO_RANGE_250DPS:
    gyroscope_sensitivity = GYRO_SENSITIVITY_250DPS;
    break;
  case GYRO_RANGE_500DPS:
    gyroscope_sensitivity = GYRO_SENSITIVITY_500DPS;
    break;
  case GYRO_RANGE_1000DPS:
    gyroscope_sensitivity = GYRO_SENSITIVITY_1000DPS;
    break;
  case GYRO_RANGE_2000DPS:
    gyroscope_sensitivity = GYRO_SENSITIVITY_2000DPS;
    break;
  }
  return writeRegister(MPU6050_GYRO_CONFIG, range << 3);
}

// setAccelerometerRange(): Set the accel range and its corresponding sensitivity.
bool ESP32_MPU6050::setAccelerometerRange(AccelRange range)
{
  switch (range)
  {
  case ACCEL_RANGE_2G:
    accelerometer_sensitivity = ACCEL_SENSITIVITY_2G;
    break;
  case ACCEL_RANGE_4G:
    accelerometer_sensitivity = ACCEL_SENSITIVITY_4G;
    break;
  case ACCEL_RANGE_8G:
    accelerometer_sensitivity = ACCEL_SENSITIVITY_8G;
    break;
  case ACCEL_RANGE_16G:
    accelerometer_sensitivity = ACCEL_SENSITIVITY_16G;
    break;
  }
  return writeRegister(MPU6050_ACCEL_CONFIG, range << 3);
}

// calibrate(): Calculate sensor offsets by averaging raw readings.
void ESP32_MPU6050::calibrate(int num_samples)
{
  long gyro_x_sum = 0;
  long gyro_y_sum = 0;
  long gyro_z_sum = 0;
  long accel_x_sum = 0;
  long accel_y_sum = 0;
  long accel_z_sum = 0;

  for (int i = 0; i < num_samples; ++i)
  {
    uint8_t buffer[14];
    if (!readRegisters(MPU6050_ACCEL_XOUT_H, 14, buffer))
    {
      // Don't print error in a loop, just skip sample
      continue;
    }

    int16_t raw_ax = (buffer[0] << 8) | buffer[1];
    int16_t raw_ay = (buffer[2] << 8) | buffer[3];
    int16_t raw_az = (buffer[4] << 8) | buffer[5];
    int16_t raw_gx = (buffer[8] << 8) | buffer[9];
    int16_t raw_gy = (buffer[10] << 8) | buffer[11];
    int16_t raw_gz = (buffer[12] << 8) | buffer[13];

    gyro_x_sum += raw_gx;
    gyro_y_sum += raw_gy;
    gyro_z_sum += raw_gz;
    accel_x_sum += raw_ax;
    accel_y_sum += raw_ay;
    accel_z_sum += raw_az;
    delay(CALIBRATION_DELAY_MS);
  }

  gyroscope_offset.x = (float)gyro_x_sum / num_samples;
  gyroscope_offset.y = (float)gyro_y_sum / num_samples;
  gyroscope_offset.z = (float)gyro_z_sum / num_samples;

  accelerometer_offset.x = (float)accel_x_sum / num_samples;
  accelerometer_offset.y = (float)accel_y_sum / num_samples;
  // For the Z-axis, we need to subtract the 1g of gravity from the average to get the true offset.
  accelerometer_offset.z = (float)accel_z_sum / num_samples - accelerometer_sensitivity;
}

// update(): Read all sensor data from registers and convert to physical units.
bool ESP32_MPU6050::update()
{
  uint8_t buffer[14];
  if (!readRegisters(MPU6050_ACCEL_XOUT_H, 14, buffer))
  {
    return false;
  }

  int16_t raw_ax = (buffer[0] << 8) | buffer[1];
  int16_t raw_ay = (buffer[2] << 8) | buffer[3];
  int16_t raw_az = (buffer[4] << 8) | buffer[5];
  int16_t raw_temp = (buffer[6] << 8) | buffer[7];
  int16_t raw_gx = (buffer[8] << 8) | buffer[9];
  int16_t raw_gy = (buffer[10] << 8) | buffer[11];
  int16_t raw_gz = (buffer[12] << 8) | buffer[13];

  readings.accelerometer.x = ((float)raw_ax - accelerometer_offset.x) / accelerometer_sensitivity;
  readings.accelerometer.y = ((float)raw_ay - accelerometer_offset.y) / accelerometer_sensitivity;
  readings.accelerometer.z = ((float)raw_az - accelerometer_offset.z) / accelerometer_sensitivity;

  readings.gyroscope.x = ((float)raw_gx - gyroscope_offset.x) / gyroscope_sensitivity;
  readings.gyroscope.y = ((float)raw_gy - gyroscope_offset.y) / gyroscope_sensitivity;
  readings.gyroscope.z = ((float)raw_gz - gyroscope_offset.z) / gyroscope_sensitivity;

  readings.temperature_celsius = (raw_temp / TEMP_SENSITIVITY_LSB_PER_DEGREE) + TEMP_OFFSET_DEGREES_CELSIUS;

  return true;
}

// writeRegister(): Write a single byte to a specific register.
bool ESP32_MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == I2C_TRANSMISSION_SUCCESS;
}

// readRegister(): Read a single byte from a specific register.
uint8_t ESP32_MPU6050::readRegister(uint8_t reg)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 1, true);
  return Wire.read();
}

// readRegisters(): Read multiple bytes from a starting register.
bool ESP32_MPU6050::readRegisters(uint8_t reg, uint8_t count, uint8_t *dest)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != I2C_TRANSMISSION_SUCCESS)
  {
    return false;
  }

  if (Wire.requestFrom(i2cAddress, count, true) != count)
  {
    return false;
  }

  for (uint8_t i = 0; i < count; i++)
  {
    dest[i] = Wire.read();
  }
  return true;
}
