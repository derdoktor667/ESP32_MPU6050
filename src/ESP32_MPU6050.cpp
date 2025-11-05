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
  if (!setGyroscopeRange(gyroRange) || !setAccelerometerRange(accelRange))
  {
    return false;
  }

  // Set the Digital Low Pass Filter (DLPF) bandwidth.
  if (!setLpfBandwidth(lpfBandwidth))
  {
    return false;
  }

  // Enable FIFO for both accelerometer and gyroscope
  if (!writeRegister(MPU6050_FIFO_EN, (1 << MPU6050_FIFO_EN_ACCEL_BIT) | (1 << MPU6050_FIFO_EN_GYRO_X_BIT) | (1 << MPU6050_FIFO_EN_GYRO_Y_BIT) | (1 << MPU6050_FIFO_EN_GYRO_Z_BIT)))
  {
    return false;
  }

  // Reset and enable FIFO
  resetFifo();

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
  accelerometer_offset.z = (float)accel_z_sum / num_samples - accelerometer_sensitivity;
}

// update(): Read all sensor data from registers and convert to physical units.
bool ESP32_MPU6050::update()
{
  uint16_t fifo_count = getFifoCount();
  if (fifo_count == 0)
  {
    return false;
  }

  // Burst read the data from the FIFO
  uint8_t buffer[fifo_count];
  readRegisters(MPU6050_FIFO_R_W, fifo_count, buffer);

  long raw_ax_sum = 0;
  long raw_ay_sum = 0;
  long raw_az_sum = 0;
  long raw_gx_sum = 0;
  long raw_gy_sum = 0;
  long raw_gz_sum = 0;

  int samples = fifo_count / MPU6050_DATA_BLOCK_SIZE;

  for (int i = 0; i < samples; i++)
  {
    int16_t raw_ax = (buffer[i * MPU6050_DATA_BLOCK_SIZE] << 8) | buffer[i * MPU6050_DATA_BLOCK_SIZE + 1];
    int16_t raw_ay = (buffer[i * MPU6050_DATA_BLOCK_SIZE + 2] << 8) | buffer[i * MPU6050_DATA_BLOCK_SIZE + 3];
    int16_t raw_az = (buffer[i * MPU6050_DATA_BLOCK_SIZE + 4] << 8) | buffer[i * MPU6050_DATA_BLOCK_SIZE + 5];
    int16_t raw_gx = (buffer[i * MPU6050_DATA_BLOCK_SIZE + 8] << 8) | buffer[i * MPU6050_DATA_BLOCK_SIZE + 9];
    int16_t raw_gy = (buffer[i * MPU6050_DATA_BLOCK_SIZE + 10] << 8) | buffer[i * MPU6050_DATA_BLOCK_SIZE + 11];
    int16_t raw_gz = (buffer[i * MPU6050_DATA_BLOCK_SIZE + 12] << 8) | buffer[i * MPU6050_DATA_BLOCK_SIZE + 13];

    raw_ax_sum += raw_ax;
    raw_ay_sum += raw_ay;
    raw_az_sum += raw_az;
    raw_gx_sum += raw_gx;
    raw_gy_sum += raw_gy;
    raw_gz_sum += raw_gz;
  }

  readings.accelerometer.x = ((float)(raw_ax_sum / samples) - accelerometer_offset.x) / accelerometer_sensitivity;
  readings.accelerometer.y = ((float)(raw_ay_sum / samples) - accelerometer_offset.y) / accelerometer_sensitivity;
  readings.accelerometer.z = ((float)(raw_az_sum / samples) - accelerometer_offset.z) / accelerometer_sensitivity;

  readings.gyroscope.x = ((float)(raw_gx_sum / samples) - gyroscope_offset.x) / gyroscope_sensitivity;
  readings.gyroscope.y = ((float)(raw_gy_sum / samples) - gyroscope_offset.y) / gyroscope_sensitivity;
  readings.gyroscope.z = ((float)(raw_gz_sum / samples) - gyroscope_offset.z) / gyroscope_sensitivity;

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

// Private Helper Functions

void ESP32_MPU6050::resetFifo()
{
  writeRegister(MPU6050_USER_CTRL, (1 << MPU6050_USER_CTRL_FIFO_RESET_BIT));
  writeRegister(MPU6050_USER_CTRL, (1 << MPU6050_USER_CTRL_FIFO_EN_BIT));
}

uint16_t ESP32_MPU6050::getFifoCount()
{
  uint8_t buffer[2];
  readRegisters(MPU6050_FIFO_COUNTH, 2, buffer);
  return (buffer[0] << 8) | buffer[1];
}
