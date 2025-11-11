#include <Arduino.h>
#include "ESP32_MPU6050.h"

ESP32_MPU6050::ESP32_MPU6050(int8_t address) : i2cAddress(address)
{
  gyroscope_offset = {0, 0, 0};
  accelerometer_offset = {0, 0, 0};
  readings = {{0, 0, 0}, {0, 0, 0}, 0};

  gyroscope_sensitivity = GYRO_SENSITIVITY_2000DPS;
  accelerometer_sensitivity = ACCEL_SENSITIVITY_16G;
}

bool ESP32_MPU6050::begin(GyroRange gyroRange, AccelRange accelRange, LpfBandwidth lpfBandwidth, uint8_t expected_who_am_i)
{
  Wire.begin();

  if (readRegister(MPU6050_WHO_AM_I) != expected_who_am_i)
  {
    return false;
  }

  if (!writeRegister(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_WAKE))
  {
    return false;
  }

  if (!setGyroscopeRange(gyroRange) || !setAccelerometerRange(accelRange))
  {
    return false;
  }

  if (!setLpfBandwidth(lpfBandwidth))
  {
    return false;
  }

  _gyro_range = gyroRange;
  _accel_range = accelRange;
  _lpf_bandwidth = lpfBandwidth;

  if (!writeRegister(MPU6050_FIFO_EN, (1 << FIFO_ACCEL_EN_BIT) | (1 << FIFO_GYRO_X_EN_BIT) | (1 << FIFO_GYRO_Y_EN_BIT) | (1 << FIFO_GYRO_Z_EN_BIT)))
  {
    return false;
  }

  resetFifo();

  return true;
}

bool ESP32_MPU6050::setLpfBandwidth(LpfBandwidth bandwidth)
{
  _lpf_bandwidth = bandwidth;
  return writeRegister(MPU6050_CONFIG, bandwidth);
}

bool ESP32_MPU6050::setGyroscopeRange(GyroRange range)
{
  _gyro_range = range;
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
  return writeRegister(MPU6050_GYRO_CONFIG, range << GYRO_CONFIG_SHIFT);
}

bool ESP32_MPU6050::setAccelerometerRange(AccelRange range)
{
  _accel_range = range;
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
  return writeRegister(MPU6050_ACCEL_CONFIG, range << ACCEL_CONFIG_SHIFT);
}

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
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    if (readSensorData(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz))
    {
      gyro_x_sum += raw_gx;
      gyro_y_sum += raw_gy;
      gyro_z_sum += raw_gz;
      accel_x_sum += raw_ax;
      accel_y_sum += raw_ay;
      accel_z_sum += raw_az;
    }
    delay(CALIBRATION_DELAY_MS);
  }

  gyroscope_offset.x = (float)gyro_x_sum / num_samples;
  gyroscope_offset.y = (float)gyro_y_sum / num_samples;
  gyroscope_offset.z = (float)gyro_z_sum / num_samples;

  accelerometer_offset.x = (float)accel_x_sum / num_samples;
  accelerometer_offset.y = (float)accel_y_sum / num_samples;
  accelerometer_offset.z = (float)accel_z_sum / num_samples - accelerometer_sensitivity;
}

bool ESP32_MPU6050::update()
{
  if (!readSensorData(&_raw_ax, &_raw_ay, &_raw_az, &_raw_gx, &_raw_gy, &_raw_gz))
  {
    return false;
  }

  readings.accelerometer.x = ((float)_raw_ax - accelerometer_offset.x) / accelerometer_sensitivity;
  readings.accelerometer.y = ((float)_raw_ay - accelerometer_offset.y) / accelerometer_sensitivity;
  readings.accelerometer.z = ((float)_raw_az - accelerometer_offset.z) / accelerometer_sensitivity;

  readings.gyroscope.x = ((float)_raw_gx - gyroscope_offset.x) / gyroscope_sensitivity;
  readings.gyroscope.y = ((float)_raw_gy - gyroscope_offset.y) / gyroscope_sensitivity;
  readings.gyroscope.z = ((float)_raw_gz - gyroscope_offset.z) / gyroscope_sensitivity;

  return true;
}

void ESP32_MPU6050::getRawReadings(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
  *ax = _raw_ax;
  *ay = _raw_ay;
  *az = _raw_az;
  *gx = _raw_gx;
  *gy = _raw_gy;
  *gz = _raw_gz;
}

bool ESP32_MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == I2C_TRANSMISSION_SUCCESS;
}


uint8_t ESP32_MPU6050::readRegister(uint8_t reg)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 1, true);
  return Wire.read();
}

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

uint16_t ESP32_MPU6050::getFifoCount()
{
  uint8_t buffer[2];
  readRegisters(MPU6050_FIFO_COUNTH, 2, buffer);
  return (buffer[0] << 8) | buffer[1];
}

void ESP32_MPU6050::resetFifo()
{
  writeRegister(MPU6050_USER_CTRL, (1 << MPU6050_USER_CTRL_FIFO_RESET_BIT));
  writeRegister(MPU6050_USER_CTRL, (1 << MPU6050_USER_CTRL_FIFO_EN_BIT));
}

bool ESP32_MPU6050::readSensorData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint16_t fifo_count = getFifoCount();
  if (fifo_count == 0)
  {
    return false;
  }

  int num_samples = fifo_count / MPU6050_DATA_BLOCK_SIZE;
  if (num_samples > MAX_FIFO_SAMPLES)
  {
    num_samples = MAX_FIFO_SAMPLES;
    fifo_count = num_samples * MPU6050_DATA_BLOCK_SIZE;
  }

  uint8_t buffer[fifo_count];
  readRegisters(MPU6050_FIFO_R_W, fifo_count, buffer);

  long raw_ax_sum = 0;
  long raw_ay_sum = 0;
  long raw_az_sum = 0;
  long raw_gx_sum = 0;
  long raw_gy_sum = 0;
  long raw_gz_sum = 0;

  for (int i = 0; i < num_samples; i++)
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

  *ax = raw_ax_sum / num_samples;
  *ay = raw_ay_sum / num_samples;
  *az = raw_az_sum / num_samples;
  *gx = raw_gx_sum / num_samples;
  *gy = raw_gy_sum / num_samples;
  *gz = raw_gz_sum / num_samples;

  return true;
}
