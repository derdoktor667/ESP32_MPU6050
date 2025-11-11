#include <Arduino.h>
#include "ESP32_MPU6050.h"

ESP32_MPU6050::ESP32_MPU6050(int8_t address) : i2cAddress(address)
{
  gyroscope_offset = {0, 0, 0};
  accelerometer_offset = {0, 0, 0};
  readings = {{0, 0, 0}, {0, 0, 0}, 0};

  gyroscope_sensitivity = GYRO_SENSITIVITY_2000DPS;
  accelerometer_sensitivity = ACCEL_SENSITIVITY_16G;
  _fifo_data_block_size = 0;
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

  FifoConfig defaultConfig; // Accel and Gyro enabled by default, Temp disabled
  if (!configureFifo(defaultConfig))
  {
    return false;
  }

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

bool ESP32_MPU6050::configureFifo(const FifoConfig &config)
{
  uint8_t fifo_enable_mask = 0;
  _fifo_data_block_size = 0;

  if (config.enable_accel)
  {
    fifo_enable_mask |= (1 << FIFO_ACCEL_EN_BIT);
    _fifo_data_block_size += 6; // 3 axes * 2 bytes/axis
  }
  if (config.enable_gyro_x)
  {
    fifo_enable_mask |= (1 << FIFO_GYRO_X_EN_BIT);
    _fifo_data_block_size += 2; // 1 axis * 2 bytes/axis
  }
  if (config.enable_gyro_y)
  {
    fifo_enable_mask |= (1 << FIFO_GYRO_Y_EN_BIT);
    _fifo_data_block_size += 2; // 1 axis * 2 bytes/axis
  }
  if (config.enable_gyro_z)
  {
    fifo_enable_mask |= (1 << FIFO_GYRO_Z_EN_BIT);
    _fifo_data_block_size += 2; // 1 axis * 2 bytes/axis
  }
  if (config.enable_temp)
  {
    fifo_enable_mask |= (1 << TEMP_FIFO_EN_BIT);
    _fifo_data_block_size += 2; // 1 axis * 2 bytes/axis
  }

  // Enable/Disable FIFO in MPU6050_USER_CTRL
  uint8_t user_ctrl_reg = readRegister(MPU6050_USER_CTRL);
  if (fifo_enable_mask > 0)
  {
    user_ctrl_reg |= (1 << MPU6050_USER_CTRL_FIFO_EN_BIT);
  }
  else
  {
    user_ctrl_reg &= ~(1 << MPU6050_USER_CTRL_FIFO_EN_BIT);
  }
  if (!writeRegister(MPU6050_USER_CTRL, user_ctrl_reg))
  {
    return false;
  }

  // Write FIFO enable mask
  if (!writeRegister(MPU6050_FIFO_EN, fifo_enable_mask))
  {
    return false;
  }

  // Reset FIFO after configuration
  resetFifo();

  return true;
}

bool ESP32_MPU6050::getFifoSample(SensorReadings &sample)
{
  if (_fifo_data_block_size == 0)
  {
    return false; // FIFO not configured or empty
  }

  if (getFifoCount() < _fifo_data_block_size)
  {
    return false; // Not enough data for a full sample
  }

  uint8_t buffer[_fifo_data_block_size];
  if (!readRegisters(MPU6050_FIFO_R_W, _fifo_data_block_size, buffer))
  {
    return false;
  }

  // Parse the buffer based on the configured FIFO
  uint8_t buffer_idx = 0;

  // Accelerometer data
  if (((readRegister(MPU6050_FIFO_EN) >> FIFO_ACCEL_EN_BIT) & 0x01) && (buffer_idx + 6 <= _fifo_data_block_size))
  {
    sample.accelerometer.x = (float)((int16_t)(buffer[buffer_idx] << 8 | buffer[buffer_idx + 1])) / accelerometer_sensitivity;
    sample.accelerometer.y = (float)((int16_t)(buffer[buffer_idx + 2] << 8 | buffer[buffer_idx + 3])) / accelerometer_sensitivity;
    sample.accelerometer.z = (float)((int16_t)(buffer[buffer_idx + 4] << 8 | buffer[buffer_idx + 5])) / accelerometer_sensitivity;
    buffer_idx += 6;
  }
  else
  {
    sample.accelerometer = {0, 0, 0};
  }

  // Gyroscope data
  uint8_t gyro_fifo_en = readRegister(MPU6050_FIFO_EN);
  if (((gyro_fifo_en >> FIFO_GYRO_X_EN_BIT) & 0x01) && (buffer_idx + 2 <= _fifo_data_block_size))
  {
    sample.gyroscope.x = (float)((int16_t)(buffer[buffer_idx] << 8 | buffer[buffer_idx + 1])) / gyroscope_sensitivity;
    buffer_idx += 2;
  }
  else
  {
    sample.gyroscope.x = 0;
  }
  if (((gyro_fifo_en >> FIFO_GYRO_Y_EN_BIT) & 0x01) && (buffer_idx + 2 <= _fifo_data_block_size))
  {
    sample.gyroscope.y = (float)((int16_t)(buffer[buffer_idx] << 8 | buffer[buffer_idx + 1])) / gyroscope_sensitivity;
    buffer_idx += 2;
  }
  else
  {
    sample.gyroscope.y = 0;
  }
  if (((gyro_fifo_en >> FIFO_GYRO_Z_EN_BIT) & 0x01) && (buffer_idx + 2 <= _fifo_data_block_size))
  {
    sample.gyroscope.z = (float)((int16_t)(buffer[buffer_idx] << 8 | buffer[buffer_idx + 1])) / gyroscope_sensitivity;
    buffer_idx += 2;
  }
  else
  {
    sample.gyroscope.z = 0;
  }

  // Temperature data
  if (((readRegister(MPU6050_FIFO_EN) >> TEMP_FIFO_EN_BIT) & 0x01) && (buffer_idx + 2 <= _fifo_data_block_size))
  {
    int16_t raw_temp = (buffer[buffer_idx] << 8 | buffer[buffer_idx + 1]);
    sample.temperature_celsius = (float)raw_temp / TEMP_SENSITIVITY_LSB_PER_DEGREE + TEMP_OFFSET_DEGREES_CELSIUS;
    buffer_idx += 2;
  }
  else
  {
    sample.temperature_celsius = 0;
  }

  return true;
}

bool ESP32_MPU6050::getFifoSamples(SensorReadings *samples_array, uint16_t max_samples_to_read, uint16_t &actual_samples_read)
{
  if (_fifo_data_block_size == 0)
  {
    actual_samples_read = 0;
    return false; // FIFO not configured or empty
  }

  uint16_t fifo_bytes_available = getFifoCount();
  uint16_t samples_available = fifo_bytes_available / _fifo_data_block_size;
  actual_samples_read = 0;

  uint16_t samples_to_read = min(max_samples_to_read, samples_available);

  for (uint16_t i = 0; i < samples_to_read; ++i)
  {
    if (getFifoSample(samples_array[i]))
    {
      actual_samples_read++;
    }
    else
    {
      // If reading a single sample fails, stop and return false
      return false;
    }
  }
  return true;
}

bool ESP32_MPU6050::isFifoOverflowed()
{
  uint8_t int_status = readRegister(MPU6050_INT_STATUS);
  return (int_status >> FIFO_OFLOW_INT_BIT) & 0x01;
}

bool ESP32_MPU6050::readSensorData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
  SensorReadings current_sample;
  if (!getFifoSample(current_sample))
  {
    return false;
  }

  // Convert float readings back to int16_t raw values for compatibility with existing calibration and update logic
  // This is a simplification; ideally, calibration and update would work directly with float SensorReadings.
  *ax = (int16_t)(current_sample.accelerometer.x * accelerometer_sensitivity);
  *ay = (int16_t)(current_sample.accelerometer.y * accelerometer_sensitivity);
  *az = (int16_t)(current_sample.accelerometer.z * accelerometer_sensitivity);
  *gx = (int16_t)(current_sample.gyroscope.x * gyroscope_sensitivity);
  *gy = (int16_t)(current_sample.gyroscope.y * gyroscope_sensitivity);
  *gz = (int16_t)(current_sample.gyroscope.z * gyroscope_sensitivity);

  return true;
}
