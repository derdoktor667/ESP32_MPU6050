#include "ESP32_MPU6050.h"

ESP32_MPU6050::ESP32_MPU6050(int8_t address) : i2cAddress(address)
{
  gyroscope_offset = {0, 0, 0};
  accelerometer_offset = {0, 0, 0};
}

bool ESP32_MPU6050::begin(GyroRange gyroRange, AccelRange accelRange)
{
  Wire.begin();

  // Verify sensor connection by checking the WHO_AM_I register.
  if (readRegister(MPU6050_WHO_AM_I) != MPU6050_ADDR)
  {
    return false;
  }

  // Wake up the sensor by writing 0 to the power management register.
  if (!writeRegister(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_WAKE))
  {
    return false;
  }

  // Set the full-scale ranges for gyroscope and accelerometer.
  if (!setGyroscopeRange(gyroRange))
    return false;
  if (!setAccelerometerRange(accelRange))
    return false;

  return true;
}

bool ESP32_MPU6050::setGyroscopeRange(GyroRange range)
{
  // The sensitivity values are from the MPU6050 datasheet.
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
  // The range is set by shifting the enum value by 3 bits to the left (MPU6050 datasheet).
  return writeRegister(MPU6050_GYRO_CONFIG, range << 3);
}

bool ESP32_MPU6050::setAccelerometerRange(AccelRange range)
{
  // The sensitivity values are from the MPU6050 datasheet.
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
  // The range is set by shifting the enum value by 3 bits to the left (MPU6050 datasheet).
  return writeRegister(MPU6050_ACCEL_CONFIG, range << 3);
}

void ESP32_MPU6050::calibrate(int num_samples)
{
  long gyro_x_sum = 0;
  long gyro_y_sum = 0;
  long gyro_z_sum = 0;
  long accel_x_sum = 0;
  long accel_y_sum = 0;
  long accel_z_sum = 0;

  // Read sensor data multiple times and sum the raw values.
  for (int i = 0; i < num_samples; ++i)
  {
    uint8_t buffer[MPU6050_DATA_BLOCK_SIZE];
    if (readRegisters(MPU6050_ACCEL_XOUT_H, MPU6050_DATA_BLOCK_SIZE, buffer))
    {
      // Combine high and low bytes to get 16-bit raw values.
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
    }
    delay(CALIBRATION_DELAY_MS);
  }

  // Calculate the average of the raw values to get the offset.
  gyroscope_offset.x = (float)gyro_x_sum / num_samples;
  gyroscope_offset.y = (float)gyro_y_sum / num_samples;
  gyroscope_offset.z = (float)gyro_z_sum / num_samples;

  accelerometer_offset.x = (float)accel_x_sum / num_samples;
  accelerometer_offset.y = (float)accel_y_sum / num_samples;
  // Z-axis offset is calculated like X and Y. The 1g of gravity should be handled by the application (e.g., flight controller).
  accelerometer_offset.z = (float)accel_z_sum / num_samples;
}

bool ESP32_MPU6050::update()
{
  uint8_t buffer[MPU6050_DATA_BLOCK_SIZE];
  // Read 14 bytes from the sensor, starting from the accelerometer output registers.
  if (!readRegisters(MPU6050_ACCEL_XOUT_H, MPU6050_DATA_BLOCK_SIZE, buffer))
  {
    return false;
  }

  // Combine the high and low bytes to get the raw 16-bit sensor values.
  int16_t raw_ax = (buffer[0] << 8) | buffer[1];
  int16_t raw_ay = (buffer[2] << 8) | buffer[3];
  int16_t raw_az = (buffer[4] << 8) | buffer[5];
  int16_t raw_temp = (buffer[6] << 8) | buffer[7];
  int16_t raw_gx = (buffer[8] << 8) | buffer[9];
  int16_t raw_gy = (buffer[10] << 8) | buffer[11];
  int16_t raw_gz = (buffer[12] << 8) | buffer[13];

  // Subtract the offset and divide by the sensitivity to get physical units.
  readings.accelerometer.x = (raw_ax - accelerometer_offset.x) / accelerometer_sensitivity;
  readings.accelerometer.y = (raw_ay - accelerometer_offset.y) / accelerometer_sensitivity;
  readings.accelerometer.z = (raw_az - accelerometer_offset.z) / accelerometer_sensitivity;

  readings.gyroscope.x = (raw_gx - gyroscope_offset.x) / gyroscope_sensitivity;
  readings.gyroscope.y = (raw_gy - gyroscope_offset.y) / gyroscope_sensitivity;
  readings.gyroscope.z = (raw_gz - gyroscope_offset.z) / gyroscope_sensitivity;

  // Convert temperature to Celsius using the formula from the datasheet.
  readings.temperature_celsius = (raw_temp / TEMP_SENSITIVITY_LSB_PER_DEGREE) + TEMP_OFFSET_DEGREES_CELSIUS;

  return true;
}

bool ESP32_MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.write(value);
  // endTransmission returns 0 on success.
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
  // endTransmission(false) keeps the connection active for a repeated start.
  if (Wire.endTransmission(false) != I2C_TRANSMISSION_SUCCESS)
  {
    return false; // Sensor not connected.
  }

  // Check if the requested number of bytes were received.
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
