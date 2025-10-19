#include <Arduino.h>
#include "ESP32_MPU6050.h"

ESP32_MPU6050::ESP32_MPU6050(int8_t address) : i2cAddress(address)
{
  gyroscope_offset = {0, 0, 0};
  accelerometer_offset = {0, 0, 0};
}

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

  // Set the full-scale ranges for gyroscope and accelerometer.
  if (!setGyroscopeRange(gyroRange))
    return false;
  if (!setAccelerometerRange(accelRange))
    return false;

  Serial.print("DEBUG: Gyro Sensitivity: ");
  Serial.println(gyroscope_sensitivity, 6);
  Serial.print("DEBUG: Accel Sensitivity: ");
  Serial.println(accelerometer_sensitivity, 6);

  // Set the Digital Low Pass Filter (DLPF) bandwidth.
  if (!setLpfBandwidth(lpfBandwidth))
    return false;

  // Initialize the FIFO buffer
  initFIFO();

  return true;
}

void ESP32_MPU6050::initFIFO()
{
  // Reset and disable FIFO
  writeRegister(MPU6050_USER_CTRL, 1 << MPU6050_USER_CTRL_FIFO_RESET_BIT);
  // Enable Gyro and Accel to be written to FIFO
  writeRegister(MPU6050_FIFO_EN, (1 << MPU6050_FIFO_EN_ACCEL_BIT) | (1 << MPU6050_FIFO_EN_GYRO_X_BIT) | (1 << MPU6050_FIFO_EN_GYRO_Y_BIT) | (1 << MPU6050_FIFO_EN_GYRO_Z_BIT));
  // Enable FIFO
  writeRegister(MPU6050_USER_CTRL, 1 << MPU6050_USER_CTRL_FIFO_EN_BIT);
}

bool ESP32_MPU6050::setLpfBandwidth(LpfBandwidth bandwidth)
{
  // The bandwidth is set by writing the enum value directly to the MPU6050_CONFIG register.
  return writeRegister(MPU6050_CONFIG, bandwidth);
}

bool ESP32_MPU6050::setGyroscopeRange(GyroRange range)
{
  Serial.print("DEBUG: setGyroscopeRange received range: ");
  Serial.println(range);
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
  Serial.print("DEBUG: gyroscope_sensitivity set to: ");
  Serial.println(gyroscope_sensitivity, 6);
  // The range is set by shifting the enum value by 3 bits to the left (MPU6050 datasheet).
  return writeRegister(MPU6050_GYRO_CONFIG, range << 3);
}

bool ESP32_MPU6050::setAccelerometerRange(AccelRange range)
{
  Serial.print("DEBUG: setAccelerometerRange received range: ");
  Serial.println(range);
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
  Serial.print("DEBUG: accelerometer_sensitivity set to: ");
  Serial.println(accelerometer_sensitivity, 6);
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
    uint8_t buffer[14]; // 6 bytes for accel, 6 bytes for gyro, 2 for temp

    // Read all 14 bytes (Accel, Temp, Gyro) directly from registers
    if (!readRegisters(MPU6050_ACCEL_XOUT_H, 14, buffer))
    {
      Serial.println("ERROR: Failed to read raw sensor registers during calibration.");
      continue; // Skip this sample if read fails
    }

    // Combine the high and low bytes to get the raw 16-bit sensor values.
    int16_t raw_ax = (buffer[0] << 8) | buffer[1];
    int16_t raw_ay = (buffer[2] << 8) | buffer[3];
    int16_t raw_az = (buffer[4] << 8) | buffer[5];
    // int16_t raw_temp = (buffer[6] << 8) | buffer[7]; // Temperature not used for offset calibration
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
  uint8_t buffer[14]; // 6 bytes for accel, 6 bytes for gyro, 2 for temp

  // Read all 14 bytes (Accel, Temp, Gyro) directly from registers
  if (!readRegisters(MPU6050_ACCEL_XOUT_H, 14, buffer))
  {
    Serial.println("ERROR: Failed to read raw sensor registers.");
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
  readings.accelerometer.x = ((float)raw_ax - accelerometer_offset.x) / accelerometer_sensitivity;
  readings.accelerometer.y = ((float)raw_ay - accelerometer_offset.y) / accelerometer_sensitivity;
  readings.accelerometer.z = ((float)raw_az - accelerometer_offset.z) / accelerometer_sensitivity;

  readings.gyroscope.x = ((float)raw_gx - gyroscope_offset.x) / gyroscope_sensitivity;
  readings.gyroscope.y = ((float)raw_gy - gyroscope_offset.y) / gyroscope_sensitivity;
  readings.gyroscope.z = ((float)raw_gz - gyroscope_offset.z) / gyroscope_sensitivity;

  readings.temperature_celsius = (raw_temp / 340.0) + 36.53; // MPU6050 datasheet formula

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