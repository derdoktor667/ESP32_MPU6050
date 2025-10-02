#pragma once

#include <Wire.h>

// MPU6050 Register Addresses
#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_WHO_AM_I_EXPECTED_VALUE 0x70

// MPU6050 Power Management 1 Register values
#define MPU6050_PWR_MGMT_1_WAKE 0x00

// MPU6050 Data Block Size
#define MPU6050_DATA_BLOCK_SIZE 14

// Gyroscope Sensitivity (LSB/dps) from MPU6050 Datasheet
#define GYRO_SENSITIVITY_250DPS 131.0f
#define GYRO_SENSITIVITY_500DPS 65.5f
#define GYRO_SENSITIVITY_1000DPS 32.8f
#define GYRO_SENSITIVITY_2000DPS 16.4f

// Accelerometer Sensitivity (LSB/g) from MPU6050 Datasheet
#define ACCEL_SENSITIVITY_2G 16384.0f
#define ACCEL_SENSITIVITY_4G 8192.0f
#define ACCEL_SENSITIVITY_8G 4096.0f
#define ACCEL_SENSITIVITY_16G 2048.0f

// Temperature Conversion Constants from MPU6050 Datasheet
#define TEMP_SENSITIVITY_LSB_PER_DEGREE 340.0f
#define TEMP_OFFSET_DEGREES_CELSIUS 36.53f

// Calibration Delay
#define CALIBRATION_DELAY_MS 2

// I2C Success Code
#define I2C_TRANSMISSION_SUCCESS 0

// Enums for sensor ranges
enum GyroRange
{
  GYRO_RANGE_250DPS,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
};

enum AccelRange
{
  ACCEL_RANGE_2G,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_16G
};

// Struct for 3-axis floating point data.
struct AxisData
{
  float x, y, z;
};

// Struct to hold all processed sensor data.
struct SensorReadings
{
  AxisData accelerometer;
  AxisData gyroscope;
  float temperature_celsius;
};

class ESP32_MPU6050
{
public:
  // The processed sensor data is stored here.
  SensorReadings readings;

  ESP32_MPU6050(int8_t address = MPU6050_ADDR);

  bool begin(GyroRange gyroRange = GYRO_RANGE_250DPS, AccelRange accelRange = ACCEL_RANGE_2G);
  bool setGyroscopeRange(GyroRange range);
  bool setAccelerometerRange(AccelRange range);
  void calibrate(int num_samples = 1000);
  bool update();

private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t reg, uint8_t count, uint8_t *dest);
  uint8_t readRegister(uint8_t reg);

  int8_t i2cAddress;
  float gyroscope_sensitivity;
  float accelerometer_sensitivity;
  AxisData gyroscope_offset;
  AxisData accelerometer_offset;
};
