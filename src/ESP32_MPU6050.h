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
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_FIFO_COUNTH 0x72
#define MPU6050_FIFO_R_W 0x74
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_WHO_AM_I_EXPECTED_VALUE 0x68

// FIFO Enable Register
#define MPU6050_FIFO_EN 0x23
#define MPU6050_FIFO_EN_ACCEL_BIT 3
#define MPU6050_FIFO_EN_GYRO_X_BIT 6
#define MPU6050_FIFO_EN_GYRO_Y_BIT 5
#define MPU6050_FIFO_EN_GYRO_Z_BIT 4

// User Control Register Bits
#define MPU6050_USER_CTRL_FIFO_EN_BIT 6
#define MPU6050_USER_CTRL_FIFO_RESET_BIT 2

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

// Enums for Digital Low Pass Filter (DLPF) bandwidths
enum LpfBandwidth : uint8_t
{
  LPF_256HZ_N_0MS = 0, // 256 Hz, 0ms delay
  LPF_188HZ_N_2MS = 1, // 188 Hz, 2ms delay
  LPF_98HZ_N_3MS = 2,  // 98 Hz, 3ms delay
  LPF_42HZ_N_5MS = 3,  // 42 Hz, 5ms delay
  LPF_20HZ_N_10MS = 4, // 20 Hz, 10ms delay
  LPF_10HZ_N_13MS = 5, // 10 Hz, 13ms delay
  LPF_5HZ_N_18MS = 6   // 5 Hz, 18ms delay
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

  bool begin(GyroRange gyroRange = GYRO_RANGE_2000DPS, AccelRange accelRange = ACCEL_RANGE_16G, LpfBandwidth lpfBandwidth = LPF_188HZ_N_2MS);
  bool setGyroscopeRange(GyroRange range);
  bool setAccelerometerRange(AccelRange range);
  bool setLpfBandwidth(LpfBandwidth bandwidth);
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

  // Private Helper Functions
  void resetFifo();
  uint16_t getFifoCount();
};
