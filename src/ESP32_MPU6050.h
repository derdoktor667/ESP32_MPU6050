#pragma once

#include <Wire.h>
#include "MPU6050_DMP_Firmware.h"
#include "helper_3dmath.h"

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
#define MPU6050_INT_STATUS 0x3A
#define FIFO_OFLOW_INT_BIT 4

// DMP-related registers (from MPU6050_6Axis_MotionApps_V6_12.h)
#define MPU6050_RA_DMP_CFG_1 0x70
#define MPU6050_RA_DMP_CFG_2 0x71
#define MPU6050_RA_DMP_INT_STATUS 0x36
#define MPU6050_RA_MOT_DETECT_CTRL 0x69
#define MPU6050_RA_INT_ENABLE 0x38 // Corrected definition for INT_ENABLE
#define MPU6050_INT_PIN_CFG 0x37   // Corrected definition for INT_PIN_CFG

// PWR_MGMT_1 bits (for MPU6050_PWR_MGMT_1 register)
#define MPU6050_PWR1_DEVICE_RESET_BIT 7

// DMP_INT_ENABLE bits (for MPU6050_INT_ENABLE register)
#define MPU6050_INTERRUPT_DMP_INT_BIT 1

// USER_CTRL bits (for MPU6050_USER_CTRL register)
#define MPU6050_USERCTRL_DMP_EN_BIT 7
#define MPU6050_USERCTRL_DMP_RESET_BIT 3

// PWR_MGMT_1 bits (for MPU6050_PWR_MGMT_1 register)
#define MPU6050_PWR1_DEVICE_RESET_BIT 7

// ACCEL_CONFIG bits (for FCHOICE_B, used by DMP)
#define MPU6050_ACONFIG_AFS_SEL_BIT 4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH 2

// MPU6050_RA_INT_PIN_CFG bits (for MPU6050_INT_PIN_CFG register, which is 0x37)
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INTCFG_INTLEVEL_BIT 7
#define MPU6050_INTCFG_INTOPEN_BIT 6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT 5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT 4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT 3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT 2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT 1
#define MPU6050_INTCFG_CLKOUT_EN_BIT 0

// MPU6050_RA_DMP_CFG_1 values
#define MPU6050_DMPCFG_EXT_SYNC_TEMP_OUT 0x00
#define MPU6050_DMPCFG_EXT_SYNC_GYRO_X 0x01
#define MPU6050_DMPCFG_EXT_SYNC_GYRO_Y 0x02
#define MPU6050_DMPCFG_EXT_SYNC_GYRO_Z 0x03
#define MPU6050_DMPCFG_EXT_SYNC_ACCEL_X 0x04
#define MPU6050_DMPCFG_EXT_SYNC_ACCEL_Y 0x05
#define MPU6050_DMPCFG_EXT_SYNC_ACCEL_Z 0x06
#define MPU6050_DMPCFG_EXT_SYNC_NONE 0x07

// MPU6050_RA_DMP_CFG_2 values
#define MPU6050_DMPCFG_FIFO_MODE_STOP_ON_FULL 0x00
#define MPU6050_DMPCFG_FIFO_MODE_OVERWRITE 0x01
#define MPU6050_DMPCFG_LP_WAKE_CTRL_1_25HZ 0x00
#define MPU6050_DMPCFG_LP_WAKE_CTRL_5HZ 0x01
#define MPU6050_DMPCFG_LP_WAKE_CTRL_20HZ 0x02
#define MPU6050_DMPCFG_LP_WAKE_CTRL_40HZ 0x03

// MPU6050_RA_MOT_DETECT_CTRL values
#define MPU6050_MOTCTRL_ACCEL_HPF_RESET 0x00
#define MPU6050_MOTCTRL_ACCEL_HPF_5HZ 0x01
#define MPU6050_MOTCTRL_ACCEL_HPF_2_5HZ 0x02
#define MPU6050_MOTCTRL_ACCEL_HPF_1_25HZ 0x03
#define MPU6050_MOTCTRL_ACCEL_HPF_0_625HZ 0x04

// MPU6050_RA_INT_ENABLE values
#define MPU6050_INTERRUPT_DMP_INT_EN 0x02

// MPU6050_RA_USER_CTRL values
#define MPU6050_USERCTRL_DMP_EN 0x80
#define MPU6050_USERCTRL_FIFO_EN 0x40
#define MPU6050_USERCTRL_I2C_MST_EN 0x20
#define MPU6050_USERCTRL_I2C_IF_DIS 0x10
#define MPU6050_USERCTRL_DMP_RESET 0x08
#define MPU6050_USERCTRL_SIG_COND_RESET 0x01

// MPU6050_RA_PWR_MGMT_1 values
#define MPU6050_PWR1_DEVICE_RESET 0x80
#define MPU6050_PWR1_SLEEP 0x40
#define MPU6050_PWR1_CYCLE 0x20
#define MPU6050_PWR1_TEMP_DIS 0x08
#define MPU6050_PWR1_CLKSEL_INTERNAL 0x00
#define MPU6050_PWR1_CLKSEL_XGYRO 0x01
#define MPU6050_PWR1_CLKSEL_YGYRO 0x02
#define MPU6050_PWR1_CLKSEL_ZGYRO 0x03
#define MPU6050_PWR1_CLKSEL_EXT32K 0x04
#define MPU6050_PWR1_CLKSEL_EXT19M 0x05

// MPU6050_RA_ACCEL_CONFIG values (for FCHOICE_B)
#define MPU6050_ACONFIG_AFS_SEL_2G 0x00
#define MPU6050_ACONFIG_AFS_SEL_4G 0x08
#define MPU6050_ACONFIG_AFS_SEL_8G 0x10
#define MPU6050_ACONFIG_AFS_SEL_16G 0x18

// FIFO Enable Register
#define MPU6050_FIFO_EN 0x23
#define FIFO_ACCEL_EN_BIT 3
#define FIFO_GYRO_X_EN_BIT 6
#define FIFO_GYRO_Y_EN_BIT 5
#define FIFO_GYRO_Z_EN_BIT 4
#define TEMP_FIFO_EN_BIT 7

// User Control Register Bits
#define MPU6050_USER_CTRL_FIFO_EN_BIT 6
#define MPU6050_USER_CTRL_FIFO_RESET_BIT 2

// Gyro and Accel Config Register Bits
#define GYRO_CONFIG_SHIFT 3
#define ACCEL_CONFIG_SHIFT 3

// MPU6050 Power Management 1 Register values
#define MPU6050_PWR_MGMT_1_WAKE 0x00

// MPU6050 Data Block Size
#define MPU6050_DATA_BLOCK_SIZE 14
#define MPU6050_DMP_PACKET_SIZE 28

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
#define TEMP_OFFSET_DEGREES_CELSIUS 16.53f

// Calibration Delay
#define CALIBRATION_DELAY_MS 2

// DMP Reset Delay
#define DMP_RESET_DELAY_MS 100

// DMP Initialization Constants
#define DMP_INT_ENABLE_NONE 0x00
#define DMP_FIFO_EN_NONE 0x00
#define DMP_ACCEL_CONFIG_2G 0x00
#define DMP_INT_PIN_CFG_ACTIVE_LOW_CLEAR_ON_READ 0x80
#define DMP_SMPLRT_DIV_100HZ 0x09
#define DMP_CONFIG_DLPF_188HZ 0x01
#define DMP_CFG_1_START_ADDRESS 0x0400
#define DMP_GYRO_CONFIG_2000DPS 0x18
#define DMP_USER_CTRL_FIFO_EN_RESET 0xC0
#define DMP_INT_ENABLE_RAW_DMP_INT_EN 0x02
#define DMP_SIGNAL_PATH_RESET_BITS 3
#define DMP_SIGNAL_PATH_RESET_VALUE 0b111

// DMP Gravity and Quaternion Scaling
#define DMP_1G_ACCEL_VALUE 8192
#define DMP_QUATERNION_SCALE 16384

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

// Struct for FIFO configuration
struct FifoConfig
{
  bool enable_accel = true;
  bool enable_gyro_x = true;
  bool enable_gyro_y = true;
  bool enable_gyro_z = true;
  bool enable_temp = false;
};

class ESP32_MPU6050
{
public:
  // The processed sensor data is stored here.
  SensorReadings readings;

  ESP32_MPU6050(int8_t address = MPU6050_ADDR);

  bool begin(GyroRange gyroRange = GYRO_RANGE_2000DPS, AccelRange accelRange = ACCEL_RANGE_16G, LpfBandwidth lpfBandwidth = LPF_256HZ_N_0MS, uint32_t i2cClockSpeed = 1000000, uint8_t expected_who_am_i = 0x70);
  bool setGyroscopeRange(GyroRange range);
  bool setAccelerometerRange(AccelRange range);
  bool setLpfBandwidth(LpfBandwidth bandwidth);
  void calibrate(int num_samples = 1000);
  bool update();
  void resetFifo();
  bool configureFifo(const FifoConfig &config);
  uint16_t getFifoCount();
  bool getFifoSample(SensorReadings &sample);
  bool getFifoSamples(SensorReadings samples[], uint16_t max_samples, uint16_t &actual_samples_read);
  bool readDirectSensorData(SensorReadings &sample);
  bool isFifoOverflowed();
  uint8_t getSensorFifoPacketSize() const { return _fifo_data_block_size; }

  // DMP methods
  uint8_t dmpInitialize();
  bool dmpPacketAvailable();
  uint16_t dmpGetFIFOPacketSize();
  uint8_t dmpGetCurrentFIFOPacket(uint8_t *data);
  uint8_t dmpGetQuaternion(int32_t *data, const uint8_t *packet = 0);
  uint8_t dmpGetQuaternion(int16_t *data, const uint8_t *packet = 0);
  uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t *packet = 0);
  uint8_t dmpGetAccel(int32_t *data, const uint8_t *packet = 0);
  uint8_t dmpGetAccel(int16_t *data, const uint8_t *packet = 0);
  uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t *packet = 0);
  uint8_t dmpGetGyro(int32_t *data, const uint8_t *packet = 0);
  uint8_t dmpGetGyro(int16_t *data, const uint8_t *packet = 0);
  uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t *packet = 0);
  uint8_t dmpGetGravity(int16_t *data, const uint8_t *packet = 0);
  uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
  uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
  uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
  uint8_t dmpGetEuler(float *data, Quaternion *q);
  uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

  // Getter methods for sensor settings
  GyroRange getGyroscopeRange() const { return _gyro_range; }
  AccelRange getAccelerometerRange() const { return _accel_range; }
  LpfBandwidth getLpfBandwidth() const { return _lpf_bandwidth; }

  // Getter and Setter for Offsets
  AxisData getGyroscopeOffset() const { return gyroscope_offset; }
  void setGyroscopeOffset(const AxisData &offset) { gyroscope_offset = offset; }
  AxisData getAccelerometerOffset() const { return accelerometer_offset; }
  void setAccelerometerOffset(const AxisData &offset) { accelerometer_offset = offset; }

  // Getter for Raw Sensor Data
  void getRawReadings(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t reg, uint8_t count, uint8_t *dest);
  uint8_t readRegister(uint8_t reg);

  int8_t i2cAddress;
  float gyroscope_sensitivity;
  float accelerometer_sensitivity;
  AxisData gyroscope_offset;
  AxisData accelerometer_offset;

  // Current sensor settings
  GyroRange _gyro_range;
  AccelRange _accel_range;
  LpfBandwidth _lpf_bandwidth;
  FifoConfig _fifo_config;   // Store the FIFO configuration
  uint32_t _i2c_clock_speed; // Store the I2C clock speed

  // Raw sensor data
  int16_t _raw_ax, _raw_ay, _raw_az, _raw_gx, _raw_gy, _raw_gz;
  uint8_t _fifo_data_block_size;
  uint8_t dmpPacketBuffer[MPU6050_DMP_PACKET_SIZE]; // DMP FIFO packet size is 28 bytes
  uint16_t dmpPacketSize;

  // Private Helper Functions
  bool writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
  bool writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
  bool writeBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
  bool writeWord(uint8_t regAddr, uint16_t data);
  bool writeProgMemoryBlock(const uint8_t *data, uint16_t length);
  bool isDMPEnabled();
};
