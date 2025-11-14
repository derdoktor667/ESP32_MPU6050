#include "ESP32_MPU6050.h"

// Create an instance of the sensor library.
ESP32_MPU6050 sensor;

void setup()
{
  // Start serial communication for debugging.
  Serial.begin(115200);

  // Initialize the sensor.
  Serial.println("Initializing MPU6050...");
  // The default settings are optimized for low latency, making them suitable for most applications.
  // Gyro Range: GYRO_RANGE_2000DPS
  // Accel Range: ACCEL_RANGE_16G
  // LPF Bandwidth: LPF_256HZ_N_0MS
  // I2C Clock Speed: 1000000 (1MHz) - default
  // If your MPU6050 has a different WHO_AM_I value, you can pass it as the last parameter.
  // For example, to set a 100kHz I2C speed and a custom WHO_AM_I (e.g., 0x68), you would call:
  // if (!sensor.begin(GYRO_RANGE_2000DPS, ACCEL_RANGE_16G, LPF_256HZ_N_0MS, 100000, 0x68))
  if (!sensor.begin())
  {
    Serial.println("Failed to initialize MPU6050! Check wiring.");
    // Loop forever if initialization fails.
    while (1)
      ;
  }

  // Calibrate the sensor. It is important to keep the sensor still during this process.
  Serial.println("Calibrating sensor... Keep it still!");
  sensor.calibrate(1000);
  Serial.println("Calibration complete.");

  // Demonstrate getting the offsets
  AxisData gyroOffset = sensor.getGyroscopeOffset();
  AxisData accelOffset = sensor.getAccelerometerOffset();

  Serial.println("Calculated Offsets:");
  Serial.print("Gyro Offset: X=");
  Serial.print(gyroOffset.x);
  Serial.print(", Y=");
  Serial.print(gyroOffset.y);
  Serial.print(", Z=");
  Serial.println(gyroOffset.z);

  Serial.print("Accel Offset: X=");
  Serial.print(accelOffset.x);
  Serial.print(", Y=");
  Serial.print(accelOffset.y);
  Serial.print(", Z=");
  Serial.println(accelOffset.z);

  // You can also set the offsets manually, for example, after loading them from NVS
  // sensor.setGyroscopeOffset({-1.2, 3.4, -0.5});
  // sensor.setAccelerometerOffset({-0.02, 0.01, -0.03});
}

void loop()
{
  // Read the sensor data.
  if (sensor.update())
  {
    // Print scaled sensor data.
    Serial.print("Scaled -> ");
    Serial.print("Accel(g): ");
    Serial.print("X=");
    Serial.print(sensor.readings.accelerometer.x, 3);
    Serial.print(", Y=");
    Serial.print(sensor.readings.accelerometer.y, 3);
    Serial.print(", Z=");
    Serial.print(sensor.readings.accelerometer.z, 3);

    Serial.print("  |  Gyro(dps): ");
    Serial.print("X=");
    Serial.print(sensor.readings.gyroscope.x, 2);
    Serial.print(", Y=");
    Serial.print(sensor.readings.gyroscope.y, 2);
    Serial.print(", Z=");
    Serial.println(sensor.readings.gyroscope.z, 2);


  }
  else
  {
    Serial.println("Failed to read data from MPU6050!");
  }

  // The FIFO buffer can be reset if needed, for example, to clear old data.
  // sensor.resetFifo();

  // Wait a bit before the next reading.

}
