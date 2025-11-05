#include "ESP32_MPU6050.h"

// Create an instance of the sensor library.
ESP32_MPU6050 sensor;

void setup()
{
  // Start serial communication for debugging.
  Serial.begin(115200);

  // Initialize the sensor.
  Serial.println("Initializing MPU6050...");
  // Optimal settings for a quadcopter flight controller:
  // Gyro Range: GYRO_RANGE_2000DPS (highest resolution for fast rotations)
  // Accel Range: ACCEL_RANGE_16G (highest resolution for strong accelerations)
  // LPF Bandwidth: LPF_188HZ_N_2MS (good balance between noise reduction and latency)
  if (!sensor.begin(GYRO_RANGE_2000DPS, ACCEL_RANGE_16G, LPF_188HZ_N_2MS))
  {
    Serial.println("Failed to initialize MPU6050! Check wiring.");
    // Loop forever if initialization fails.
    while (1)
      ;
  }

  // Calibrate the sensor. It is important to keep the sensor still during this process.
  Serial.println("Calibrating sensor... Freeze!");
  sensor.calibrate(1000);
  Serial.println("Calibration complete.");
}

void loop()
{
  // Read the sensor data.
  if (sensor.update())
  {
    // Print accelerometer data in g's.
    Serial.print("Accel(g): ");
    Serial.print("X=");
    Serial.print(sensor.readings.accelerometer.x, 3);
    Serial.print(", Y=");
    Serial.print(sensor.readings.accelerometer.y, 3);
    Serial.print(", Z=");
    Serial.print(sensor.readings.accelerometer.z, 3);

    // Print gyroscope data in degrees per second.
    Serial.print("  |  Gyro(dps): ");
    Serial.print("X=");
    Serial.print(sensor.readings.gyroscope.x, 2);
    Serial.print(", Y=");
    Serial.print(sensor.readings.gyroscope.y, 2);
    Serial.print(", Z=");
    Serial.print(sensor.readings.gyroscope.z, 2);

    // Print temperature in degrees Celsius.
    Serial.print("  |  Temp(C)=");
    Serial.println(sensor.readings.temperature_celsius, 2);
  }
  else
  {
    Serial.println("Failed to read data from MPU6050!");
  }

  // Wait a bit before the next reading.
  delay(100);
}
