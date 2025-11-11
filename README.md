# ESP32_MPU6050 Arduino Library

This is an Arduino library for the MPU6050 gyroscope and accelerometer sensor, specifically optimized for use with the ESP32 in projects like quadcopters.

It provides a clean interface to read calibrated sensor data in standard physical units (g's and degrees/sec).

## Features

*   Easy initialization and configuration.
*   Built-in sensor calibration.
*   Access to both scaled and raw sensor data.
*   Getters and setters for sensor offsets.
*   Public access to reset the FIFO buffer.
*   Data is returned in a single struct for atomic access.
*   Output is in physical units (g for acceleration, dps for gyroscope).
*   Configurable accelerometer and gyroscope ranges.
*   **Configurable Digital Low Pass Filter (DLPF) bandwidth.**

## Installation

1.  [Download](https://github.com/derdoktor667/ESP32_MPU6050/archive/refs/heads/main.zip) the latest version of this library.
2.  In the Arduino IDE, go to `Sketch` > `Include Library` > `Add .ZIP Library...`
3.  Select the downloaded ZIP file.

## Usage

The library is designed to be easy to use while providing powerful features.

First, include the library and create an `ESP32_MPU6050` object:

```cpp
#include "ESP32_MPU6050.h"

ESP32_MPU6050 sensor;
```

In your `setup()` function, initialize the sensor. You can specify the desired gyroscope, accelerometer ranges, the Digital Low Pass Filter (DLPF) bandwidth, and the expected WHO_AM_I value. Then, run the calibration routine. **The sensor must be kept still during calibration.**



```cpp

void setup() {

  Serial.begin(115200);



  // Initialize the sensor with default settings

  // Gyro Range: GYRO_RANGE_2000DPS

  // Accel Range: ACCEL_RANGE_16G

  // LPF Bandwidth: LPF_256HZ_N_0MS

    // WHO_AM_I: 0x70 (Default)

    // I2C Clock Speed: 1000000 (1MHz) (Default)

    if (!sensor.begin()) {

      Serial.println("Failed to initialize MPU6050! Check wiring.");

      while (1);

    }



  // Calibrate the sensor. Do not move it during this process.

  Serial.println("Calibrating sensor... Hold it.");

  sensor.calibrate();

  Serial.println("Calibration complete.");

}

```



### Digital Low Pass Filter (DLPF) Configuration







The MPU6050 features a Digital Low Pass Filter (DLPF) to reduce noise from high-frequency vibrations. This is particularly important for applications like quadcopters.







The `begin()` method now accepts an optional `LpfBandwidth` parameter. The available options are:







*   `LPF_256HZ_N_0MS` (256 Hz, 0ms delay) - Minimal filtering, highest bandwidth (default).



*   `LPF_188HZ_N_2MS` (188 Hz, 2ms delay)



*   `LPF_98HZ_N_3MS` (98 Hz, 3ms delay)



*   `LPF_42HZ_N_5MS` (42 Hz, 5ms delay) - **Recommended for quadcopters.**



*   `LPF_20HZ_N_10MS` (20 Hz, 10ms delay)



*   `LPF_10HZ_N_13MS` (10 Hz, 13ms delay)



*   `LPF_5HZ_N_18MS` (5 Hz, 18ms delay) - Strongest filtering, highest latency.







**Recommendation for Quadcopters:**



For quadcopter flight controllers, `LPF_42HZ_N_5MS` is generally a good starting point. It provides a good balance between filtering out motor-induced vibrations and maintaining low latency for responsive control.







### I2C Clock Speed Configuration







The `begin()` method also accepts an optional `i2cClockSpeed` parameter to set the I2C communication speed. The default speed is 1,000,000 Hz (1 MHz), which is the maximum supported by the ESP32. You can set it to other common speeds like 400,000 Hz (400 kHz) or 100,000 Hz (100 kHz) if needed.







Example of setting I2C clock speed to 400 kHz:



```cpp



if (!sensor.begin(GYRO_RANGE_2000DPS, ACCEL_RANGE_16G, LPF_256HZ_N_0MS, 400000)) {



  // Handle error



}



```







### Advanced Usage



The library provides additional functions for more advanced use cases, such as getting and setting offsets, and accessing raw sensor data.



```cpp

void advanced_setup() {

  // ... initialization ...



  // Get the calculated offsets

  AxisData gyroOffset = sensor.getGyroscopeOffset();

  AxisData accelOffset = sensor.getAccelerometerOffset();



  // You can also set the offsets manually, for example, after loading them from NVS

  // sensor.setGyroscopeOffset({-1.2, 3.4, -0.5});

  // sensor.setAccelerometerOffset({-0.02, 0.01, -0.03});

}



void loop() {

  if (sensor.update()) {

    // Access scaled accelerometer data in g's

    float accelX = sensor.readings.accelerometer.x;

    float accelY = sensor.readings.accelerometer.y;

    float accelZ = sensor.readings.accelerometer.z;



    // Access scaled gyroscope data in degrees/sec

    float gyroX = sensor.readings.gyroscope.x;

    float gyroY = sensor.readings.gyroscope.y;

    float gyroZ = sensor.readings.gyroscope.z;



    // Access raw sensor data

    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;

    sensor.getRawReadings(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);



    Serial.print("Accel(g): ");

    Serial.print("X="); Serial.print(accelX);

    Serial.print(" | Y="); Serial.print(accelY);

    Serial.print(" | Z="); Serial.println(accelZ);

  }

  

  delay(100);

}

```



**Note on MPU6050 WHO_AM_I Value:**

Some MPU6050 modules, particularly clones or variants, might report a `WHO_AM_I` register value different from the default `0x70`. If you encounter an initialization failure, you can pass the expected `WHO_AM_I` value as the last parameter to the `begin()` function. For example, if your sensor returns `0x68`, you would initialize the sensor like this:

```cpp

sensor.begin(GYRO_RANGE_2000DPS, ACCEL_RANGE_16G, LPF_256HZ_N_0MS, 0x68);

```



## Example



You can find a more detailed example sketch in the `examples` folder, demonstrating the recommended settings for quadcopters.



## Contributing



Contributions are welcome! Please open an issue or submit a pull request.



## License



This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
