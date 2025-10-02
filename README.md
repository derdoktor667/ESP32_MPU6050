# ESP32_MPU6050 Arduino Library

This is an Arduino library for the MPU6050 gyroscope and accelerometer sensor, specifically optimized for use with the ESP32 in projects like quadcopters.

It provides a clean interface to read calibrated sensor data in standard physical units (g's and degrees/sec).

## Features

*   Easy initialization and configuration.
*   Built-in sensor calibration.
*   Data is returned in a single struct for atomic access.
*   Output is in physical units (g for acceleration, dps for gyroscope).
*   Configurable accelerometer and gyroscope ranges.

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

In your `setup()` function, initialize the sensor. You can specify the desired gyroscope and accelerometer ranges. Then, run the calibration routine. **The sensor must be kept still during calibration.**

```cpp
void setup() {
  Serial.begin(115200);

  // Initialize the sensor with a specific range
  if (!sensor.begin(GYRO_RANGE_500DPS, ACCEL_RANGE_4G)) {
    Serial.println("Failed to initialize MPU6050! Check wiring.");
    while (1);
  }

  // Calibrate the sensor. Do not move it during this process.
  Serial.println("Calibrating sensor... Hold it.");
  sensor.calibrate();
  Serial.println("Calibration complete.");
}
```

Finally, in your `loop()` function, call `sensor.update()` and then access the calibrated data through the public `sensor.readings` struct.

**Note on MPU6050 WHO_AM_I Value:**
Some MPU6050 modules, particularly clones or variants, might report a `WHO_AM_I` register value different from the standard `0x68`. If you encounter an initialization failure with a `WHO_AM_I` value like `0x70`, you might need to adjust the `MPU6050_WHO_AM_I_EXPECTED_VALUE` define in `src/ESP32_MPU6050.h` to match your sensor's reported value. The library is designed to be adaptable to such variations.

```cpp
void loop() {
  if (sensor.update()) {
    // Access accelerometer data in g's
    float accelX = sensor.readings.accelerometer.x;
    float accelY = sensor.readings.accelerometer.y;
    float accelZ = sensor.readings.accelerometer.z;

    // Access gyroscope data in degrees/sec
    float gyroX = sensor.readings.gyroscope.x;
    float gyroY = sensor.readings.gyroscope.y;
    float gyroZ = sensor.readings.gyroscope.z;

    Serial.print("Accel(g): ");
    Serial.print("X="); Serial.print(accelX);
    Serial.print(" | Y="); Serial.print(accelY);
    Serial.print(" | Z="); Serial.println(accelZ);
  }
  
  delay(100);
}
```

## Example

You can find a more detailed example sketch in the `examples` folder.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
