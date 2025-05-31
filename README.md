# Raspberry Pi TDK InvenSense MPU-6050 Driver
User mode driver for [TDK InvenSense MPU-6050]

## Overview
This project provides a user-mode C driver for the TDK InvenSense MPU-6050 6-axis motion sensor (3-axis accelerometer + 3-axis gyroscope) for use on the Raspberry Pi. The driver allows you to configure the sensor, read raw and calibrated motion data, and estimate device orientation (pitch, roll, yaw) using a complementary filter.

## File Structure
- `mpu6050.c`: Main driver implementation. Contains all logic for sensor communication, configuration, calibration, motion/orientation calculation, and filtering.
- `mpu6050.h`: Header file with type definitions and function prototypes.
- `mpu6050_example.c`: Example program demonstrating how to use the driver to read motion and orientation data.

## Key Features in `mpu6050.c`
- **I2C Communication**: Handles low-level I2C read/write to the MPU-6050 registers.
- **Configuration**: Set accelerometer/gyro ranges, clock source, and sleep mode.
- **Calibration**: Get/set sensor offsets for all axes. Includes an `mpu6050_autocalibration` function to automatically determine offsets by averaging stationary samples.
- **Motion Reading**: Reads raw accelerometer and gyroscope data, converts to physical units (g, deg/s).
- **Orientation Estimation**: Calculates pitch, roll, and yaw using a complementary filter that fuses accelerometer and gyro data for improved accuracy and stability.
- **Angle Limiting**: Ensures angles remain within valid ranges.
- **Detailed Comments**: All functions are documented for clarity and maintainability.

## Example Usage: `mpu6050_example.c`
This example demonstrates how to:
- Connect to the MPU-6050 sensor over I2C
- Configure the sensor (range, units, clock, etc.)
- Read and print current calibration offsets
- Set calibration offsets (either manually or using the autocalibration function)
- Read and print motion data (acceleration and gyro)
- Read and print orientation data (pitch, roll, yaw)

## How to Compile and Run

### Prerequisites
- Raspberry Pi with I2C enabled
- MPU-6050 sensor connected to the Pi's I2C bus
- GCC compiler

### Build the Example
```bash
gcc mpu6050_example.c mpu6050.c -o mpu6050_example -lm
```

### Run the Example
```bash
./mpu6050_example
```

The program will:
1. Connect to the MPU-6050 and print the device ID.
2. Print current calibration offsets.
3. Set calibration offsets (edit in code or use autocalibration).
4. Print 100 samples of raw accelerometer and gyro data.
5. Print 1000 samples of estimated orientation (roll, pitch, yaw).

## Using the Autocalibration Function

The driver provides an automatic calibration function, `mpu6050_autocalibration`, to help you determine the optimal accelerometer and gyroscope offsets for your MPU-6050. This is especially useful for improving accuracy and reducing bias in your sensor readings.

### How It Works
- The function collects a specified number of samples while the sensor is stationary.
- It averages the raw accelerometer and gyroscope readings.
- It calculates and returns the offsets needed to zero the sensor outputs (with Z-accel adjusted for gravity).

### Example Usage
Add the following code to your setup, after initializing the sensor:

```c
mpu6050_offset_t offsets;
// Automatically calibrate using 1000 samples (sensor must be stationary)
if (mpu6050_autocalibration(mpu6050_h, config, &offsets, 1000) == OK) {
    // Write the calculated offsets to the sensor
    mpu6050_setoffsets(mpu6050_h, offsets);
    printf("Autocalibration complete. Offsets applied.\n");
} else {
    printf("Autocalibration failed.\n");
}
```

### Notes
- The sensor must remain completely still during calibration for best results.
- You can adjust the number of samples (e.g., 1000) for a balance between speed and accuracy.
- After calibration, the offsets are written to the sensor and will be used for all subsequent readings.

## API Overview
- `mpu6050_connect`, `mpu6050_initialise`, `mpu6050_setgyrorange`, `mpu6050_setaccelrange`, `mpu6050_setsleepenabled`, `mpu6050_reset`: Sensor setup/configuration.
- `mpu6050_getmotion`: Read current acceleration and gyro data.
- `mpu6050_getorientation`: Estimate orientation using a complementary filter.
- `mpu6050_getoffsets`, `mpu6050_setoffsets`: Get/set calibration offsets.
- `mpu6050_autocalibration`: Automatically determine offsets by averaging stationary samples.

For more details, see the comments in `mpu6050.c` and the usage in `mpu6050_example.c`.

[TDK InvenSense MPU-6050]: https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/