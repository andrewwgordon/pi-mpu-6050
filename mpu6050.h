/** 
*   @file mpu6050.h
*   @author Andrew Gordon
*   @date 2 Jan 2021 
*
*   @brief TDK InvenSense MPU6050 driver.
*
*/
#ifndef _MPU6060_H
#define _MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#define OK      (1)
#define ERROR   (-1)

/**
 *  @brief i2c bus, either 0 or 1.
 *
 */
typedef enum mpu_i2c_bus_enum
{
    MPU_I2C_BUS_0,                                          // equates to linux /dev/i2c-0
    MPU_I2C_BUS_1                                           // equates to linux /dev/i2c-1
} mpu_i2c_bus_t;

/**
 *  @brief MPU6050 i2c address, default 0x68.
 *
 */
typedef enum mpu6050_i2c_addr_enum
{
    AD0_LOW     =   0x68,                                   // address pin low (GND), default for InvenSense evaluation board
    AD0_HIGH    =   0x69,                                   // address pin high (VCC)
    DEFAULT     =   AD0_LOW                                 // default address
} mpu6050_i2c_addr_t;

/**
 *  @brief MPU address
 *
 */
typedef struct mpu6050_addr_struct
{
    mpu_i2c_bus_t       i2c_bus;                            // I2C bus
    mpu6050_i2c_addr_t  i2c_addr;                           // I2C device address
} mpu6050_addr_t;

/**
 *  @brief MPU range and scale
 *
 */
typedef struct mpu6050_rangescale_struct
{
    int     range;                                          // range parmater between 0 and 3
    float   scale;                                          // scale divider for selected range
} mpu6050_rangescale_t;

/**
 *  @brief MPU Accelerometer ranges and scales.
 *
 */
#define ACCEL_FS_2      (mpu6050_rangescale_t){0x00,16384}  // +/- 2g
#define ACCEL_FS_4      (mpu6050_rangescale_t){0x01,8192}   // +/- 4g
#define ACCEL_FS_8      (mpu6050_rangescale_t){0x02,4096}   // +/- 8g
#define ACCEL_FS_16     (mpu6050_rangescale_t){0x03,2048}   // +/- 16g

/**
 *  @brief MPU Gyroscope ranges and scales.
 *
 */
#define GYRO_FS_250     (mpu6050_rangescale_t){0x00,131}    // +/- 250 deg/s
#define GYRO_FS_500     (mpu6050_rangescale_t){0x01,65.5}   // +/- 500 deg/s
#define GYRO_FS_1000    (mpu6050_rangescale_t){0x02,32.8}   // +/- 1000 deg/s
#define GYRO_FS_2000    (mpu6050_rangescale_t){0x03,16.4}   // +/- 2000 deg/s

/**
 *  @brief Engineering unit of measure multipliers
 *
 */
#define G               (1.0)                               // Acceleration in g (default)
#define MS2             (9.80665)                           // Acceleration in ms-2
#define DEGS            (1.0)                               // Rotational velocity in deg/s (default)
#define RADS            (0.0174533)                         // Rotational veloclity in rad/s

/**
 *  @brief Sample rate clock source.
 *
 */
typedef enum mpu6050_clock_source_enum 
{
    CLOCK_INTERNAL  =   0x00,                               // internal clock.
    CLOCK_PLL_XGYRO =   0x01,                               // X Gyro
    CLOCK_PLL_YGYRO =   0x02,                               // Y Gyro
    CLOCK_PLL_ZGYRO =   0x03,                               // Z Gyro
    CLOCK_PLL_EXT32K=   0x04,                               // External 
    CLOCK_PLL_EXT19M=   0x05,                               // External
    CLOCK_KEEP_RESET=   0x07                                // Reset
} mpu6050_clock_source_t;

/**
 *  @brief MPU configuration
 *
 */
typedef struct mpu6050_config_struct
{
    mpu6050_clock_source_t  clock_source;                   // Clock source
    mpu6050_rangescale_t    gyro_range;                     // Gyro range
    float                   gyro_uom;                       // Gyro units of measure
    mpu6050_rangescale_t    accel_range;                    // Accelerometer range
    float                   accel_uom;                      // Accelerometer units of measure
} mpu6050_config_t;

/**
 *  @brief MPU calibration offsets variables.
 *
 */
typedef struct mpu6050_offsetvar_struct
{
    int16_t x;                                              // x axis
    int16_t y;                                              // y axis
    int16_t z;                                              // z axis
} mpu6050_offsetvar_t;

/**
 *  @brief MPU calibration offsets.
 *
 */
typedef struct mpu6050_offset_struct
{
    mpu6050_offsetvar_t accel;                              // Accelerometer
    mpu6050_offsetvar_t gyro;                               // Gyroscope
} mpu6050_offset_t;

/**
 *  @brief Motion
 *
 */
typedef struct mpu6050_motion_struct
{
    float ax;                                               // Acceleration in x
    float ay;                                               // Acceleration in y
    float az;                                               // Acceleration in z
    float gx;                                               // Rotational velocity in x
    float gy;                                               // Rotational velocity in y
    float gz;                                               // Rotational velocity in z
} mpu6050_motion_t;

/**
*   Aquire a connection to the MPU.
*   @param[in] mpu6050_addr The I2C bus and devioe address.
*   @return The handle to the MPU or -1 for error.
*/
int16_t 
mpu6050_connect(mpu6050_addr_t mpu6050_addr);

/**
*   Get the device of the MPU (should be 0x34)
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[out] deviceid The I2C bus and devioe address.
*   @return Sucess or error.
*/
int8_t 
mpu6050_getdeviceid(int16_t mpu6050_h, uint8_t *deviceid);

/**
*   Get the device of the MPU (should be 0x34)
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] mpu6050_config MPU configuration.
*   @return Sucess or error.
*/
int8_t 
mpu6050_initialise(int16_t mpu6050_h,mpu6050_config_t mpu6050_config);

/**
*   Get the current Accelerometer and Gyro calibration offsets.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[out] mpu6050_offset MPU calbiration offsets.
*   @return Sucess or error.
*/
int8_t 
mpu6050_getoffsets(int16_t mpu6050_h,mpu6050_offset_t *mpu6050_offset);

/**
*   Set the Accelerometer and Gyro calibration offsets.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] mpu6050_offset MPU calbiration offsets.
*   @return Sucess or error.
*/
int8_t
mpu6050_setoffsets(int16_t mpu6050_h,mpu6050_offset_t offsets);

/**
*   Get the device of the MPU (should be 0x34)
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] mpu6050_config MPU configuration.
*   @param[out] motion Current motion readings for Accelerometer and Gyro.
*   @return Sucess or error.
*/
int8_t 
mpu6050_getmotion(int16_t mpu6050_h,mpu6050_config_t mpu6050_config,mpu6050_motion_t *motion);

/**
*   Reset the MPU.
*   @param[in] mpu6050_h Handle to the MPU.
*   @return Sucess or error.
*/
int8_t 
mpu6050_reset(uint16_t mpu6050_h);
#endif