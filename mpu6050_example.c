#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include "mpu6050.h"

int
main(void)
{
    mpu6050_addr_t      mpu6050_addr;
    int16_t             mpu6050_h;
    uint8_t             deviceid;
    mpu6050_config_t    config;
    
    // Set the I2C bus and device address.
    //
    mpu6050_addr.i2c_bus = MPU_I2C_BUS_1;
    mpu6050_addr.i2c_addr = DEFAULT;
    // Connect to the MPU.
    //
    mpu6050_h = mpu6050_connect(mpu6050_addr);
    if (mpu6050_h != ERROR)
    {
        printf("Connected to MPU6050...\n");
    }
    else
    {
        perror("Error");
        return errno;
    }
    // Get the MPU Device Id.
    //
    if (mpu6050_getdeviceid(mpu6050_h,&deviceid) != ERROR)
    {
        printf("Device Id: 0x%x\n",deviceid);
    }
    else
    {
        perror("Error");
        return errno;
    }
    // Set the MPU confiuration.
    //
    config.accel_range = ACCEL_FS_2;        // +/- 2g
    config.accel_uom= G;                    // UoM in g.
    config.gyro_range = GYRO_FS_250;        // +/- 250 deg/s
    config.gyro_uom = DEGS;                 // UoM in deg/s
    config.clock_source = CLOCK_PLL_XGYRO;  // Clock source X Gyro
    // Iniitalise the MPU
    //
    if (mpu6050_initialise(mpu6050_h,config) != ERROR)
    {
        printf("MPU initialised.\n");
    }
    else
    {
        perror("Error");
        return errno;
    }
    // Get the current calibration offsets.
    //
    mpu6050_offset_t offsets;    
    if (mpu6050_getoffsets(mpu6050_h,&offsets) != ERROR)
    {
        printf("Current offsets...\n");
        printf("Acceleration X:%d\n",offsets.accel.x);
        printf("Acceleration Y:%d\n",offsets.accel.y);
        printf("Acceleration Z:%d\n",offsets.accel.z);
        printf("Gyroscope X:%d\n",offsets.gyro.x);
        printf("Gyroscope Y:%d\n",offsets.gyro.y);
        printf("Gyroscope Z:%d\n",offsets.gyro.z);
    }
    // My settings, yours will differ (auto calibrate function to do)
    //
    offsets.accel.x=-269;
    offsets.accel.y=675;
    offsets.accel.z=1163;
    offsets.gyro.x=11;
    offsets.gyro.y=-72;
    offsets.gyro.z=-10;
    // Write my calibration offsets to the MPU.
    //
    if (mpu6050_setoffsets(mpu6050_h,offsets) == ERROR)
    {
        perror("Error");
        return errno;
    }
    mpu6050_motion_t motion;
    printf("Press <RETURN>:");
    fflush(stdout);
    getchar();
    // Get a thousand readings.
    //
    for (uint16_t i = 0;i < 1000;i++)
    {
        if (mpu6050_getmotion(mpu6050_h,config,&motion) != ERROR)
        {
            printf("ax:%f ay:%f az:%f gx:%f gy:%f gz:%f\n",motion.ax,motion.ay,motion.az,motion.gx,motion.gy,motion.gz);
        }
        else
        {
            perror("Error");
        }        
        usleep(10000);
    }
    return 0;
}