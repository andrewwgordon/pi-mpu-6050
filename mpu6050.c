/** 
*   @file mpu6050.c
*   @author Andrew Gordon
*   @date 2 Jan 2021 
*
*   @brief TDK InvenSense MPU6050 driver.
*
*/
#include "i2cdev.h"
#include "mpu6050.h"

#define DEVICE_ID                       0x34

#define SELF_TEST_XA_1_BIT              0x07
#define SELF_TEST_XA_1_LENGTH           0x03
#define SELF_TEST_XA_2_BIT              0x05
#define SELF_TEST_XA_2_LENGTH           0x02
#define SELF_TEST_YA_1_BIT              0x07
#define SELF_TEST_YA_1_LENGTH           0x03
#define SELF_TEST_YA_2_BIT              0x03
#define SELF_TEST_YA_2_LENGTH           0x02
#define SELF_TEST_ZA_1_BIT              0x07
#define SELF_TEST_ZA_1_LENGTH           0x03
#define SELF_TEST_ZA_2_BIT              0x01
#define SELF_TEST_ZA_2_LENGTH           0x02

#define SELF_TEST_XG_1_BIT              0x04
#define SELF_TEST_XG_1_LENGTH           0x05
#define SELF_TEST_YG_1_BIT              0x04
#define SELF_TEST_YG_1_LENGTH           0x05
#define SELF_TEST_ZG_1_BIT              0x04
#define SELF_TEST_ZG_1_LENGTH           0x05

#define TC_PWR_MODE_BIT                 7
#define TC_OFFSET_BIT                   6
#define TC_OFFSET_LENGTH                6
#define TC_OTP_BNK_VLD_BIT              0

#define VDDIO_LEVEL_VLOGIC              0
#define VDDIO_LEVEL_VDD                 1

#define CFG_EXT_SYNC_SET_BIT            5
#define CFG_EXT_SYNC_SET_LENGTH         3
#define CFG_DLPF_CFG_BIT                2
#define CFG_DLPF_CFG_LENGTH             3

#define EXT_SYNC_DISABLED               0x0
#define EXT_SYNC_TEMP_OUT_L             0x1
#define EXT_SYNC_GYRO_XOUT_L            0x2
#define EXT_SYNC_GYRO_YOUT_L            0x3
#define EXT_SYNC_GYRO_ZOUT_L            0x4
#define EXT_SYNC_ACCEL_XOUT_L           0x5
#define EXT_SYNC_ACCEL_YOUT_L           0x6
#define EXT_SYNC_ACCEL_ZOUT_L           0x7

#define DLPF_BW_256                     0x00
#define DLPF_BW_188                     0x01
#define DLPF_BW_98                      0x02
#define DLPF_BW_42                      0x03
#define DLPF_BW_20                      0x04
#define DLPF_BW_10                      0x05
#define DLPF_BW_5                       0x06

#define GCONFIG_FS_SEL_BIT              4
#define GCONFIG_FS_SEL_LENGTH           2

#define ACONFIG_XA_ST_BIT               7
#define ACONFIG_YA_ST_BIT               6
#define ACONFIG_ZA_ST_BIT               5
#define ACONFIG_AFS_SEL_BIT             4
#define ACONFIG_AFS_SEL_LENGTH          2
#define ACONFIG_ACCEL_HPF_BIT           2
#define ACONFIG_ACCEL_HPF_LENGTH        3   

#define DHPF_RESET                      0x00
#define DHPF_5                          0x01
#define DHPF_2P5                        0x02
#define DHPF_1P25                       0x03
#define DHPF_0P63                       0x04
#define DHPF_HOLD                       0x07

#define TEMP_FIFO_EN_BIT                7
#define XG_FIFO_EN_BIT                  6
#define YG_FIFO_EN_BIT                  5
#define ZG_FIFO_EN_BIT                  4
#define ACCEL_FIFO_EN_BIT               3
#define SLV2_FIFO_EN_BIT                2
#define SLV1_FIFO_EN_BIT                1
#define SLV0_FIFO_EN_BIT                0

#define MULT_MST_EN_BIT                 7
#define WAIT_FOR_ES_BIT                 6
#define SLV_3_FIFO_EN_BIT               5
#define I2C_MST_P_NSR_BIT               4
#define I2C_MST_CLK_BIT                 3
#define I2C_MST_CLK_LENGTH              4

#define CLOCK_DIV_348                   0x0
#define CLOCK_DIV_333                   0x1
#define CLOCK_DIV_320                   0x2
#define CLOCK_DIV_308                   0x3
#define CLOCK_DIV_296                   0x4
#define CLOCK_DIV_286                   0x5
#define CLOCK_DIV_276                   0x6
#define CLOCK_DIV_267                   0x7
#define CLOCK_DIV_258                   0x8
#define CLOCK_DIV_500                   0x9
#define CLOCK_DIV_471                   0xA
#define CLOCK_DIV_444                   0xB
#define CLOCK_DIV_421                   0xC
#define CLOCK_DIV_400                   0xD
#define CLOCK_DIV_381                   0xE
#define CLOCK_DIV_364                   0xF

#define I2C_SLV_RW_BIT                  7
#define I2C_SLV_ADDR_BIT                6
#define I2C_SLV_ADDR_LENGTH             7
#define I2C_SLV_EN_BIT                  7
#define I2C_SLV_BYTE_SW_BIT             6
#define I2C_SLV_REG_DIS_BIT             5
#define I2C_SLV_GRP_BIT                 4
#define I2C_SLV_LEN_BIT                 3
#define I2C_SLV_LEN_LENGTH              4

#define I2C_SLV4_RW_BIT                 7
#define I2C_SLV4_ADDR_BIT               6
#define I2C_SLV4_ADDR_LENGTH            7
#define I2C_SLV4_EN_BIT                 7
#define I2C_SLV4_INT_EN_BIT             6
#define I2C_SLV4_REG_DIS_BIT            5
#define I2C_SLV4_MST_DLY_BIT            4
#define I2C_SLV4_MST_DLY_LENGTH         5

#define MST_PASS_THROUGH_BIT            7
#define MST_I2C_SLV4_DONE_BIT           6
#define MST_I2C_LOST_ARB_BIT            5
#define MST_I2C_SLV4_NACK_BIT           4
#define MST_I2C_SLV3_NACK_BIT           3
#define MST_I2C_SLV2_NACK_BIT           2
#define MST_I2C_SLV1_NACK_BIT           1
#define MST_I2C_SLV0_NACK_BIT           0

#define INTCFG_INT_LEVEL_BIT            7
#define INTCFG_INT_OPEN_BIT             6
#define INTCFG_LATCH_INT_EN_BIT         5
#define INTCFG_INT_RD_CLEAR_BIT         4
#define INTCFG_FSYNC_INT_LEVEL_BIT      3
#define INTCFG_FSYNC_INT_EN_BIT         2
#define INTCFG_I2C_BYPASS_EN_BIT        1
#define INTCFG_CLKOUT_EN_BIT            0

#define INTMODE_ACTIVEHIGH              0x00
#define INTMODE_ACTIVELOW               0x01

#define INTDRV_PUSHPULL                 0x00
#define INTDRV_OPENDRAIN                0x01

#define INTLATCH_50USPULSE              0x00
#define INTLATCH_WAITCLEAR              0x01

#define INTCLEAR_STATUSREAD             0x00
#define INTCLEAR_ANYREAD                0x01

#define INTERRUPT_FF_BIT                7
#define INTERRUPT_MOT_BIT               6
#define INTERRUPT_ZMOT_BIT              5
#define INTERRUPT_FIFO_OFLOW_BIT        4
#define INTERRUPT_I2C_MST_INT_BIT       3
#define INTERRUPT_PLL_RDY_INT_BIT       2
#define INTERRUPT_DMP_INT_BIT           1
#define INTERRUPT_DATA_RDY_BIT          0

#define DMPINT_5_BIT                    5
#define DMPINT_4_BIT                    4
#define DMPINT_3_BIT                    3
#define DMPINT_2_BIT                    2
#define DMPINT_1_BIT                    1
#define DMPINT_0_BIT                    0

#define MOTION_MOT_XNEG_BIT             7
#define MOTION_MOT_XPOS_BIT             6
#define MOTION_MOT_YNEG_BIT             5
#define MOTION_MOT_YPOS_BIT             4
#define MOTION_MOT_ZNEG_BIT             3
#define MOTION_MOT_ZPOS_BIT             2
#define MOTION_MOT_ZRMOT_BIT            0

#define DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define PATHRESET_GYRO_RESET_BIT        2
#define PATHRESET_ACCEL_RESET_BIT       1
#define PATHRESET_TEMP_RESET_BIT        0

#define DETECT_ACCEL_ON_DELAY_BIT       5
#define DETECT_ACCEL_ON_DELAY_LENGTH    2
#define DETECT_FF_COUNT_BIT             3
#define DETECT_FF_COUNT_LENGTH          2
#define DETECT_MOT_COUNT_BIT            1
#define DETECT_MOT_COUNT_LENGTH         2

#define DETECT_DECREMENT_RESET          0x0
#define DETECT_DECREMENT_1              0x1
#define DETECT_DECREMENT_2              0x2
#define DETECT_DECREMENT_4              0x3

#define USERCTRL_DMP_EN_BIT             7
#define USERCTRL_FIFO_EN_BIT            6
#define USERCTRL_I2C_MST_EN_BIT         5
#define USERCTRL_I2C_IF_DIS_BIT         4
#define USERCTRL_DMP_RESET_BIT          3
#define USERCTRL_FIFO_RESET_BIT         2
#define USERCTRL_I2C_MST_RESET_BIT      1
#define USERCTRL_SIG_COND_RESET_BIT     0

#define PWR1_DEVICE_RESET_BIT           7
#define PWR1_SLEEP_BIT                  6
#define PWR1_CYCLE_BIT                  5
#define PWR1_TEMP_DIS_BIT               3
#define PWR1_CLKSEL_BIT                 2
#define PWR1_CLKSEL_LENGTH              3

#define PWR2_LP_WAKE_CTRL_BIT           7
#define PWR2_LP_WAKE_CTRL_LENGTH        2
#define PWR2_STBY_XA_BIT                5
#define PWR2_STBY_YA_BIT                4
#define PWR2_STBY_ZA_BIT                3
#define PWR2_STBY_XG_BIT                2
#define PWR2_STBY_YG_BIT                1
#define PWR2_STBY_ZG_BIT                0

#define WAKE_FREQ_1P25                  0x0
#define WAKE_FREQ_2P5                   0x1
#define WAKE_FREQ_5                     0x2
#define WAKE_FREQ_10                    0x3

#define BANKSEL_PRFTCH_EN_BIT           6
#define BANKSEL_CFG_USER_BANK_BIT       5
#define BANKSEL_MEM_SEL_BIT             4
#define BANKSEL_MEM_SEL_LENGTH          5

#define WHO_AM_I_BIT                    6
#define WHO_AM_I_LENGTH                 6

#define DMP_MEMORY_BANKS                8
#define DMP_MEMORY_BANK_SIZE            256
#define DMP_MEMORY_CHUNK_SIZE           16

/**
 *  @brief MPU6050 regsiters as per https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf.
 *
 */
typedef enum reg_enum
{
    XG_OFFS_TC      =   0x00,   //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
    YG_OFFS_TC      =   0x01,   //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
    ZG_OFFS_TC      =   0x02,   //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
    X_FINE_GAIN     =   0x03,   //[7:0] X_FINE_GAIN
    Y_FINE_GAIN     =   0x04,   //[7:0] Y_FINE_GAIN
    Z_FINE_GAIN     =   0x05,   //[7:0] Z_FINE_GAIN
    XA_OFFS_H       =   0x06,   //[15:0] XA_OFFS
    XA_OFFS_L_TC    =   0x07,
    YA_OFFS_H       =   0x08,   //[15:0] YA_OFFS
    YA_OFFS_L_TC    =   0x09,
    ZA_OFFS_H       =   0x0A,   //[15:0] ZA_OFFS
    ZA_OFFS_L_TC    =   0x0B,
    SELF_TEST_X     =   0x0D,   //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
    SELF_TEST_Y     =   0x0E,   //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
    SELF_TEST_Z     =   0x0F,   //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
    SELF_TEST_A     =   0x10,   //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
    XG_OFFS_USRH    =   0x13,   //[15:0] XG_OFFS_USR
    XG_OFFS_USRL    =   0x14,
    YG_OFFS_USRH    =   0x15,   //[15:0] YG_OFFS_USR
    YG_OFFS_USRL    =   0x16,
    ZG_OFFS_USRH    =   0x17,   //[15:0] ZG_OFFS_USR
    ZG_OFFS_USRL    =   0x18,
    SMPLRT_DIV      =   0x19,   // sample rate drivder, Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    CONFIG          =   0x1A,   // Frame Synchronization (FSYNC) pin sampling and the Digital Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers
    GYRO_CONFIG     =   0x1B,   // gyroscope self-test and configure the gyroscopes’ full scale range
    ACCEL_CONFIG    =   0x1C,   // accelerometer self test and configure the accelerometer full scale range
    FF_THR          =   0x1D,
    FF_DUR          =   0x1E,
    MOT_THR         =   0x1F,
    MOT_DUR         =   0x20,
    ZRMOT_THR       =   0x21,
    ZRMOT_DUR       =   0x22,
    FIFO_EN         =   0x23,   // determines which sensor measurements are loaded into the FIFO buffer
    I2C_MST_CTRL    =   0x24,   // auxiliary I2C bus for single-master or multi-master control
    I2C_SLV0_ADDR   =   0x25,   // configure the data transfer sequence for Slave 0
    I2C_SLV0_REG    =   0x26,   // register for Slave 0
    I2C_SLV0_CTRL   =   0x27,   // control for Slave 0 
    I2C_SLV1_ADDR   =   0x28,   // configure the data transfer sequence for Slave 1
    I2C_SLV1_REG    =   0x29,   // register for Slave 1
    I2C_SLV1_CTRL   =   0x2A,   // control for Slave 1
    I2C_SLV2_ADDR   =   0x2B,   // configure the data transfer sequence for Slave 2
    I2C_SLV2_REG    =   0x2C,   
    I2C_SLV2_CTRL   =   0x2D,
    I2C_SLV3_ADDR   =   0x2E,
    I2C_SLV3_REG    =   0x2F,
    I2C_SLV3_CTRL   =   0x30,
    I2C_SLV4_ADDR   =   0x31,
    I2C_SLV4_REG    =   0x32,
    I2C_SLV4_DO     =   0x33,
    I2C_SLV4_CTRL   =   0x34,
    I2C_SLV4_DI     =   0x35,
    I2C_MST_STATUS  =   0x36,
    INT_PIN_CFG     =   0x37,
    INT_ENABLE      =   0x38,
    DMP_INT_STATUS  =   0x39,
    INT_STATUS      =   0x3A,
    ACCEL_XOUT_H    =   0x3B,
    ACCEL_XOUT_L    =   0x3C,
    ACCEL_YOUT_H    =   0x3D,
    ACCEL_YOUT_L    =   0x3E,
    ACCEL_ZOUT_H    =   0x3F,
    ACCEL_ZOUT_L    =   0x40,
    TEMP_OUT_H      =   0x41,
    TEMP_OUT_L      =   0x42,
    GYRO_XOUT_H     =   0x43,
    GYRO_XOUT_L     =   0x44,
    GYRO_YOUT_H     =   0x45,
    GYRO_YOUT_L     =   0x46,
    GYRO_ZOUT_H     =   0x47,
    GYRO_ZOUT_L     =   0x48,
    EXT_SENS_DATA_00 =  0x49,
    EXT_SENS_DATA_01 =  0x4A,
    EXT_SENS_DATA_02 =  0x4B,
    EXT_SENS_DATA_03 =  0x4C,
    EXT_SENS_DATA_04 =  0x4D,
    EXT_SENS_DATA_05 =  0x4E,
    EXT_SENS_DATA_06 =  0x4F,
    EXT_SENS_DATA_07 =  0x50,
    EXT_SENS_DATA_08 =  0x51,
    EXT_SENS_DATA_09 =  0x52,
    EXT_SENS_DATA_10 =  0x53,
    EXT_SENS_DATA_11 =  0x54,
    EXT_SENS_DATA_12 =  0x55,
    EXT_SENS_DATA_13 =  0x56,
    EXT_SENS_DATA_14 =  0x57,
    EXT_SENS_DATA_15 =  0x58,
    EXT_SENS_DATA_16 =  0x59,
    EXT_SENS_DATA_17 =  0x5A,
    EXT_SENS_DATA_18 =  0x5B,
    EXT_SENS_DATA_19 =  0x5C,
    EXT_SENS_DATA_20 =  0x5D,
    EXT_SENS_DATA_21 =  0x5E,
    EXT_SENS_DATA_22 =  0x5F,
    EXT_SENS_DATA_23 =  0x60,
    MOT_DETECT_STATUS = 0x61,
    I2C_SLV0_DO       = 0x63,
    I2C_SLV1_DO       = 0x64,
    I2C_SLV2_DO       = 0x65,
    I2C_SLV3_DO       = 0x66,
    I2C_MST_DELAY_CTRL= 0x67,
    SIGNAL_PATH_RESET = 0x68,
    MOT_DETECT_CTRL   = 0x69,
    USER_CTRL         = 0x6A,
    PWR_MGMT_1        = 0x6B,
    PWR_MGMT_2        = 0x6C,
    BANK_SEL          = 0x6D,
    MEM_START_ADDR    = 0x6E,
    MEM_R_W           = 0x6F,
    DMP_CFG_1         = 0x70,
    DMP_CFG_2         = 0x71,
    FIFO_COUNTH       = 0x72,
    FIFO_COUNTL       = 0x73,
    FIFO_R_W          = 0x74,
    WHO_AM_I          = 0x75  
} reg_t;

/**
*   Aquire a connection to the MPU.
*   @param[in] mpu6050_addr The I2C bus and devioe address.
*   @return The handle to the MPU or -1 for error.
*/
int16_t 
mpu6050_connect(mpu6050_addr_t addr)
{
    int16_t mpu6050_h;

    // Initialise the I2C bus at the device address.
    //
    mpu6050_h = i2cdev_init(addr.i2c_bus,addr.i2c_addr);
    if (mpu6050_h != ERROR)
    {
        uint8_t deviceid;
        // Check Device Id at address is 0x34
        //
        if ((mpu6050_getdeviceid(mpu6050_h,&deviceid) != ERROR) && (deviceid == DEVICE_ID))
        {
            return mpu6050_h;
        }
    }
    return ERROR;
}

/**
*   Get the device of the MPU (should be 0x34)
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[out] deviceid The I2C bus and devioe address.
*   @return Sucess or error.
*/
int8_t 
mpu6050_getdeviceid(int16_t mpu6050_h, uint8_t *deviceid)
{
    uint8_t buf;
    
    // Get the Device Id.
    //
    if (i2cdev_readbits(mpu6050_h,WHO_AM_I,WHO_AM_I_BIT,WHO_AM_I_LENGTH,&buf) != ERROR)
    {
        *deviceid = buf;
        return OK;
    }
    else
    {
        return ERROR;
    }
}

/**
*   Set the MPU clock source.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] clock_source The clock source
*   @return Sucess or error.
*/
int8_t
mpu6050_setclocksource(uint16_t mpu6050_h,mpu6050_clock_source_t clock_source)
{
    return i2cdev_writebits(mpu6050_h,PWR_MGMT_1,PWR1_CLKSEL_BIT,PWR1_CLKSEL_LENGTH,clock_source);
}

/**
*   Set the Gyro range.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] range_scale The range and scale factor.
*   @return Sucess or error.
*/
int8_t 
mpu6050_setgyrorange(uint16_t mpu6050_h,mpu6050_rangescale_t rangescale)
{
    return i2cdev_writebits(mpu6050_h,GYRO_CONFIG,GCONFIG_FS_SEL_BIT,GCONFIG_FS_SEL_LENGTH,rangescale.range);
}

/**
*   Set the Accelerometer range.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] range_scale The range and scale factor.
*   @return Sucess or error.
*/
int8_t 
mpu6050_setaccelrange(uint16_t mpu6050_h,mpu6050_rangescale_t rangescale)
{
    return i2cdev_writebits(mpu6050_h,ACCEL_CONFIG,ACONFIG_AFS_SEL_BIT,ACONFIG_AFS_SEL_LENGTH,rangescale.range);
}

/**
*   Set MPU sleep mode.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] enabled True or false.
*   @return Sucess or error.
*/
int8_t 
mpu6050_setsleepenabled(uint16_t mpu6050_h,bool enabled)
{
    return i2cdev_writebit(mpu6050_h,PWR_MGMT_1,PWR1_SLEEP_BIT,enabled);
}

/**
*   Reset the MPU.
*   @param[in] mpu6050_h Handle to the MPU.
*   @return Sucess or error.
*/
int8_t 
mpu6050_reset(uint16_t mpu6050_h)
{
    int8_t err;
    // Requres full 0x01 to work on my MPU-6050?
    //
    err = i2cdev_writebyte(mpu6050_h,PWR_MGMT_1,0x01);    
    // Recommend wait for 50ms
    //
    usleep(50000);
    return err;
}

/**
*   Initialise the MPU.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] mpu6050_config MPU configuration.
*   @return Sucess or error.
*/
int8_t 
mpu6050_initialise(int16_t mpu6050_h, mpu6050_config_t config)
{
    // Initialise the MPU
    // Reset
    //
    if (mpu6050_reset(mpu6050_h) == ERROR)
    {
        return ERROR;
    }
    // Set the clock source
    //
    if (mpu6050_setclocksource(mpu6050_h,config.clock_source) == ERROR)
    {
        return ERROR;
    }
    // Set the Gyro range
    //
    if (mpu6050_setgyrorange(mpu6050_h,config.gyro_range) == ERROR)
    {
        return ERROR;
    }
    // Set the Accelerometer range
    //
    if (mpu6050_setaccelrange(mpu6050_h,config.accel_range) == ERROR)
    {
        return ERROR;
    }
    // Disable sleep mode
    //
    if (mpu6050_setsleepenabled(mpu6050_h,false) == ERROR)
    {
        return ERROR;
    }
    // To do - Bandwidth filter and sampling rate.
    return OK;
}

/**
*   Get a single offset value
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] offset_reg Offset register.
*   @param[out] offset Offset value.
*   @return Sucess or error.
*/
int8_t 
mpu6050_getoffset(int16_t mpu6050_h,reg_t offset_reg,int16_t *offset)
{
    uint8_t buf[2];

    // Read two bytes at register.
    //
    if (i2cdev_readbytes(mpu6050_h,offset_reg,2,buf) != ERROR)
    {
        // Two complement to 16-bit unsigned.
        //
        *offset = (((int16_t)buf[0]) << 8) | buf[1];
    }
    else
    {
        return ERROR;
    }
}

/**
*   Set the Accelerometer and Gyro calibration offsets.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] mpu6050_offset MPU calbiration offsets.
*   @return Sucess or error.
*/
int8_t 
mpu6050_setoffsets(int16_t mpu6050_h,mpu6050_offset_t offsets)
{
    // Set Accelerometer X axis.
    //
    if (i2cdev_writeword(mpu6050_h,XA_OFFS_H,offsets.accel.x) == ERROR)
    {
        return ERROR;
    }
    // Set Accelerometer Y axis.
    //
    if (i2cdev_writeword(mpu6050_h,YA_OFFS_H,offsets.accel.y) == ERROR)
    {
        return ERROR;
    }
    // Set Accelerometer Z axis.
    //
    if (i2cdev_writeword(mpu6050_h,ZA_OFFS_H,offsets.accel.z) == ERROR)
    {
        return ERROR;
    }
    // Set Gyroscope X axis.
    //
    if (i2cdev_writeword(mpu6050_h,XG_OFFS_USRH,offsets.gyro.x) == ERROR)
    {
        return ERROR;
    }
    // Set Gyroscope Y axis.
    //
    if (i2cdev_writeword(mpu6050_h,YG_OFFS_USRH,offsets.gyro.y) == ERROR)
    {
        return ERROR;
    }
    // Set Gyroscope Z axis.
    //
    if (i2cdev_writeword(mpu6050_h,ZG_OFFS_USRH,offsets.gyro.z) == ERROR)
    {
        return ERROR;
    }
}

/**
*   Get the current Accelerometer and Gyro calibration offsets.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[out] mpu6050_offset MPU calbiration offsets.
*   @return Sucess or error.
*/
int8_t 
mpu6050_getoffsets(int16_t mpu6050_h,mpu6050_offset_t *mpu6050_offset)
{
    int16_t offset;

    // Get Accelerometer X axis.
    //
    if (mpu6050_getoffset(mpu6050_h,XA_OFFS_H,&offset) == ERROR)
    {
        return ERROR;
    }
    mpu6050_offset->accel.x=offset;
    // Get Accelerometer Y axis.
    //
    if (mpu6050_getoffset(mpu6050_h,YA_OFFS_H,&offset) == ERROR)
    {
        return ERROR;
    }
    mpu6050_offset->accel.y=offset;
    // Get Accelerometer Z axis.
    //
    if (mpu6050_getoffset(mpu6050_h,ZA_OFFS_H,&offset) == ERROR)
    {
        return ERROR;
    }
    mpu6050_offset->accel.z=offset;
    // Get Gyroscope X axis.
    //
    if (mpu6050_getoffset(mpu6050_h,XG_OFFS_USRH,&offset) == ERROR)
    {
        return ERROR;
    }
    mpu6050_offset->gyro.x=offset;
    // Get Gyroscope Y axis.
    //
    if (mpu6050_getoffset(mpu6050_h,YG_OFFS_USRH,&offset) == ERROR)
    {
        return ERROR;
    }
    mpu6050_offset->gyro.y=offset;
    // Get Gyroscope Z axis.
    //
    if (mpu6050_getoffset(mpu6050_h,ZG_OFFS_USRH,&offset) == ERROR)
    {
        return ERROR;
    }
    mpu6050_offset->gyro.z=offset;
    return OK;
}

/**
*   Return LSB, MSB in a word to 16-bit signed int.
*   @param[in] LSB Left significant byte
*   @param[in] MSB Most significant byte
*   @return int16_t
*/
int16_t 
two_complement_to_int(uint8_t LSB, uint8_t MSB) 
{
	int16_t signed_int = 0;
	uint16_t word;

	word = (uint16_t)(( MSB << 8) | LSB);
	if((word & 0x8000) == 0x8000) 
    {
		signed_int = (int16_t) -(~word);
	} 
    else 
    {
		signed_int = (int16_t) (word & 0x7fff);
	}
	return signed_int;
}

/**
*   Get motion readings.
*   @param[in] mpu6050_h Handle to the MPU.
*   @param[in] mpu6050_config MPU configuration.
*   @param[out] motion Current motion readings for Accelerometer and Gyro.
*   @return Sucess or error.
*/
int8_t 
mpu6050_getmotion(int16_t mpu6050_h,mpu6050_config_t config,mpu6050_motion_t *motion)
{
    uint8_t buffer[14];

    // Read 14 bytes from ACCEL_XOUT_H, fetching acceleration and gyro readings in one go.
    //
    if (i2cdev_readbytes(mpu6050_h,ACCEL_XOUT_H,14,buffer) == ERROR)
    {
       return ERROR;
    }
    // Convert raw word to float via scale factor and units of measure.
    //
    motion->ax = (float)two_complement_to_int(buffer[1],buffer[0]) / config.accel_range.scale * config.accel_uom;
    motion->ay = (float)two_complement_to_int(buffer[3],buffer[2]) / config.accel_range.scale * config.accel_uom;
    motion->az = (float)two_complement_to_int(buffer[5],buffer[4]) / config.accel_range.scale * config.accel_uom;
    motion->gx = (float)two_complement_to_int(buffer[9],buffer[8]) / config.gyro_range.scale  * config.gyro_uom;
    motion->gy = (float)two_complement_to_int(buffer[11],buffer[10]) / config.gyro_range.scale  * config.gyro_uom;
    motion->gz = (float)two_complement_to_int(buffer[13],buffer[12]) / config.gyro_range.scale  * config.gyro_uom;
    return OK;
}