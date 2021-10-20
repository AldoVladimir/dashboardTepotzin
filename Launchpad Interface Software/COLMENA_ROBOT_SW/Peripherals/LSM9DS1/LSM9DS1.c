/************************************************************
 * LSM9DS1.c  Created on: 28/01/2019                        *
 *                                                          *
 *      Universidad Nacional Autonoma de Mexico             *
 *          Instituto de Ciencias Nucleares                 *
 *     Laboratorio de Instrumentacion Espacial LINX         *
 *                                                          *
 *  Desarrollador: Ing. Manuel Andres Herrera Juarez.       *
 *                                                          *
 ************************************************************/

/******************************************************************************************
 *                                                                                        *
 *    Libreria para Arduino del LSM9DS1 de Sparkfun Adaptada al lenguaje C, asi como      *
 *    los perifericos para su uso en el microcontrolador CC26x0 de Texas Instruments      *
 *    y mas especifico en el proyecto de los Robots Lunares por LINX                      *
 *                                                                                        *
 ******************************************************************************************/


/********************************************************************************************
*   SFE_LSM9DS1.cpp                                                                         *
*                                                                                           *
*   SFE_LSM9DS1 Library Source File                                                         *
*       Jim Lindblom @ SparkFun Electronics                                                 *
*       Original Creation Date: February 27, 2015                                           *
*       https://github.com/sparkfun/LSM9DS1_Breakout                                        *
*                                                                                           *
*   This file implements all functions of the LSM9DS1 class. Functions here range           *
*   from higher level stuff, like reading/writing LSM9DS1 registers to low-level,           *
*   hardware reads and writes. Both SPI and I2C handler functions can be found              *
*   towards the bottom of this file.                                                        *
*                                                                                           *
*   Development environment specifics:                                                      *
*       IDE: Arduino 1.6                                                                    *
*       Hardware Platform: Arduino Uno                                                      *
*       LSM9DS1 Breakout Version: 1.0                                                       *
*                                                                                           *
*   This code is beerware; if you see me (or any other SparkFun employee) at the            *
*   local, and you've found our code helpful, please buy us a round!                        *
*   Distributed as-is; no warranty is given.                                                *
*                                                                                           *
*********************************************************************************************/


#include "LSM9DS1.h"
#include <driverlib/cpu.h>
// Sensor Sensitivity Constants
// Values set according to the typical specifications provided in
// table 3 of the LSM9DS1 datasheet. (pg 12)
#define SENSITIVITY_ACCELEROMETER_2  0.00006104
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058


void IMU_init( uint8_t xgAddr, uint8_t mAddr)
{
    Imu._xgAddress = xgAddr;
    Imu._mAddress = mAddr;

    Imu.settings.gyro.enabled = true;
    Imu.settings.gyro.enableX = true;
    Imu.settings.gyro.enableY = true;
    Imu.settings.gyro.enableZ = true;
    // gyro scale can be 245, 500, or 2000
    Imu.settings.gyro.scale = G_SCALE_500DPS;//245;
    // gyro sample rate: value between 1-6
    // 1 = 14.9    4 = 238
    // 2 = 59.5    5 = 476
    // 3 = 119     6 = 952
    Imu.settings.gyro.sampleRate = 6;
    // gyro cutoff frequency: value between 0-3
    // Actual value of cutoff frequency depends
    // on sample rate.
    Imu.settings.gyro.bandwidth = 0;
    Imu.settings.gyro.lowPowerEnable = false;
    Imu.settings.gyro.HPFEnable = false;
    // Gyro HPF cutoff frequency: value between 0-9
    // Actual value depends on sample rate. Only applies
    // if gyroHPFEnable is true.
    Imu.settings.gyro.HPFCutoff = 0;
    Imu.settings.gyro.flipX = false;
    Imu.settings.gyro.flipY = false;
    Imu.settings.gyro.flipZ = false;
    Imu.settings.gyro.orientation = 0;
    Imu.settings.gyro.latchInterrupt = true;

    Imu.settings.accel.enabled = true;
    Imu.settings.accel.enableX = true;
    Imu.settings.accel.enableY = true;
    Imu.settings.accel.enableZ = true;
    // accel scale can be 2, 4, 8, or 16
    Imu.settings.accel.scale = 2;
    // accel sample rate can be 1-6
    // 1 = 10 Hz    4 = 238 Hz
    // 2 = 50 Hz    5 = 476 Hz
    // 3 = 119 Hz   6 = 952 Hz
    Imu.settings.accel.sampleRate = 6;
    // Accel cutoff freqeuncy can be any value between -1 - 3.
    // -1 = bandwidth determined by sample rate
    // 0 = 408 Hz   2 = 105 Hz
    // 1 = 211 Hz   3 = 50 Hz
    Imu.settings.accel.bandwidth = -1;
    Imu.settings.accel.highResEnable = false;
    // accelHighResBandwidth can be any value between 0-3
    // LP cutoff is set to a factor of sample rate
    // 0 = ODR/50    2 = ODR/9
    // 1 = ODR/100   3 = ODR/400
    Imu.settings.accel.highResBandwidth = 0;

    Imu.settings.mag.enabled = true;
    // mag scale can be 4, 8, 12, or 16
    Imu.settings.mag.scale = 4;
    // mag data rate can be 0-7
    // 0 = 0.625 Hz  4 = 10 Hz
    // 1 = 1.25 Hz   5 = 20 Hz
    // 2 = 2.5 Hz    6 = 40 Hz
    // 3 = 5 Hz      7 = 80 Hz
    Imu.settings.mag.sampleRate = 7;
    Imu.settings.mag.tempCompensationEnable = false;
    // magPerformance can be any value between 0-3
    // 0 = Low power mode      2 = high performance
    // 1 = medium performance  3 = ultra-high performance
    Imu.settings.mag.XYPerformance = 3;
    Imu.settings.mag.ZPerformance = 3;
    Imu.settings.mag.lowPowerEnable = false;
    // magOperatingMode can be 0-2
    // 0 = continuous conversion
    // 1 = single-conversion
    // 2 = power down
    Imu.settings.mag.operatingMode = 0;

    Imu.settings.temp.enabled = true;
    int i;
    for (i=0; i<3; i++)
    {
        Imu.gBias[i] = 0;
        Imu.aBias[i] = 0;
        Imu.mBias[i] = 0;
        Imu.gBiasRaw[i] = 0;
        Imu.aBiasRaw[i] = 0;
        Imu.mBiasRaw[i] = 0;
    }
    Imu._autoCalc = false;
}


uint16_t IMU_begin()
{

    IMU_constrainScales();
    // Once we have the scale values, we can calculate the resolution
    // of each sensor. That's what these functions are for. One for each sensor
    IMU_calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
    IMU_calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
    IMU_calcaRes(); // Calculate g / ADC tick, stored in aRes variable

    // To verify communication, we can read from the WHO_AM_I register of
    // each Imu. Store those in a variable so we can return them.
    uint8_t mTest = IMU_mReadByte(WHO_AM_I_M);      // Read the gyro WHO_AM_I
    uint8_t xgTest = IMU_xgReadByte(WHO_AM_I_XG);   // Read the accel/mag WHO_AM_I
    uint16_t whoAmICombined = (xgTest << 8) | mTest;

    if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
        return 0;

    // Gyro initialization stuff:
    IMU_initGyro(); // This will "turn on" the gyro. Setting up interrupts, etc.

    // Accelerometer initialization stuff:
    IMU_initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.

    // Magnetometer initialization stuff:
    IMU_initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.

    IMU_calibrate(true);
    // Once everything is initialized, return the WHO_AM_I registers we read:
    return whoAmICombined;
}

void IMU_initGyro()
{
    uint8_t tempRegValue = 0;

    // CTRL_REG1_G (Default value: 0x00)
    // [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
    // ODR_G[2:0] - Output data rate selection
    // FS_G[1:0] - Gyroscope full-scale selection
    // BW_G[1:0] - Gyroscope bandwidth selection

    // To disable gyro, set sample rate bits to 0. We'll only set sample
    // rate if the gyro is enabled.
    if (Imu.settings.gyro.enabled)
    {
        tempRegValue = (Imu.settings.gyro.sampleRate & 0x07) << 5;
    }
    switch (Imu.settings.gyro.scale)
    {
        case 500:
            tempRegValue |= (0x1 << 3);
            break;
        case 2000:
            tempRegValue |= (0x3 << 3);
            break;
        // Otherwise we'll set it to 245 dps (0x0 << 4)
    }
    tempRegValue |= (Imu.settings.gyro.bandwidth & 0x3);
    IMU_xgWriteByte(CTRL_REG1_G, tempRegValue);

    // CTRL_REG2_G (Default value: 0x00)
    // [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
    // INT_SEL[1:0] - INT selection configuration
    // OUT_SEL[1:0] - Out selection configuration
    IMU_xgWriteByte(CTRL_REG2_G, 0x00);

    // CTRL_REG3_G (Default value: 0x00)
    // [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
    // LP_mode - Low-power mode enable (0: disabled, 1: enabled)
    // HP_EN - HPF enable (0:disabled, 1: enabled)
    // HPCF_G[3:0] - HPF cutoff frequency
    tempRegValue = Imu.settings.gyro.lowPowerEnable ? (1<<7) : 0;
    if (Imu.settings.gyro.HPFEnable)
    {
        tempRegValue |= (1<<6) | (Imu.settings.gyro.HPFCutoff & 0x0F);
    }
    IMU_xgWriteByte(CTRL_REG3_G, tempRegValue);

    // CTRL_REG4 (Default value: 0x38)
    // [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
    // Zen_G - Z-axis output enable (0:disable, 1:enable)
    // Yen_G - Y-axis output enable (0:disable, 1:enable)
    // Xen_G - X-axis output enable (0:disable, 1:enable)
    // LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
    // 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
    tempRegValue = 0;
    if (Imu.settings.gyro.enableZ) tempRegValue |= (1<<5);
    if (Imu.settings.gyro.enableY) tempRegValue |= (1<<4);
    if (Imu.settings.gyro.enableX) tempRegValue |= (1<<3);
    if (Imu.settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
    IMU_xgWriteByte(CTRL_REG4, tempRegValue);

    // ORIENT_CFG_G (Default value: 0x00)
    // [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
    // SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
    // Orient [2:0] - Directional user orientation selection
    tempRegValue = 0;
    if (Imu.settings.gyro.flipX) tempRegValue |= (1<<5);
    if (Imu.settings.gyro.flipY) tempRegValue |= (1<<4);
    if (Imu.settings.gyro.flipZ) tempRegValue |= (1<<3);
    IMU_xgWriteByte(ORIENT_CFG_G, tempRegValue);
}

void IMU_initAccel()
{
    uint8_t tempRegValue = 0;

    //  CTRL_REG5_XL (0x1F) (Default value: 0x38)
    //  [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
    //  DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
    //      00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
    //  Zen_XL - Z-axis output enabled
    //  Yen_XL - Y-axis output enabled
    //  Xen_XL - X-axis output enabled
    if (Imu.settings.accel.enableZ) tempRegValue |= (1<<5);
    if (Imu.settings.accel.enableY) tempRegValue |= (1<<4);
    if (Imu.settings.accel.enableX) tempRegValue |= (1<<3);

    IMU_xgWriteByte(CTRL_REG5_XL, tempRegValue);

    // CTRL_REG6_XL (0x20) (Default value: 0x00)
    // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
    // ODR_XL[2:0] - Output data rate & power mode selection
    // FS_XL[1:0] - Full-scale selection
    // BW_SCAL_ODR - Bandwidth selection
    // BW_XL[1:0] - Anti-aliasing filter bandwidth selection
    tempRegValue = 0;
    // To disable the accel, set the sampleRate bits to 0.
    if (Imu.settings.accel.enabled)
    {
        tempRegValue |= (Imu.settings.accel.sampleRate & 0x07) << 5;
    }
    switch (Imu.settings.accel.scale)
    {
        case 4:
            tempRegValue |= (0x2 << 3);
            break;
        case 8:
            tempRegValue |= (0x3 << 3);
            break;
        case 16:
            tempRegValue |= (0x1 << 3);
            break;
        // Otherwise it'll be set to 2g (0x0 << 3)
    }
    if (Imu.settings.accel.bandwidth >= 0)
    {
        tempRegValue |= (1<<2); // Set BW_SCAL_ODR
        tempRegValue |= (Imu.settings.accel.bandwidth & 0x03);
    }
    IMU_xgWriteByte(CTRL_REG6_XL, tempRegValue);

    // CTRL_REG7_XL (0x21) (Default value: 0x00)
    // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
    // HR - High resolution mode (0: disable, 1: enable)
    // DCF[1:0] - Digital filter cutoff frequency
    // FDS - Filtered data selection
    // HPIS1 - HPF enabled for interrupt function
    tempRegValue = 0;
    if (Imu.settings.accel.highResEnable)
    {
        tempRegValue |= (1<<7); // Set HR bit
        tempRegValue |= (Imu.settings.accel.highResBandwidth & 0x3) << 5;
    }
    IMU_xgWriteByte(CTRL_REG7_XL, tempRegValue);
}


// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void IMU_calibrate( bool autoCalc)
{
    //uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    uint8_t samples = 32;
    int ii;
    int32_t aBiasRawTemp[3] = {0, 0, 0};
    int32_t gBiasRawTemp[3] = {0, 0, 0};

    // Turn on FIFO and set threshold to 32 samples
    //IMU_enableFIFO(true);
    /*IMU_setFIFO(FIFO_THS, 0x1F);
    while (samples < 0x1F)
    {
        samples = (IMU_xgReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
    }*/
    for(ii = 0; ii < samples ; ii++)
    {   // Read the gyro data stored in the FIFO
        IMU_readGyro();
        gBiasRawTemp[0] += Imu.gx;
        gBiasRawTemp[1] += Imu.gy;
        gBiasRawTemp[2] += Imu.gz;
        IMU_readAccel();
        aBiasRawTemp[0] += Imu.ax;
        aBiasRawTemp[1] += Imu.ay;
        aBiasRawTemp[2] += Imu.az;// - (int16_t)(1./Imu.aRes); // Assumes sensor facing up!
        CPUdelay(320000);

    }
    for (ii = 0; ii < 3; ii++)
    {
        Imu.gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
        Imu.gBias[ii] = IMU_calcGyro(Imu.gBiasRaw[ii]);
        Imu.aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
        Imu.aBias[ii] = IMU_calcAccel(Imu.aBiasRaw[ii]);
    }

    IMU_enableFIFO(false);
    IMU_setFIFO(FIFO_OFF, 0x00);

    if (autoCalc) Imu._autoCalc = true;
}

void IMU_calibrateMag( bool loadIn)
{
    int i, j;
    int16_t magMin[3] = {0, 0, 0};
    int16_t magMax[3] = {0, 0, 0}; // The road warrior

    for (i=0; i<128; i++)
    {
        while (!IMU_magAvailable(ALL_AXIS));

        IMU_readMag();
        int16_t magTemp[3] = {0, 0, 0};
        magTemp[0] = Imu.mx;
        magTemp[1] = Imu.my;
        magTemp[2] = Imu.mz;
        for (j = 0; j < 3; j++)
        {
            if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
            if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
        }
    }
    for (j = 0; j < 3; j++)
    {
        Imu.mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
        Imu.mBias[j] = IMU_calcMag(Imu.mBiasRaw[j]);
        if (loadIn)
            IMU_magOffset(j, Imu.mBiasRaw[j]);
    }

}

void IMU_magOffset( uint8_t axis, int16_t offset)
{
    if (axis > 2)
        return;
    uint8_t msb, lsb;
    msb = (offset & 0xFF00) >> 8;
    lsb = offset & 0x00FF;
    IMU_mWriteByte(OFFSET_X_REG_L_M + (2 * axis), lsb);
    IMU_mWriteByte(OFFSET_X_REG_H_M + (2 * axis), msb);
}

void IMU_initMag()
{
    uint8_t tempRegValue = 0;

    // CTRL_REG1_M (Default value: 0x10)
    // [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
    // TEMP_COMP - Temperature compensation
    // OM[1:0] - X & Y axes op mode selection
    //  00:low-power, 01:medium performance
    //  10: high performance, 11:ultra-high performance
    // DO[2:0] - Output data rate selection
    // ST - Self-test enable
    if (Imu.settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
    tempRegValue |= (Imu.settings.mag.XYPerformance & 0x3) << 5;
    tempRegValue |= (Imu.settings.mag.sampleRate & 0x7) << 2;
    IMU_mWriteByte(CTRL_REG1_M, tempRegValue);

    // CTRL_REG2_M (Default value 0x00)
    // [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
    // FS[1:0] - Full-scale configuration
    // REBOOT - Reboot memory content (0:normal, 1:reboot)
    // SOFT_RST - Reset config and user registers (0:default, 1:reset)
    tempRegValue = 0;
    switch (Imu.settings.mag.scale)
    {
    case 8:
        tempRegValue |= (0x1 << 5);
        break;
    case 12:
        tempRegValue |= (0x2 << 5);
        break;
    case 16:
        tempRegValue |= (0x3 << 5);
        break;
    // Otherwise we'll default to 4 gauss (00)
    }
    IMU_mWriteByte(CTRL_REG2_M, tempRegValue); // +/-4Gauss

    // CTRL_REG3_M (Default value: 0x03)
    // [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
    // I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
    // LP - Low-power mode cofiguration (1:enable)
    // SIM - SPI mode selection (0:write-only, 1:read/write enable)
    // MD[1:0] - Operating mode
    //  00:continuous conversion, 01:single-conversion,
    //  10,11: Power-down
    tempRegValue = 0;
    if (Imu.settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
    tempRegValue |= (Imu.settings.mag.operatingMode & 0x3);
    IMU_mWriteByte(CTRL_REG3_M, tempRegValue); // Continuous conversion mode

    // CTRL_REG4_M (Default value: 0x00)
    // [0][0][0][0][OMZ1][OMZ0][BLE][0]
    // OMZ[1:0] - Z-axis operative mode selection
    //  00:low-power mode, 01:medium performance
    //  10:high performance, 10:ultra-high performance
    // BLE - Big/little endian data
    tempRegValue = 0;
    tempRegValue = (Imu.settings.mag.ZPerformance & 0x3) << 2;
    IMU_mWriteByte(CTRL_REG4_M, tempRegValue);

    // CTRL_REG5_M (Default value: 0x00)
    // [0][BDU][0][0][0][0][0][0]
    // BDU - Block data update for magnetic data
    //  0:continuous, 1:not updated until MSB/LSB are read
    tempRegValue = 0;
    IMU_mWriteByte(CTRL_REG5_M, tempRegValue);
}

uint8_t IMU_accelAvailable()
{
    uint8_t status = IMU_xgReadByte(STATUS_REG_1);

    return (status & (1<<0));
}

uint8_t IMU_gyroAvailable()
{
    uint8_t status = IMU_xgReadByte(STATUS_REG_1);

    return ((status & (1<<1)) >> 1);
}

uint8_t IMU_tempAvailable()
{
    uint8_t status = IMU_xgReadByte(STATUS_REG_1);

    return ((status & (1<<2)) >> 2);
}

uint8_t IMU_magAvailable( IMU_axis axis)
{
    uint8_t status;
    status = IMU_mReadByte(STATUS_REG_M);


    return ((status & (1<<axis)) >> axis);
}

void IMU_readAccel()
{
    uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
    if ( IMU_xgReadBytes(OUT_X_L_XL, temp, 6) == 6 ) // Read 6 bytes, beginning at OUT_X_L_XL
    {
        Imu.ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
        Imu.ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
        Imu.az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
//        if (Imu._autoCalc)
//        {
//            Imu.ax -= Imu.aBiasRaw[X_AXIS];
//            Imu.ay -= Imu.aBiasRaw[Y_AXIS];
//            Imu.az -= Imu.aBiasRaw[Z_AXIS];
//        }
    }
}

int16_t IMU_readAccelAxis( IMU_axis axis)
{
    uint8_t temp[2];
    int16_t value;
    if ( IMU_xgReadBytes(OUT_X_L_XL + (2 * axis), temp, 2) == 2)
    {
        value = (temp[1] << 8) | temp[0];

        if (Imu._autoCalc)
            value -= Imu.aBiasRaw[axis];

        return value;
    }
    return 0;
}

void IMU_readMag()
{
    uint8_t temp[6]; // We'll read six bytes from the mag into temp
    if ( IMU_mReadBytes(OUT_X_L_M, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_M
    {
        Imu.mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
        Imu.my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
        Imu.mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
    }
}

int16_t IMU_readMagAxis( IMU_axis axis)
{
    uint8_t temp[2];
    if ( IMU_mReadBytes(OUT_X_L_M + (2 * axis), temp, 2) == 2)
    {
        return (temp[1] << 8) | temp[0];
    }
    return 0;
}

void IMU_readTemp()
{
    uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp
    if ( IMU_xgReadBytes(OUT_TEMP_L, temp, 2) == 2 ) // Read 2 bytes, beginning at OUT_TEMP_L
    {
        Imu.temperature = ((int16_t)temp[1] << 8) | temp[0];
    }
}

void IMU_readGyro()
{
    uint8_t temp[6]; // We'll read six bytes from the gyro into temp
    if ( IMU_xgReadBytes(OUT_X_L_G, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_G
    {
        Imu.gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
        Imu.gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
        Imu.gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
        if (Imu._autoCalc)
        {
            Imu.gx -= Imu.gBiasRaw[X_AXIS];
            Imu.gy -= Imu.gBiasRaw[Y_AXIS];
            Imu.gz -= Imu.gBiasRaw[Z_AXIS];
        }
    }
}

int16_t IMU_readGyroAxis( IMU_axis axis)
{
    uint8_t temp[2];
    int16_t value;

    if ( IMU_xgReadBytes(OUT_X_L_G + (2 * axis), temp, 2) == 2)
    {
        value = (temp[1] << 8) | temp[0];

        if (Imu._autoCalc)
            value -= Imu.gBiasRaw[axis];

        return value;
    }
    return 0;
}

float IMU_calcGyro( int16_t gyro)
{
    // Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
    if(fabs(Imu.gRes*gyro) < 0.5)
        return 0;
    return Imu.gRes * gyro;
}

float IMU_calcAccel( int16_t accel)
{
    // Return the accel raw reading times our pre-calculated g's / (ADC tick):
    if(fabs(Imu.aRes*accel) < 0.078)
        return 0;
    return Imu.aRes * accel;
}

float IMU_calcMag( int16_t mag)
{
    // Return the mag raw reading times our pre-calculated Gs / (ADC tick):
    return Imu.mRes * mag;
}

void IMU_setGyroScale( uint16_t gScl)
{
    // Read current value of CTRL_REG1_G:
    uint8_t ctrl1RegValue = IMU_xgReadByte(CTRL_REG1_G);
    // Mask out scale bits (3 & 4):
    ctrl1RegValue &= 0xE7;
    switch (gScl)
    {
        case 500:
            ctrl1RegValue |= (0x1 << 3);
            Imu.settings.gyro.scale = 500;
            break;
        case 2000:
            ctrl1RegValue |= (0x3 << 3);
            Imu.settings.gyro.scale = 2000;
            break;
        default: // Otherwise we'll set it to 245 dps (0x0 << 4)
            Imu.settings.gyro.scale = 245;
            break;
    }
    IMU_xgWriteByte(CTRL_REG1_G, ctrl1RegValue);

    IMU_calcgRes();
}

void IMU_setAccelScale( uint8_t aScl)
{
    // We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
    uint8_t tempRegValue = IMU_xgReadByte(CTRL_REG6_XL);
    // Mask out accel scale bits:
    tempRegValue &= 0xE7;

    switch (aScl)
    {
        case 4:
            tempRegValue |= (0x2 << 3);
            Imu.settings.accel.scale = 4;
            break;
        case 8:
            tempRegValue |= (0x3 << 3);
            Imu.settings.accel.scale = 8;
            break;
        case 16:
            tempRegValue |= (0x1 << 3);
            Imu.settings.accel.scale = 16;
            break;
        default: // Otherwise it'll be set to 2g (0x0 << 3)
            Imu.settings.accel.scale = 2;
            break;
    }
    IMU_xgWriteByte(CTRL_REG6_XL, tempRegValue);

    // Then calculate a new aRes, which relies on aScale being set correctly:
    IMU_calcaRes();
}

void IMU_setMagScale( uint8_t mScl)
{
    // We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
    uint8_t temp = IMU_mReadByte(CTRL_REG2_M);
    // Then mask out the mag scale bits:
    temp &= 0xFF^(0x3 << 5);

    switch (mScl)
    {
    case 8:
        temp |= (0x1 << 5);
        Imu.settings.mag.scale = 8;
        break;
    case 12:
        temp |= (0x2 << 5);
        Imu.settings.mag.scale = 12;
        break;
    case 16:
        temp |= (0x3 << 5);
        Imu.settings.mag.scale = 16;
        break;
    default: // Otherwise we'll default to 4 gauss (00)
        Imu.settings.mag.scale = 4;
        break;
    }

    // And write the new register value back into CTRL_REG6_XM:
    IMU_mWriteByte(CTRL_REG2_M, temp);

    // We've updated the sensor, but we also need to update our class variables
    // First update mScale:
    //mScale = mScl;
    // Then calculate a new mRes, which relies on mScale being set correctly:
    IMU_calcmRes();
}

void IMU_setGyroODR( uint8_t gRate)
{
    // Only do this if gRate is not 0 (which would disable the gyro)
    if ((gRate & 0x07) != 0)
    {
        // We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
        uint8_t temp = IMU_xgReadByte(CTRL_REG1_G);
        // Then mask out the gyro ODR bits:
        temp &= 0xFF^(0x7 << 5);
        temp |= (gRate & 0x07) << 5;
        // Update our settings struct
        Imu.settings.gyro.sampleRate = gRate & 0x07;
        // And write the new register value back into CTRL_REG1_G:
        IMU_xgWriteByte(CTRL_REG1_G, temp);
    }
}

void IMU_setAccelODR( uint8_t aRate)
{
    // Only do this if aRate is not 0 (which would disable the accel)
    if ((aRate & 0x07) != 0)
    {
        // We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
        uint8_t temp = IMU_xgReadByte(CTRL_REG6_XL);
        // Then mask out the accel ODR bits:
        temp &= 0x1F;
        // Then shift in our new ODR bits:
        temp |= ((aRate & 0x07) << 5);
        Imu.settings.accel.sampleRate = aRate & 0x07;
        // And write the new register value back into CTRL_REG1_XM:
        IMU_xgWriteByte(CTRL_REG6_XL, temp);
    }
}


void IMU_setMagODR( uint8_t mRate)
{
    // We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
    uint8_t temp = IMU_mReadByte(CTRL_REG1_M);
    // Then mask out the mag ODR bits:
    temp &= 0xFF^(0x7 << 2);
    // Then shift in our new ODR bits:
    temp |= ((mRate & 0x07) << 2);
    Imu.settings.mag.sampleRate = mRate & 0x07;
    // And write the new register value back into CTRL_REG5_XM:
    IMU_mWriteByte(CTRL_REG1_M, temp);
}

void IMU_calcgRes()
{
    switch (Imu.settings.gyro.scale)
    {
    case 245:
        Imu.gRes = SENSITIVITY_GYROSCOPE_245;
        break;
    case 500:
        Imu.gRes = SENSITIVITY_GYROSCOPE_500;
        break;
    case 2000:
        Imu.gRes = SENSITIVITY_GYROSCOPE_2000;
        break;
    default:
        break;
    }
}

void IMU_calcaRes()
{
    switch (Imu.settings.accel.scale)
    {
    case 2:
        Imu.aRes = SENSITIVITY_ACCELEROMETER_2;
        //Imu.aRes = 2/ 32768.0;
        break;
    case 4:
        Imu.aRes = SENSITIVITY_ACCELEROMETER_4;
        //Imu.aRes = 4/ 32768.0;
        break;
    case 8:
        Imu.aRes = SENSITIVITY_ACCELEROMETER_8;
        //Imu.aRes = 8/ 32768.0;
        break;
    case 16:
        Imu.aRes = SENSITIVITY_ACCELEROMETER_16;
        //Imu.aRes = 16/ 32768.0;
        break;
    default:
        break;
    }
}

void IMU_calcmRes()
{
    switch (Imu.settings.mag.scale)
    {
    case 4:
        Imu.mRes = SENSITIVITY_MAGNETOMETER_4;
        break;
    case 8:
        Imu.mRes = SENSITIVITY_MAGNETOMETER_8;
        break;
    case 12:
        Imu.mRes = SENSITIVITY_MAGNETOMETER_12;
        break;
    case 16:
        Imu.mRes = SENSITIVITY_MAGNETOMETER_16;
        break;
    }
}


void IMU_configInactivity( uint8_t duration, uint8_t threshold, bool sleepOn)
{
    uint8_t temp = 0;

    temp = threshold & 0x7F;
    if (sleepOn) temp |= (1<<7);
    IMU_xgWriteByte(ACT_THS, temp);

    IMU_xgWriteByte(ACT_DUR, duration);
}

uint8_t IMU_getInactivity()
{
    uint8_t temp = IMU_xgReadByte(STATUS_REG_0);
    temp &= (0x10);
    return temp;
}


void IMU_configAccelInt( uint8_t generator, bool andInterrupts)
{
    // Use variables from accel_interrupt_generator, OR'd together to create
    // the [generator]value.
    uint8_t temp = generator;
    if (andInterrupts) temp |= 0x80;
    IMU_xgWriteByte(INT_GEN_CFG_XL, temp);
}

void IMU_configAccelThs( uint8_t threshold, IMU_axis axis, uint8_t duration, bool wait)
{
    // Write threshold value to INT_GEN_THS_?_XL.
    // axis will be 0, 1, or 2 (x, y, z respectively)
    IMU_xgWriteByte(INT_GEN_THS_X_XL + axis, threshold);

    // Write duration and wait to INT_GEN_DUR_XL
    uint8_t temp;
    temp = (duration & 0x7F);
    if (wait) temp |= 0x80;
    IMU_xgWriteByte(INT_GEN_DUR_XL, temp);
}

uint8_t IMU_getAccelIntSrc()
{
    uint8_t intSrc = IMU_xgReadByte(INT_GEN_SRC_XL);

    // Check if the IA_XL (interrupt active) bit is set
    if (intSrc & (1<<6))
    {
        return (intSrc & 0x3F);
    }

    return 0;
}

void IMU_configGyroInt( uint8_t generator, bool aoi, bool latch)
{
    // Use variables from accel_interrupt_generator, OR'd together to create
    // the [generator]value.
    uint8_t temp = generator;
    if (aoi) temp |= 0x80;
    if (latch) temp |= 0x40;
    IMU_xgWriteByte(INT_GEN_CFG_G, temp);
}

void IMU_configGyroThs( int16_t threshold, IMU_axis axis, uint8_t duration, bool wait)
{
    uint8_t buffer[2];
    buffer[0] = (threshold & 0x7F00) >> 8;
    buffer[1] = (threshold & 0x00FF);
    // Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
    // axis will be 0, 1, or 2 (x, y, z respectively)
    IMU_xgWriteByte(INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
    IMU_xgWriteByte(INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);

    // Write duration and wait to INT_GEN_DUR_XL
    uint8_t temp;
    temp = (duration & 0x7F);
    if (wait) temp |= 0x80;
    IMU_xgWriteByte(INT_GEN_DUR_G, temp);
}

uint8_t IMU_getGyroIntSrc()
{
    uint8_t intSrc = IMU_xgReadByte(INT_GEN_SRC_G);

    // Check if the IA_G (interrupt active) bit is set
    if (intSrc & (1<<6))
    {
        return (intSrc & 0x3F);
    }

    return 0;
}



void IMU_configMagThs( uint16_t threshold)
{
    // Write high eight bits of [threshold] to INT_THS_H_M
    IMU_mWriteByte(INT_THS_H_M, (uint8_t)((threshold & 0x7F00) >> 8));
    // Write low eight bits of [threshold] to INT_THS_L_M
    IMU_mWriteByte(INT_THS_L_M, (uint8_t)(threshold & 0x00FF));
}

uint8_t IMU_getMagIntSrc()
{
    uint8_t intSrc = IMU_mReadByte(INT_SRC_M);

    // Check if the INT (interrupt active) bit is set
    if (intSrc & (1<<0))
    {
        return (intSrc & 0xFE);
    }

    return 0;
}

void IMU_sleepGyro( bool enable)
{
    uint8_t temp = IMU_xgReadByte(CTRL_REG9);
    if (enable) temp |= (1<<6);
    else temp &= ~(1<<6);
    IMU_xgWriteByte(CTRL_REG9, temp);
}

void IMU_enableFIFO( bool enable)
{
    uint8_t temp = IMU_xgReadByte(CTRL_REG9);
    if (enable) temp |= (1<<1);
    else temp &= ~(1<<1);
    IMU_xgWriteByte(CTRL_REG9, temp);
}

void IMU_setFIFO( fifoMode_type fifoMode, uint8_t fifoThs)
{
    // Limit threshold - 0x1F (31) is the maximum. If more than that was asked
    // limit it to the maximum.
    uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
    IMU_xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t IMU_getFIFOSamples()
{
    return (IMU_xgReadByte(FIFO_SRC) & 0x3F);
}

void IMU_constrainScales()
{
    if ((Imu.settings.gyro.scale != 245) && (Imu.settings.gyro.scale != 500) &&
        (Imu.settings.gyro.scale != 2000))
    {
        Imu.settings.gyro.scale = 245;
    }

    if ((Imu.settings.accel.scale != 2) && (Imu.settings.accel.scale != 4) &&
        (Imu.settings.accel.scale != 8) && (Imu.settings.accel.scale != 16))
    {
        Imu.settings.accel.scale = 2;
    }

    if ((Imu.settings.mag.scale != 4) && (Imu.settings.mag.scale != 8) &&
        (Imu.settings.mag.scale != 12) && (Imu.settings.mag.scale != 16))
    {
        Imu.settings.mag.scale = 4;
    }
}

void IMU_xgWriteByte( uint8_t subAddress, uint8_t data){
    return writeByte_I2C(Imu._xgAddress, subAddress, data);
}

void IMU_mWriteByte( uint8_t subAddress, uint8_t data){
    return writeByte_I2C(Imu._mAddress, subAddress, data);
}

uint8_t IMU_xgReadByte( uint8_t subAddress){
    return readByte_I2C(Imu._xgAddress, subAddress);
}

uint8_t IMU_xgReadBytes( uint8_t subAddress, uint8_t * dest, uint8_t count){
    return readBytes_I2C(Imu._xgAddress, subAddress | 0x80, dest, count);
}

uint8_t IMU_mReadByte( uint8_t subAddress){
    return readByte_I2C(Imu._mAddress, subAddress);
}

uint8_t IMU_mReadBytes( uint8_t subAddress, uint8_t * dest, uint8_t count){
    return readBytes_I2C(Imu._mAddress, subAddress | 0x80, dest, count);
}
