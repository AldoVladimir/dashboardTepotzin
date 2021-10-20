/************************************************************
 * LSM9DS1.h  Created on: 28/01/2019				    	*
 *															*
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
*   SFE_LSM9DS1.h                                                                           *
*                                                                                           *
*   SFE_LSM9DS1 Library Header File                                                         *
*       Jim Lindblom @ SparkFun Electronics                                                 *
*       Original Creation Date: February 27, 2015                                           *
*       https://github.com/sparkfun/LSM9DS1_Breakout                                        *
*                                                                                           *
*   This file prototypes the LSM9DS1 class, implemented in SFE_LSM9DS1.cpp. In              *
*   addition, it defines every register in the LSM9DS1 (both the Gyro and Accel/            *
*   Magnetometer registers).                                                                *
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

#ifndef _TEPOTZIN_LSM9DS1_LSM9DS1_H_
#define _TEPOTZIN_LSM9DS1_LSM9DS1_H_

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_prcm.h"
#include "inc/hw_ioc.h"
#include "driverlib/i2c.h"
#include "driverlib/ioc.h"
#include "driverlib/gpio.h"
#include "driverlib/prcm.h"
#include "driverlib/sys_ctrl.h"
#include "Registers.h"
#include "Types.h"
//#include "Tepitzin/Control.h"

#include <math.h>

#include "../../Peripherals/I2c.h"

#define IMU_AG_ADDR(sa0)    ((sa0) == 0 ? 0x6A : 0x6B)
#define IMU_M_ADDR(sa1)     ((sa1) == 0 ? 0x1C : 0x1E)
#define dt 0.01;



#ifdef __cplusplus //Si se usa el compilador de C++
extern "C"  //Define las siguientes funciones como codigo C externo
{
#endif

typedef enum IMU_axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ALL_AXIS
}IMU_axis;

struct LSM9DS1{
    IMUSettings settings;
    // We'll store the gyro, accel, and magnetometer readings in a series of
    // public class variables. Each sensor gets three variables -- one for each
    // axis. Call readGyro(), readAccel(), and readMag() first, before using
    // these variables!
    // These values are the RAW signed 16-bit readings from the sensors.
    int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
    int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
    int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
    int16_t temperature; // Chip temperature
    float gBias[3], aBias[3], mBias[3];
    int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

     // x_mAddress and gAddress store the I2C address or SPI chip select pin
    // for each sensor.
    uint8_t _mAddress, _xgAddress;

    // gRes, aRes, and mRes store the current resolution for each sensor.
    // Units of these values would be DPS (or g's or Gs's) per ADC tick.
    // This value is calculated as (sensor scale) / (2^15).
    float gRes, aRes, mRes;

    // _autoCalc keeps track of whether we're automatically subtracting off
    // accelerometer and gyroscope bias calculated in calibrate().
    bool _autoCalc;
    
}Imu;

// begin() -- Initialize the gyro, accelerometer, and magnetometer.
// This will set up the scale and output rate of each sensor. The values set
// in the IMUSettings struct will take effect after calling this function.
uint16_t IMU_begin();

void IMU_calibrate(bool autoCalc);
void IMU_calibrateMag(bool loadIn);
void IMU_magOffset(uint8_t axis, int16_t offset);

// accelAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Output:  1 - New data available
//          0 - No new data available
uint8_t IMU_accelAvailable();

// gyroAvailable() -- Polls the gyroscope status register to check
// if new data is available.
// Output:  1 - New data available
//          0 - No new data available
uint8_t IMU_gyroAvailable();

// tempAvailable() -- Polls the temperature status register to check
// if new data is available.
// Output:  1 - New data available
//          0 - No new data available
uint8_t IMU_tempAvailable();

// magAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Input:
//  - axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
//    on one specific axis. Or ALL_AXIS (default) to check for new data
//    on all axes.
// Output:  1 - New data available
//          0 - No new data available
uint8_t IMU_magAvailable(IMU_axis axis);

// readGyro() -- Read the gyroscope output registers.
// This function will read all six gyroscope output registers.
// The readings are stored in the class' gx, gy, and gz variables. Read
// those _after_ calling readGyro().
void IMU_readGyro();

// int16_t readGyro(axis) -- Read a specific axis of the gyroscope.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//  - axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//  A 16-bit signed integer with sensor data on requested axis.
int16_t IMU_readGyroAxis(IMU_axis axis);

// readAccel() -- Read the accelerometer output registers.
// This function will read all six accelerometer output registers.
// The readings are stored in the class' ax, ay, and az variables. Read
// those _after_ calling readAccel().
void IMU_readAccel();

// int16_t readAccel(axis) -- Read a specific axis of the accelerometer.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//  - axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//  A 16-bit signed integer with sensor data on requested axis.

int16_t IMU_readAccelAxis(IMU_axis axis);

// readMag() -- Read the magnetometer output registers.
// This function will read all six magnetometer output registers.
// The readings are stored in the class' mx, my, and mz variables. Read
// those _after_ calling readMag().
void IMU_readMag();

// int16_t readMag(axis) -- Read a specific axis of the magnetometer.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//  - axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//  A 16-bit signed integer with sensor data on requested axis.
int16_t IMU_readMagAxis(IMU_axis axis);

// readTemp() -- Read the temperature output register.
// This function will read two temperature output registers.
// The combined readings are stored in the class' temperature variables. Read
// those _after_ calling readTemp().
void IMU_readTemp();

// calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
// This function reads in a signed 16-bit value and returns the scaled
// DPS. This function relies on gScale and gRes being correct.
// Input:
//  - gyro = A signed 16-bit raw reading from the gyroscope.
float IMU_calcGyro(int16_t gyro);

// calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
// This function reads in a signed 16-bit value and returns the scaled
// g's. This function relies on aScale and aRes being correct.
// Input:
//  - accel = A signed 16-bit raw reading from the accelerometer.
float IMU_calcAccel(int16_t accel);

// calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
// This function reads in a signed 16-bit value and returns the scaled
// Gs. This function relies on mScale and mRes being correct.
// Input:
//  - mag = A signed 16-bit raw reading from the magnetometer.
float IMU_calcMag(int16_t mag);

// setGyroScale() -- Set the full-scale range of the gyroscope.
// This function can be called to set the scale of the gyroscope to
// 245, 500, or 200 degrees per second.
// Input:
//  - gScl = The desired gyroscope scale. Must be one of three possible
//      values from the gyro_scale.
void IMU_setGyroScale(uint16_t gScl);

// setAccelScale() -- Set the full-scale range of the accelerometer.
// This function can be called to set the scale of the accelerometer to
// 2, 4, 6, 8, or 16 g's.
// Input:
//  - aScl = The desired accelerometer scale. Must be one of five possible
//      values from the accel_scale.
void IMU_setAccelScale(uint8_t aScl);

// setMagScale() -- Set the full-scale range of the magnetometer.
// This function can be called to set the scale of the magnetometer to
// 2, 4, 8, or 12 Gs.
// Input:
//  - mScl = The desired magnetometer scale. Must be one of four possible
//      values from the mag_scale.
void IMU_setMagScale(uint8_t mScl);

// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
// Input:
//  - gRate = The desired output rate and cutoff frequency of the gyro.
void IMU_setGyroODR(uint8_t gRate);

// setAccelODR() -- Set the output data rate of the accelerometer
// Input:
//  - aRate = The desired output rate of the accel.
void IMU_setAccelODR(uint8_t aRate);

// setMagODR() -- Set the output data rate of the magnetometer
// Input:
//  - mRate = The desired output rate of the mag.
void IMU_setMagODR(uint8_t mRate);

// configInactivity() -- Configure inactivity interrupt parameters
// Input:
//  - duration = Inactivity duration - actual value depends on gyro ODR
//  - threshold = Activity Threshold
//  - sleepOn = Gyroscope operating mode during inactivity.
//    true: gyroscope in sleep mode
//    false: gyroscope in power-down
void IMU_configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn);

// configAccelInt() -- Configure Accelerometer Interrupt Generator
// Input:
//  - generator = Interrupt axis/high-low events
//    Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
//  - andInterrupts = AND/OR combination of interrupt events
//    true: AND combination
//    false: OR combination
void IMU_configAccelInt(uint8_t generator, bool andInterrupts);

// configAccelThs() -- Configure the threshold of an accelereomter axis
// Input:
//  - threshold = Interrupt threshold. Possible values: 0-255.
//    Multiply by 128 to get the actual raw accel value.
//  - axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//  - duration = Duration value must be above or below threshold to trigger interrupt
//  - wait = Wait function on duration counter
//    true: Wait for duration samples before exiting interrupt
//    false: Wait function off
void IMU_configAccelThs(uint8_t threshold, IMU_axis axis, uint8_t duration, bool wait);

// configGyroInt() -- Configure Gyroscope Interrupt Generator
// Input:
//  - generator = Interrupt axis/high-low events
//    Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
//  - aoi = AND/OR combination of interrupt events
//    true: AND combination
//    false: OR combination
//  - latch: latch gyroscope interrupt request.
void IMU_configGyroInt(uint8_t generator, bool aoi, bool latch);

// configGyroThs() -- Configure the threshold of a gyroscope axis
// Input:
//  - threshold = Interrupt threshold. Possible values: 0-0x7FF.
//    Value is equivalent to raw gyroscope value.
//  - axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//  - duration = Duration value must be above or below threshold to trigger interrupt
//  - wait = Wait function on duration counter
//    true: Wait for duration samples before exiting interrupt
//    false: Wait function off
void IMU_configGyroThs(int16_t threshold, IMU_axis axis, uint8_t duration, bool wait);


// configMagThs() -- Configure the threshold of a gyroscope axis
// Input:
//  - threshold = Interrupt threshold. Possible values: 0-0x7FF.
//    Value is equivalent to raw magnetometer value.
void IMU_configMagThs(uint16_t threshold);

// getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
uint8_t IMU_getGyroIntSrc();

// getGyroIntSrc() -- Get contents of accelerometer interrupt source register
uint8_t IMU_getAccelIntSrc();

// getGyroIntSrc() -- Get contents of magnetometer interrupt source register
uint8_t IMU_getMagIntSrc();

// getGyroIntSrc() -- Get status of inactivity interrupt
uint8_t IMU_getInactivity();

// sleepGyro() -- Sleep or wake the gyroscope
// Input:
//  - enable: True = sleep gyro. False = wake gyro.
void IMU_sleepGyro(bool enable);

// enableFIFO() - Enable or disable the FIFO
// Input:
//  - enable: true = enable, false = disable.
void IMU_enableFIFO(bool enable);

// setFIFO() - Configure FIFO mode and Threshold
// Input:
//  - fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
//    Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
//  - fifoThs: FIFO threshold level setting
//    Any value from 0-0x1F is acceptable.
void IMU_setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);

// getFIFOSamples() - Get number of FIFO samples
uint8_t IMU_getFIFOSamples();


// init() -- Sets up gyro, accel, and mag settings to default.
// - xgAddr - Sets either the I2C address of the accel/gyro or SPI chip
//   select pin connected to the CS_XG pin.
// - mAddr - Sets either the I2C address of the magnetometer or SPI chip
//   select pin connected to the CS_M pin.
void IMU_init(uint8_t xgAddr, uint8_t mAddr);

// initGyro() -- Sets up the gyroscope to begin reading.
// This function steps through all five gyroscope control registers.
// Upon exit, the following parameters will be set:
//  - CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled.
//      95 Hz ODR, 12.5 Hz cutoff frequency.
//  - CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
//      set to 7.2 Hz (depends on ODR).
//  - CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
//      active high). Data-ready output enabled on DRDY_G.
//  - CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
//      address. Scale set to 245 DPS. SPI mode set to 4-wire.
//  - CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
void IMU_initGyro();

// initAccel() -- Sets up the accelerometer to begin reading.
// This function steps through all accelerometer related control registers.
// Upon exit these registers will be set as:
//  - CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
//  - CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
//      all axes enabled.
//  - CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
//  - CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
void IMU_initAccel();

// initMag() -- Sets up the magnetometer to begin reading.
// This function steps through all magnetometer-related control registers.
// Upon exit these registers will be set as:
//  - CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
//  - CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
//      requests don't latch. Temperature sensor disabled.
//  - CTRL_REG6_XM = 0x00:  2 Gs scale.
//  - CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
//  - INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
void IMU_initMag();

// gReadByte() -- Reads a byte from a specified gyroscope register.
// Input:
//  - subAddress = Register to be read from.
// Output:
//  - An 8-bit value read from the requested address.
uint8_t IMU_mReadByte(uint8_t subAddress);

// gReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the gyroscope.
// Input:
//  - subAddress = Register to be read from.
//  - * dest = A pointer to an array of uint8_t's. Values read will be
//      stored in here on return.
//  - count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
//  the data read upon exit.
uint8_t IMU_mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

// gWriteByte() -- Write a byte to a register in the gyroscope.
// Input:
//  - subAddress = Register to be written to.
//  - data = data to be written to the register.
void IMU_mWriteByte(uint8_t subAddress, uint8_t data);

// xmReadByte() -- Read a byte from a register in the accel/mag sensor
// Input:
//  - subAddress = Register to be read from.
// Output:
//  - An 8-bit value read from the requested register.
uint8_t IMU_xgReadByte(uint8_t subAddress);

// xmReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the accelerometer/magnetometer.
// Input:
//  - subAddress = Register to be read from.
//  - * dest = A pointer to an array of uint8_t's. Values read will be
//      stored in here on return.
//  - count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
//  the data read upon exit.
uint8_t IMU_xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
// Input:
//  - subAddress = Register to be written to.
//  - data = data to be written to the register.
void IMU_xgWriteByte(uint8_t subAddress, uint8_t data);

// calcgRes() -- Calculate the resolution of the gyroscope.
// This function will set the value of the gRes variable. gScale must
// be set prior to calling this function.
void IMU_calcgRes();

// calcmRes() -- Calculate the resolution of the magnetometer.
// This function will set the value of the mRes variable. mScale must
// be set prior to calling this function.
void IMU_calcmRes();

// calcaRes() -- Calculate the resolution of the accelerometer.
// This function will set the value of the aRes variable. aScale must
// be set prior to calling this function.
void IMU_calcaRes();

//////////////////////
// Helper Functions //
//////////////////////
void IMU_constrainScales();


/*
 * Funcion que realiza el control PID
 */
extern void motors_Control_PID_Controller();


#ifdef __cplusplus //Si se usa el compilador de C++
}
#endif

#endif
