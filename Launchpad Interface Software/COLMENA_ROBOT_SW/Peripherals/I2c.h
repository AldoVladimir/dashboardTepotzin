/************************************************************
 * I2c.h  Created on: 29/01/2019							*
 *															*
 *      Universidad Nacional Autonoma de Mexico             *
 *          Instituto de Ciencias Nucleares                 *
 *     Laboratorio de Instrumentacion Espacial LINX         *
 *                                                          *
 *  Desarrollador: Ing. Manuel Andres Herrera Juarez.       *
 *                                                          *
 ************************************************************/


#ifndef PERIPHERALS_I2C_H_
#define PERIPHERALS_I2C_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "driverlib/i2c.h"
#include "driverlib/ioc.h"
#include "driverlib/gpio.h"
#include "driverlib/prcm.h"
#include "driverlib/cpu.h"
#include "Gpio.h"

#ifdef __cplusplus
extern "C"{
#endif

void initI2C(uint32_t pin_SDA, uint32_t pin_SCL);
void writeByte_I2C(uint8_t device_Address, uint8_t device_Register, uint8_t data);
uint8_t readByte_I2C(uint8_t device_Address, uint8_t device_Register);
uint8_t readBytes_I2C(uint8_t device_Address, uint8_t device_Register, uint8_t* out_Buffer, uint8_t size);


#ifdef __cplusplus
}
#endif
#endif /* PERIPHERALS_I2C_H_ */
