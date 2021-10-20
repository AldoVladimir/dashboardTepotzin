/************************************************************
 * Gpio.h  Created on: 28/01/2019                           *
 *                                                          *
 *      Universidad Nacional Autonoma de Mexico             *
 *          Instituto de Ciencias Nucleares                 *
 *     Laboratorio de Instrumentacion Espacial LINX         *
 *                                                          *
 *  Desarrollador: Ing. Manuel Andres Herrera Juarez.       *
 *                                                          *
 ************************************************************/


#ifndef PERIPHERALS_GPIO_H_
#define PERIPHERALS_GPIO_H_

#include <stdint.h>
#include <driverlib/prcm.h>
#include <driverlib/ioc.h>
#include <driverlib/gpio.h>

#ifdef __cplusplus
extern "C"
{
#endif

void InitGPIO();
void setDigitalInput(uint32_t pin);
void setDigitalOutput(uint32_t pin);
void digitalWrite(uint32_t pin, bool value);


#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_GPIO_H_ */
