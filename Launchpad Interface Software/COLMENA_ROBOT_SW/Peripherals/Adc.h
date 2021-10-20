/************************************************************
 * Adc.h  Created on: 28/01/2019                            *
 *                                                          *
 *      Universidad Nacional Autonoma de Mexico             *
 *          Instituto de Ciencias Nucleares                 *
 *     Laboratorio de Instrumentacion Espacial LINX         *
 *                                                          *
 *  Desarrollador: Ing. Manuel Andres Herrera Juarez.       *
 *                                                          *
 ************************************************************/
#ifndef PERIPHERALS_ADC_H_
#define PERIPHERALS_ADC_H_

#include <stdint.h>
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
#include <driverlib/ioc.h>

#ifdef __cplusplus
extern "C"
{
#endif


void initADC(uint32_t pin);
uint32_t shotADC(uint32_t pin);
int32_t shotADCMicroVolts(uint32_t pin);
float shotADCMiliVolts(uint32_t pin);
float shotADCVolts(uint32_t pin);

#ifdef __cplusplus
}
#endif


#endif /* PERIPHERALS_ADC_H_ */
