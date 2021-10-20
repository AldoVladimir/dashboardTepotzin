/************************************************************
 * Timer.h  Created on: 28/01/2019                          *
 *                                                          *
 *      Universidad Nacional Autonoma de Mexico             *
 *          Instituto de Ciencias Nucleares                 *
 *     Laboratorio de Instrumentacion Espacial LINX         *
 *                                                          *
 *  Desarrollador: Ing. Manuel Andres Herrera Juarez.       *
 *                                                          *
 ************************************************************/

#ifndef PERIPHERALS_TIMER_H_
#define PERIPHERALS_TIMER_H_

#include <stdint.h>
#include <driverlib/timer.h>
#include <driverlib/prcm.h>


#ifdef __cplusplus
extern "C"
{
#endif


void init16BitTimer(uint32_t baseTimer, uint32_t subTimer, uint32_t preescale, uint32_t freq, void (*TimerInt_Handler)(void));

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_TIMER_H_ */
