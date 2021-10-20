/************************************************************
 * Gpio.c  Created on: 28/01/2019                           *
 *                                                          *
 *      Universidad Nacional Autonoma de Mexico             *
 *          Instituto de Ciencias Nucleares                 *
 *     Laboratorio de Instrumentacion Espacial LINX         *
 *                                                          *
 *  Desarrollador: Ing. Manuel Andres Herrera Juarez.       *
 *                                                          *
 ************************************************************/


#include "Gpio.h"

/*
 * Funcion que inicializa los puertos de entrada/salida
 *
 */
void InitGPIO(){
    //Habilita dominio de poder a los perifericos. GPIO pertenece a este dominio.
    if(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)!=PRCM_DOMAIN_POWER_ON){
        PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
        while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)!=PRCM_DOMAIN_POWER_ON);
    }

    //Habilita señal de reloj a GPIO y actualiza cambios.
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
}


/*
 * Funcion que establece un pin como entrada
 *
 * pin  =   Entrada digital
 *          IOID_0 a IOID_31
 */
void setDigitalInput(uint32_t pin){
    IOCPinTypeGpioInput(pin);
    GPIO_setOutputEnableDio(pin, GPIO_OUTPUT_DISABLE);
}


/*
 * Funcion que establece un pin como salida
 *
 * pin  =   Salida digital
 *          IOID_0 a IOID_31
 */
void setDigitalOutput(uint32_t pin){
    IOCPinTypeGpioOutput(pin);
    GPIO_setOutputEnableDio(pin, GPIO_OUTPUT_ENABLE);
}


/*
 * Funcion escribe un valor booleano a un pin de salida
 *
 * pin  =   Salida digital
 *          IOID_0 a IOID_31
 *
 * value    =   Valor a escribir
 *              1 true
 *              0 false
 */
void digitalWrite(uint32_t pin, bool value){
    GPIO_writeDio(pin, value);
}
