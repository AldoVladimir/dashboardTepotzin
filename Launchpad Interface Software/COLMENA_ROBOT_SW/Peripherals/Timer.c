/************************************************************
 * Timer.c  Created on: 28/01/2019                          *
 *                                                          *
 *      Universidad Nacional Autonoma de Mexico             *
 *          Instituto de Ciencias Nucleares                 *
 *     Laboratorio de Instrumentacion Espacial LINX         *
 *                                                          *
 *  Desarrollador: Ing. Manuel Andres Herrera Juarez.       *
 *                                                          *
 ************************************************************/

#include "Timer.h"

/*
 * Funcion que configura e inicia un SubTimer de cualquier GPT en modo de 16 bits
 * baseTimer =  Contador Base
 *              GPT0_BASE
 *              GPT1_BASE
 *              GPT2_BASE
 *              GPT3_BASE
 *
 * subTimer =   SubContador
 *              TIMER_A
 *              TIMER_B
 *
 * preescale = Escala del contador
 *             Entero sin signo de 0 a 255
 *
 *             preescale=(freq/frecuencia deseada)-1
 *
 * freq =       Frecuencia en kilohertz
 *              Entero sin signo
 *              Frecuencia final = (freq)/(preescale + 1)
 *              si preescale=0 Frecuencia final = freq
 *
 * TimerInt_Handler =   Funcion de interrupcion que es llamada cuando
 *                      el timer finaliza la cuenta 0
 *
 * */
void init16BitTimer(uint32_t baseTimer, uint32_t subTimer, uint32_t preescale, uint32_t freq, void (*TimerInt_Handler)(void)){
    //Obten numero de Timer a partir de la direccion del registro
    uint32_t numOfTimer = (baseTimer - 0x40010000)>>12;

    //Verifica que el Timer exista
    if (numOfTimer > 3 || (subTimer != TIMER_A && subTimer != TIMER_B)){
        return;
    }

    //Calcula la cuenta a la que se reinicia el contador
    //uint32_t count = (65535)/(freq*(preescale+1));
    uint32_t count = (24000/freq);

    //Habilita el dominio de poder a los perifericos. Los timers pertenecen a dicho domino.
    if(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)!=PRCM_DOMAIN_POWER_ON){
       PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
       while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)!=PRCM_DOMAIN_POWER_ON);
    }

    //Habilita seï¿½al de reloj al Timer seleccionado y guarda ajustes
    PRCMPeripheralRunEnable(numOfTimer);
    PRCMPeripheralSleepEnable(numOfTimer);
    PRCMPeripheralDeepSleepEnable(numOfTimer);
    PRCMLoadSet();

    //Deshabilita Subtimer y configura como periodicos
    if(subTimer == TIMER_A){
        HWREG(baseTimer + GPT_O_CTL) &= ~(0x001);
        HWREG(baseTimer + GPT_O_TAMR) = 0x002;
    }else{
        HWREG(baseTimer + GPT_O_CTL) &= ~(0x100);
        HWREG(baseTimer + GPT_O_TBMR) = 0x002;
    }

    //Divide el timer en 2 de 16 bits con un preescalador de 8 bits
    HWREG(baseTimer + GPT_O_CFG) = 0x04;

    //Establece cuenta de reinicio y preescalador
    TimerLoadSet(baseTimer, subTimer, count);
    TimerPrescaleSet(baseTimer, subTimer, preescale);
    TimerPrescaleMatchSet(baseTimer, subTimer, 0);

    //Registra y Habilita la interrupcion al reinicio del timer para ejecutar la funcion TimerInt_Handler
    TimerIntRegister(baseTimer, subTimer, TimerInt_Handler);
    if(subTimer == TIMER_A){
        TimerIntEnable(baseTimer, TIMER_TIMA_TIMEOUT);
    }else{
        TimerIntEnable(baseTimer, TIMER_TIMB_TIMEOUT);
    }

    //Habilita Subtimer
    if(subTimer == TIMER_A){
        HWREG(baseTimer + GPT_O_CTL) |= 0x001;
    }else{
        HWREG(baseTimer + GPT_O_CTL) |= 0x100;
    }
}


