/************************************************************
 * Adc.c  Created on: 28/01/2019                            *
 *                                                          *
 *      Universidad Nacional Autonoma de Mexico             *
 *          Instituto de Ciencias Nucleares                 *
 *     Laboratorio de Instrumentacion Espacial LINX         *
 *                                                          *
 *  Desarrollador: Ing. Manuel Andres Herrera Juarez.       *
 *                                                          *
 ************************************************************/

#include "Adc.h"

/*
 * Funcion usada para inicializar el conversor Analogico-Digital
 *
 * pin  =   Pin de entrada donde se leera la señal ADC
 *          IOID_23
 *          IOID_24
 *          .
 *          .
 *          .
 *          IOID_30
 *
 */
void initADC(uint32_t pin){
    //Habilia el reloj para los perifericos Auxiliares y el conversor ADC
    AUXWUCClockEnable(AUX_WUC_ANAIF_CLOCK|AUX_WUC_ADI_CLOCK);
    AUXWUCClockEnable(AUX_WUC_ADC_CLOCK);

    //Espera a que la seÃ±al de reloj llegue al periferico
    while(AUXWUCClockStatus(AUX_WUC_ADC_CLOCK)==AUX_WUC_CLOCK_OFF);

    //Mapea el pin elegido al numero de conversor ADC a AUXIO
    uint32_t auxIO = IOC_PORT_AUX_IO;
    if(pin>=IOID_23 && pin<=IOID_30){
        auxIO= 7-(30-pin) + 9;
    }else{
        return;
    }

    //Selecciona el conversor como default
    AUXADCSelectInput(auxIO);

    //Configura el pin de salida como entrada analogica
    IOCPortConfigureSet(pin, auxIO, IOC_INPUT_DISABLE);

    //Configura ADC con un trigger Manual, maximo 200 Kmuestras/s
    AUXADCEnableSync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);
}


/*
 * Funcion que obtiene el valor de la señal analogica
 * retorna un entero sin signo de 0 a 4095
 *
 * pin  =   Pin de entrada donde se leera la señal ADC
 *          IOID_23
 *          IOID_24
 *          .
 *          .
 *          .
 *          IOID_30
 */
uint32_t shotADC(uint32_t pin){
    //Mapea el pin elegido al numero de conversor ADC a AUXIO
    uint32_t auxIO = IOC_PORT_AUX_IO;
    if(pin>=IOID_23 && pin<=IOID_30){
        auxIO= 7-(30-pin) + 9;
    }else{
        return 0;
    }

    //Selecciona como ADC el pin.
    AUXADCSelectInput(auxIO);

    AUXADCGenManualTrigger();
    while(AUXADCGetFifoStatus()==AUXADC_FIFO_EMPTY_M);
    return AUXADCPopFifo();
}


/*
 * Funcion que obtiene el valor de la señal analogica
 * en microvolts. Retorna un entero sin signo de 32 bits
 *
 */
int32_t shotADCMicroVolts(uint32_t pin){
    return AUXADCValueToMicrovolts(AUXADC_FIXED_REF_VOLTAGE_NORMAL, shotADC(pin));
    //return AUXADCValueToMicrovolts(AUXADC_FIXED_REF_VOLTAGE_UNSCALED, shotADC(pin));
}


/*
 * Funcion que obtiene el valor de la señal analogica
 * a milivolts. Retorna un valor flotante
 *
 * pin  =   Pin de entrada donde se leera la señal ADC
 *          IOID_23
 *          IOID_24
 *          .
 *          .
 *          .
 *          IOID_30
 */
float shotADCMiliVolts(uint32_t pin){
    return ((float)shotADCMicroVolts(pin))*0.001;
}


/*
 * Funcion que obtiene el valor de la señal analogica
 * en Volts. Retorna un valor flotante
 *
 * pin  =   Pin de entrada donde se leera la señal ADC
 *          IOID_23
 *          IOID_24
 *          .
 *          .
 *          .
 *          IOID_30
 */
float shotADCVolts(uint32_t pin){
    return ((float)shotADCMicroVolts(pin))*0.000001;
}


