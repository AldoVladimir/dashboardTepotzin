#include "NavBoot.h"


void NavBoot(struct NavState * NavS){

    /*
     * Estado Boot de la máquina de estados. Es el primer estado en entrar. Levanta todos los periféricos del
     * microcontrolador y espera hasta que se haga una primera detección de la señal de otro robot.
     */

    sleep(TIME_TO_ON_SEC);                      // Cada vez que se reinicie el robot, se tomará "TIME_TO_ON_SEC" segundos para esperar a que el capacitor se cargue

    // ----- Inicia secuencia de booteo, levanta todos los módulos -----

    GPIO_init();                                // Inicializa los GPIOS para su uso con la memoria externa
    SPI_init();                                 // Inicializa el manejador del periférico SPI para comunicación con la memoria y el DAC

    Init_CoilRx();                              // Inicializa la bobina receptora
    Init_CoilTx();                              // Inicializa la bobina emisora

#if FST_TIME_MEM_WRITE
    WriteDefaultParametersInMemory();           // Si es la primera vez que se programa al robot, se escriben los parámetros de defecto en la memoria
#endif
    UpdateParametersFromMemory(DEFAULT_PARAMS); // Cuando bootea o rebootea el robot, usa siempre los parámetros de navegación de default

    CoilRxMeasurement(NavS);                    // Obtiene el tamaño de cluster inicial
    NavS->LCS = NavS->ACS;                      // El último tamaño de cluster es el mismo

    Init_Motors();                              // Inicializa los motores del robot

    InitSensors();                              // Inicializa los sensores del robot
    // IsysCalibration();                          // Realiza la calibración de la corriente del sistema.

    InitRF();                                   // Inicializa el módulo de RF
    // RFCalibration();                            // Realiza la calibración del módulo RF

    InitCommsTimer();                           // Inicializa el timer de transmisión de telemetría al TTDM

    // WaitForFirstDetection(CS_to_CHtype(NavS->ACS)); // Espera hasta que se detecte a algún otro robot activado. Si ya se había activado antes, no busca nada
}

void WaitForFirstDetection(enum Ch_type acs_ch){

    /*
     * Esta función comprueba si ya se ha realizado una primera detección de la señal de otro robot.
     * Si el robot se reinició es posible que ya haya detectado a algún otro, por lo cual no espera
     * y continua con el resto de la ejecución.
     * FD = 1 (significa que ya se había realizado la detección con anterioridad)
     * FD = 0 (significa que no se ha realizado la detección nunca)
     */

    uint8_t FD[1] = {0};                            // Variable para leer parámetro FIRST DETECTION
    ReadFlashMemory("FIRST_DET", FD, 1);            // Lee el parámetro

    if(FD[0]){                                      // Si ya se había realizado la detección anteriormente
        return;                                     // Sale de la función y no espera más
    }
    else{                                           // Si no:
        //while(!IsOtherRobotActivated());            // Espera hasta que detecte cualquier otro robot activado
        while(1){

            if(IsOtherRobotActivated()){            // Si hay otro robot activado
                break;                              // Sale del ciclo
            }else{                                  // Si no:
                //SetToneInChannel(acs_ch);           // Emite en el canal dado por el ACS
                SetToneInChannel(CH1);              // Emite en el canal 1 para indicar a otros robots que está presente
                sleep(1);
            }
        }

        FD[0] = 1;                                  // Indica que detectó a otro robot
        WriteFlashMemory("FIRST_DET", FD, 1);       // Guarda dicha variable en la memoria
    }
}

