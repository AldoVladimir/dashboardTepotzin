#include "NavBoot.h"


void NavBoot(struct NavState * NavS){

    /*
     * Estado Boot de la m�quina de estados. Es el primer estado en entrar. Levanta todos los perif�ricos del
     * microcontrolador y espera hasta que se haga una primera detecci�n de la se�al de otro robot.
     */

    sleep(TIME_TO_ON_SEC);                      // Cada vez que se reinicie el robot, se tomar� "TIME_TO_ON_SEC" segundos para esperar a que el capacitor se cargue

    // ----- Inicia secuencia de booteo, levanta todos los m�dulos -----

    GPIO_init();                                // Inicializa los GPIOS para su uso con la memoria externa
    SPI_init();                                 // Inicializa el manejador del perif�rico SPI para comunicaci�n con la memoria y el DAC

    Init_CoilRx();                              // Inicializa la bobina receptora
    Init_CoilTx();                              // Inicializa la bobina emisora

#if FST_TIME_MEM_WRITE
    WriteDefaultParametersInMemory();           // Si es la primera vez que se programa al robot, se escriben los par�metros de defecto en la memoria
#endif
    UpdateParametersFromMemory(DEFAULT_PARAMS); // Cuando bootea o rebootea el robot, usa siempre los par�metros de navegaci�n de default

    CoilRxMeasurement(NavS);                    // Obtiene el tama�o de cluster inicial
    NavS->LCS = NavS->ACS;                      // El �ltimo tama�o de cluster es el mismo

    Init_Motors();                              // Inicializa los motores del robot

    InitSensors();                              // Inicializa los sensores del robot
    // IsysCalibration();                          // Realiza la calibraci�n de la corriente del sistema.

    InitRF();                                   // Inicializa el m�dulo de RF
    // RFCalibration();                            // Realiza la calibraci�n del m�dulo RF

    InitCommsTimer();                           // Inicializa el timer de transmisi�n de telemetr�a al TTDM

    // WaitForFirstDetection(CS_to_CHtype(NavS->ACS)); // Espera hasta que se detecte a alg�n otro robot activado. Si ya se hab�a activado antes, no busca nada
}

void WaitForFirstDetection(enum Ch_type acs_ch){

    /*
     * Esta funci�n comprueba si ya se ha realizado una primera detecci�n de la se�al de otro robot.
     * Si el robot se reinici� es posible que ya haya detectado a alg�n otro, por lo cual no espera
     * y continua con el resto de la ejecuci�n.
     * FD = 1 (significa que ya se hab�a realizado la detecci�n con anterioridad)
     * FD = 0 (significa que no se ha realizado la detecci�n nunca)
     */

    uint8_t FD[1] = {0};                            // Variable para leer par�metro FIRST DETECTION
    ReadFlashMemory("FIRST_DET", FD, 1);            // Lee el par�metro

    if(FD[0]){                                      // Si ya se hab�a realizado la detecci�n anteriormente
        return;                                     // Sale de la funci�n y no espera m�s
    }
    else{                                           // Si no:
        //while(!IsOtherRobotActivated());            // Espera hasta que detecte cualquier otro robot activado
        while(1){

            if(IsOtherRobotActivated()){            // Si hay otro robot activado
                break;                              // Sale del ciclo
            }else{                                  // Si no:
                //SetToneInChannel(acs_ch);           // Emite en el canal dado por el ACS
                SetToneInChannel(CH1);              // Emite en el canal 1 para indicar a otros robots que est� presente
                sleep(1);
            }
        }

        FD[0] = 1;                                  // Indica que detect� a otro robot
        WriteFlashMemory("FIRST_DET", FD, 1);       // Guarda dicha variable en la memoria
    }
}

