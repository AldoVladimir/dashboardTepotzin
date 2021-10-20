#include "COMMS_Manager.h"

//#pragma DATA_ALIGN (teleP, 4);

GPTimerCC26XX_Handle hTimer2A;              // Manejador del Timer2A (Maneja la transmisi�n de datos al TTDM)
uint16_t _250ms_counter = 0;                // Contador de cuartos de segundo

TelemetryPacket teleP = {};                 // Estructura de telemetr�a
CompleteStatePacket CompStateP = {};        // Estructura de estado completo

union Packet_InCMD IncomingCommand = {};    // Estructura de paquete de comando entrante



bool DecodeReceivedCommand(union Packet_InCMD * cmd_received){

    /*
     * Esta funci�n valida y decodifica el comando entrante que recibe como
     * par�metro en @cmd_received, el cual es una estructura del tipo Packet_InCMD.
     */

//    uint16_t inCRC = cmd_received->data_RX[PAYLOAD_RX_LENGTH-2];    // CRC16_H
//    inCRC = inCRC << 8;                                             // CRC16_H
//    inCRC |= cmd_received->data_RX[PAYLOAD_RX_LENGTH-1];            // CRC16_L
//
//    uint16_t CRC16 = CRC_16(cmd_received->data_RX, PAYLOAD_RX_LENGTH-2);
//
//    if(inCRC != CRC16){             // Si el CRC no coincide
//        return false;
//    }
//
//    if(!((cmd_received->Mask_Robot_ID)&(1 << (ROBOT_ID - 1)))){     // Si el comando no iba dirigido a este robot
//        return false;
//    }

    if(cmd_received->Header == IN_HEADER){
        switch(cmd_received->CMD_ID){
            case CMD_TELE:
                SendTelemetryFrame();
            break;

            case CMD_TELE_PAUSE_NAV:
                SendTelemetryFrame();
            break;

            case CMD_PAUSE_NAV:
                SendTelemetryFrame();
            break;

            case CMD_RESUME_NAV:
                SendTelemetryFrame();
            break;

            case CMD_CONF_PARMS1:
//                SetNewParameters(cmd_received, Pty1);
                SendTelemetryFrame();
            break;

            case CMD_CONF_PARMS2:
                SetNewParameters(cmd_received, Pty2);
                SendTelemetryFrame();
            break;

            case CMD_CONF_PARMS3:
                SetNewParameters(cmd_received, Pty3);
                SendTelemetryFrame();
            break;

            case CMD_STATE:
                SendRobotStateFrame();
            break;

            case CMD_SCI_DATA:
                StartCoilDataSequence();
            break;

            default:
                return false;
        }
    }
    else{
        return false;
    }

    return true;
}

void SendTelemetryFrame(){

    /*
     * Esta funci�n genera datos de telemetr�a y construye
     * un paquete para ser transmitido a trav�s de RF hacia el TTDM.
     */

    teleP.Header        = OUT_HEADER;
    teleP.CMD_ID        = OUT_CMD_TELE;
    teleP.Robot_ID      = ROBOT_ID;

    teleP.Cluster_Size  = 0;
    teleP.Nav_Cycles    = 69;

    teleP.I_sys         = ReadSensor(I_SYSTEM).ADCSensor_Meas;
    teleP.I_mot         = ReadSensor(I_motor).ADCSensor_Meas;
    teleP.V_uC          = AONBatMonBatteryVoltageGet();

    teleP.Tmp_uC        = AONBatMonTemperatureGetDegC();
    teleP.Tmp_top       = ReadSensor(Tmp_top).ADCSensor_Meas;
    teleP.Tmp_bottom    = ReadSensor(Tmp_bot).ADCSensor_Meas;

    SensorRawMeas imu_meas = ReadSensor(IMU_MAG);
    teleP.MX            = imu_meas.IMU_Meas_X;
    teleP.MY            = imu_meas.IMU_Meas_Y;
    teleP.MZ            = imu_meas.IMU_Meas_Z;



    teleP.packet_TX[PAYLOAD_TX_LENGTH-3] = OUT_END;

    uint16_t CRC16 = CRC_16(teleP.packet_TX, PAYLOAD_TX_LENGTH-2);


    teleP.packet_TX[PAYLOAD_TX_LENGTH-2] = (CRC16 >> 8);
    teleP.packet_TX[PAYLOAD_TX_LENGTH-1] = (CRC16 & 0xFF);

    CONTROL = true;

    SetPacketToTransmit(teleP.packet_TX);       // Establece el paquete a transmitir

    usleep(250000);                                   // Da un retraso para que el TTDM est� listo para recibir

    TransmitData();                             // Transmite el paquete de datos
    usleep(250000);                             // Da un segundo retraso para esperar a que el paquete se env�e
}

void SendRobotStateFrame(){

    /*
     * Esta funci�n lee los par�metros de configuraci�n actuales del robot
     * y genera un paquete de datos para ser transmitido.
     */

    CompStateP.Header           = OUT_HEADER;
    CompStateP.CMD_ID           = OUT_CMD_COMPSTATE;
    CompStateP.Robot_ID         = ROBOT_ID;
    CompStateP.DC               = 0x00;

    memcpy(CompStateP.packet_TX + COMPST_PARMS1_INDEX, CurrentParams1.byte_array, PARAMS1_LEN);
    memcpy(CompStateP.packet_TX + COMPST_PARMS2_INDEX, CurrentParams2.byte_array, PARAMS2_LEN);
    memcpy(CompStateP.packet_TX + COMPST_PARMS3_INDEX, CurrentParams3.byte_array, PARAMS3_LEN);

    CompStateP.packet_TX[PAYLOAD_TX_LENGTH-3] = OUT_END;

    uint16_t CRC16 = CRC_16(CompStateP.packet_TX, PAYLOAD_TX_LENGTH-2);

    CompStateP.packet_TX[PAYLOAD_TX_LENGTH-2] = (CRC16 >> 8);
    CompStateP.packet_TX[PAYLOAD_TX_LENGTH-1] = (CRC16 & 0xFF);

    SetPacketToTransmit(CompStateP.packet_TX);      // Establece el paquete a transmitir
    usleep(500000);                                 // Da un retraso para que el TTDM est� listo para recibir
    TransmitData();                                 // Transmite el paquete de datos
    usleep(250000);                                 // Da un segundo retraso para esperar a que el paquete se env�e
}

void InitCommsTimer(){

    // ------------ Configuraci�n de Timer2A para la transmisi�n del datos al TTDM  -------------
    GPTimerCC26XX_Params paramsGPT2A;                                   // Par�metros para el GPTimer a usar
    GPTimerCC26XX_Params_init(&paramsGPT2A);                            // Inicializa los par�metros por defecto
    paramsGPT2A.width          = GPT_CONFIG_16BIT;                      // Timer de 32bits
    paramsGPT2A.mode           = GPT_MODE_PERIODIC;                     // Modo peri�dico
    paramsGPT2A.direction      = GPTimerCC26XX_DIRECTION_UP;            // Cuenta hacia arriba
    paramsGPT2A.debugStallMode = GPTimerCC26XX_DEBUG_STALL_ON;          // No se detiene en el modo debug
    hTimer2A = GPTimerCC26XX_open(Board_GPTIMER2A, &paramsGPT2A);       // Instanc�a un objeto timer con los par�metros dados. Timer2A

    GPTimerCC26XX_Value loadVal_T2A = 12000000;                         // Establece valor al timer. t_Tx = Tclk*loadVal = (1/48MHz)*(12000000) = 250ms
    GPTimerCC26XX_setLoadValue(hTimer2A, loadVal_T2A);                  // Carga el valor al timer
    GPTimerCC26XX_registerInterrupt(hTimer2A, SendTelemetry_timer2ACallback, GPT_INT_TIMEOUT); // Establece el tipo y rutina de interrupci�n. Por timeout

    GPTimerCC26XX_start(hTimer2A);                                      // Arranca el timer
}

void SendTelemetry_timer2ACallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask){

    /*
     * El timer 2A lleva el tiempo de transmisi�n de telemetr�a hacia el TTDM.
     * Tiene una resoluci�n de 250ms, por lo que se hace uso de la variable
     * @_250ms_counter para llevar una cuenta de varios segundos.
     * Cuando la cuenta de dicha variable llega a @COMMS_MAX_COUNT, se setea
     * la bandera de transmisi�n de telemetr�a y se reinicia el contador.
     */

    if(_250ms_counter == COMMS_MAX_COUNT){              // Si se llego a la cuenta m�xima
        SetNavCOMMSFlag(true);                          // Activa la bandera de env�o de telemetr�a
        _250ms_counter = 0;                             // Reinicia el contador
    }
    else{                                               // Si no se ha llegado a la cuenta m�xima
        _250ms_counter++;                               // Incrementa el contador
    }
}

void NavCOMMSSequenceWtTTDM(){

    /*
     * Esta funci�n realiza una secuencia de comunicaci�n con el TTDM
     * durante la navegaci�n.
     */

    uint8_t i;

    for(i=0; i<MAX_COMMS_ATTEMPTS; i++){                // Realiza MAX_COMMS_ATTEMPTS para comunicarse con el TTDM

        SendTelemetryFrame();                           // Env�a la telemetr�a al TTDM
        ReceiveData();                                  // Despu�s, se pone en modo recepci�n para esperar una respuesta
        sleep(1);                                       // Espera 1 segundo a que llegue una respuesta
        if(NEW_COMMAND){                                // Si recibi� un nuevo comando:
            DecodeReceivedCommand(&IncomingCommand);    // Decodifica el comando y realiza la secuencia de acciones
            break;                                      // Sale del ciclo, no realiza otro intento pues ya complet� la comunicaci�n
        }
    }
}


unsigned short CRC_16(unsigned char *payload, int lenPayload){

    /*
    * Esta funci�n calcula el CRC16 de una trama de datos @payload
    * con longitud establecida por el par�metro @lenPayload.
    */

    int bit,cbit;
    unsigned short crc=0,inv;

    for (bit=lenPayload*8-1; bit>=0; bit--){
        inv = ( payload[lenPayload-1-bit/8] & 0x01 << bit % 8 ) >> bit % 8 ;
        inv ^= crc >> 15;
        crc <<= 1;
        for (cbit=15; cbit>0; cbit--){
            if ( (CRC16_POLYNOMIAL >> cbit) & 0x01 ){
                crc = (((crc >> cbit) & 0x01) ^ inv) << cbit | (~(0x01 << cbit)& crc);
            }
        }
        crc = ( crc & ~0x0001) | inv;
    }
    return (crc);
}
