#include "Radio.h"

// Variables de RF

RF_Object rfObject;
RF_Handle rfHandle;
RF_Handle rfHandle2;
RF_CmdHandle rfCmdHandle;
RF_CmdHandle rfCmdHandle2;
RF_EventMask terminationReason;
RF_Params rfParams;

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */

#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];

/* Receive dataQueue for RF Core to fill in data */
dataQueue_t dataQueue;
rfc_dataEntryGeneral_t* currentDataEntry;
uint8_t packetLength = 255;                     // Tamaño del paquete de datos recibido desde el TTDM
uint8_t* packetDataPointer;                     // Apuntador de paquete de datos

// Variables auxiliares de la comunicación por Radio
bool CurrentMode = RF_RX;                       // Modo actual del radio
enum Ch_type CurrentChannel = CH_none;          // Ningún canal seleccionado al inicio
bool NEW_COMMAND = false;                       // Bandera que indica que se recibió un nuevo comando
bool NEW_DATA_FROM_ROBOT = false;                       // Bandera que indica que se recibió un nuevo comando
//bool CONTROL = false;

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e){

    if (e & RF_EventRxEntryDone){

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        // packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(IncomingCommand.data_RX, packetDataPointer, (packetLength + 1));

        RFQueue_nextEntry();                // PREPARAMOS LA ENTRADA DEL EL SIGUIENTE PAQUETE

        NEW_DATA_FROM_ROBOT = true;         // Se establece la bandera que indica que se recibió un nuevo comando
    }
}

void InitRF(){

    /*
     * Esta función inicializa el Core de RF
     * SampleRate: 800kBaud
     * Deviation: 25kHz
     */

    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES+1))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    //RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    // ---------------------------------------------------------------
    RF_cmdPropRx.rxConf.bAppendRssi = 0;   /* RSSI */
    RF_cmdPropTx.pktLen = PAYLOAD_TX_LENGTH;
    // RF_cmdPropTx.pPkt = Pq;   // packet_TX;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    // ---------------------------------------------------------------
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;
    // ---------------------------------------------------------------
    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop,(RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    sleep(1);

}

void ChangeRadioMode(bool mode, enum Ch_type ch){

    /*
     * Esta función cambia el modo (Tx/Rx) y canal del radio
     */

    if((mode != CurrentMode) || (ch != CurrentChannel)){                // Primero comprueba que el modo actual sea distinto al solicitado

        CurrentMode = mode; CurrentChannel = ch;                        // Actualiza el modo y canal actual

        switch(ch){
            case CHm1:                                // 2475.75 MHz
                RF_cmdFs.frequency = 0x09AB;
                RF_cmdFs.fractFreq = 0xC000;
                break;
            case CH1:                                 // 2476.75 MHz
                RF_cmdFs.frequency = 0x09AC;
                RF_cmdFs.fractFreq = 0xC000;
                break;
            case CH2:                                 // 2477.25 MHz
                RF_cmdFs.frequency = 0x09AD;
                RF_cmdFs.fractFreq = 0x4000;
                break;
            case CH3:                                 // 2477.75 MHz
                RF_cmdFs.frequency = 0x09AD;
                RF_cmdFs.fractFreq = 0xC000;
                break;
            case CH4:                                 // 2478.25 MHz
                RF_cmdFs.frequency = 0x09AE;
                RF_cmdFs.fractFreq = 0x4000;
                break;
            case CH5:                                 // 2478.75 MHz
                RF_cmdFs.frequency = 0x09AE;
                RF_cmdFs.fractFreq = 0xC000;
                break;
            case CH6:                                 // 2479.25 MHz
                RF_cmdFs.frequency = 0x09AF;
                RF_cmdFs.fractFreq = 0x4000;
                break;
            case CH7:
                RF_cmdFs.frequency = 0x09AF;
                RF_cmdFs.fractFreq = 0xC000;
                break;
        }

        RF_flushCmd(rfHandle, RF_CMDHANDLE_FLUSH_ALL, 0);           // Borra cualquier comando anterior
        rfCmdHandle  = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
        if(mode == RF_RX){
            rfCmdHandle2 = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &callback,RF_EventRxEntryDone);
        }
        if(mode == RF_TX){
            rfCmdHandle2 = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdTxTest, RF_PriorityNormal, NULL, 0);
        }

        usleep(10000);      // Espera 10ms para que el Core cambie de modo
    }

}

void SetPacketToTransmit(uint8_t* _packet){

    /*
     * Esta función establece el paquete que será transmitido a través del radio
     */

    RF_cmdPropTx.pPkt = _packet;
}


// ----------------------------------------------- Comunicaciones por Radio -----------------------------------------------


void TransmitData(){
    /*
     * Esta función prepara el radio en el canal 7 y transmite el paquete de datos
     * apuntado por el atributo RF_cmdPropTx.pPkt.
     */
    RF_cmdFs.frequency = 0x09AF;        // 2479.75 MHz (Canal 7)
    RF_cmdFs.fractFreq = 0xC000;
    RF_flushCmd(rfHandle, RF_CMDHANDLE_FLUSH_ALL, 0);
    rfCmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    rfCmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
}

void ReceiveData(){
    /*
     * Esta función prepara el radio en el canal 7 y lo prepara para recibir
     * alguna cadena de datos. Esta será recibida en la rutina de interrupción callback.
     */
    RF_cmdFs.frequency = 0x09AF;        // 2479.75 MHz  (Canal 7)
    RF_cmdFs.fractFreq = 0xC000;
    RF_flushCmd(rfHandle, RF_CMDHANDLE_FLUSH_ALL, 0);
    rfCmdHandle=RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    rfCmdHandle2= RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &callback,RF_EventRxEntryDone);
}

int8_t senseRSSI(enum Ch_type ch){

    /*
     * Esta función cambia el modo del radio a recepción en el canal especificado.
     * Después obtiene el RSSI en ese canal y lo retorna.
     */

    ChangeRadioMode(RF_RX, ch);
    return RF_getRssi(rfHandle);
}
