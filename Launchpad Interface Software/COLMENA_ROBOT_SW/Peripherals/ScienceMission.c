
#include "ScienceMission.h"

ScienceDataPacket SciDataP = {};


void StartCoilDataSequence(){

    Sci_Fs fs[7] = {Sci_f_200kHz, Sci_f_100kHz, Sci_f_10kHz, Sci_f_1kHz, Sci_f_100Hz, Sci_f_10Hz, Sci_f_1Hz};
    uint8_t i = 0;
    uint16_t * data_p;
    for(i=0; i<7; i++){
        data_p = CoilRxMeasurement_ScienceMission(fs[i]);       // Realiza las mediciones
        SendCoilDataToTTDM(i+1, data_p);                        // Se envían las mediciones al TTDM
        sleep(1);                                               // Espera un segundo antes de la siguiente medición
    }
}

void SendCoilDataToTTDM(uint8_t freq_code, uint16_t * data){

    SciDataP.Header        = OUT_HEADER;
    SciDataP.CMD_ID        = OUT_CMD_TELE;
    SciDataP.Robot_ID      = ROBOT_ID;
    SciDataP.Freq          = freq_code;

    memcpy(SciDataP.packet_TX + SCIDATA_INDEX, (uint8_t*)data, SCIDATA_BYTE_LEN);

    SciDataP.packet_TX[PAYLOAD_TX_LENGTH-3] = OUT_END;

    uint16_t CRC16 = CRC_16(SciDataP.packet_TX, PAYLOAD_TX_LENGTH-2);

    SciDataP.packet_TX[PAYLOAD_TX_LENGTH-2] = (CRC16 >> 8);
    SciDataP.packet_TX[PAYLOAD_TX_LENGTH-1] = (CRC16 & 0xFF);

    SetPacketToTransmit(SciDataP.packet_TX);    // Establece el paquete a transmitir
    usleep(500000);                             // Da un retraso para que el TTDM esté listo para recibir
    TransmitData();                             // Transmite el paquete de datos
    usleep(250000);                             // Da un segundo retraso para esperar a que el paquete se envíe
}




