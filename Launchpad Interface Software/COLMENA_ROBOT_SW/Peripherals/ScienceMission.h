
#ifndef PERIPHERALS_SCIENCEMISSION_H_
#define PERIPHERALS_SCIENCEMISSION_H_

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "Coil.h"
#include "COMMS_Manager.h"
#include "Radio.h"

#define SCIDATA_INDEX   4           // Índice del inicio de los datos de ciencia en la trama de datos
#define SCIDATA_BYTE_LEN 246        // Número de bytes de datos de ciencia
#define PAYLOAD_TX_LENGTH   254     // Tamaño de trama de datos de Tx

typedef union ScienceDataPacket_S{
    struct{
        uint8_t Header;
        uint8_t CMD_ID;
        uint8_t Robot_ID;
        uint8_t Freq;
    };
    uint8_t packet_TX[PAYLOAD_TX_LENGTH];
} ScienceDataPacket;

void StartCoilDataSequence();
void SendCoilDataToTTDM(uint8_t freq_code, uint16_t * data);


#endif /* PERIPHERALS_SCIENCEMISSION_H_ */
