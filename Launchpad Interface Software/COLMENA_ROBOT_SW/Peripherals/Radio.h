

#ifndef PERIPHERALS_RADIO_H_
#define PERIPHERALS_RADIO_H_

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "Board.h"
#include <ti/drivers/rf/RF.h>
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"
#include "COMMS_Manager.h"

/***** Defines *****/

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8    // Constant header size of a Generic Data Entry
#define MAX_LENGTH             255  // Max length byte the radio will accept
#define NUM_DATA_ENTRIES       2    // NOTE: Only two data entries supported at the moment
#define NUM_APPENDED_BYTES     2    // The Data Entries data field will contain:
                                    // 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                    // Max 30 payload bytes
                                    // 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1)

#define RF_TX   false           // Modo de transmisión RF
#define RF_RX   true            // Modo de recepción RF

// Enumeración para indicar cuales son los canales de TX y RX
enum Ch_type {
  CH_none = 0,      // Ningún canal seleccionado
  CHm1,             // f(-1)
  CH1,              // f(1)
  CH2,
  CH3,
  CH4,
  CH5,
  CH6,
  CH7               // f(7)
};


// ------- Prototipos de funciones -------

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
void InitRF();
void SetPacketToTransmit(uint8_t* _packet);
void ChangeRadioMode(bool mode, enum Ch_type channel);
void TransmitData();
void ReceiveData();
int8_t senseRSSI(enum Ch_type ch);

extern bool NEW_COMMAND;                // Variable externa para ser accesada desde otro segmento de código
extern bool CONTROL;
extern bool NEW_DATA_FROM_ROBOT;
#endif /* PERIPHERALS_RADIO_H_ */
