#ifndef PERIPHERALS_COMMS_MANAGER_H_
#define PERIPHERALS_COMMS_MANAGER_H_

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "Peripherals/Radio.h"
#include "Robot.h"
#include "Peripherals/SensorsManager.h"
#include "RobotNav/Params_Manager.h"
#include "ScienceMission.h"
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include "RobotNav/StateMachine.h"

#define CRC16_POLYNOMIAL 0x015935

#define PAYLOAD_TX_LENGTH   254         // Tamaño de trama de datos de Tx
#define PAYLOAD_RX_LENGTH   254          // Tamaño de trama de datos de Rx

#define IN_HEADER           0x76        // Header válido de comandos de entrada
#define IN_END              0x0A        // Byte de fin de la trama de entrada
#define OUT_HEADER          0x52        // Header válido de comandos de salida
#define OUT_END             0x0B        // Byte de fin de la trama de salida

/*
 * Índices de posición de cada uno de los arreglos de parámetros
 * de la trama de estado completo.
 */
#define COMPST_PARMS1_INDEX     4
#define COMPST_PARMS2_INDEX     18
#define COMPST_PARMS3_INDEX     46

// Enumeración que define cada uno de los ID de comando de entrada
typedef enum{
  CMD_TELE              = 0x80,
  CMD_TELE_PAUSE_NAV    = 0x81,
  CMD_PAUSE_NAV         = 0x82,
  CMD_RESUME_NAV        = 0x83,
  CMD_STATE             = 0x84,
  CMD_SCI_DATA          = 0x85,
  CMD_CONF_PARMS1       = 0x86,
  CMD_CONF_PARMS2       = 0x87,
  CMD_CONF_PARMS3       = 0x88
} IN_CMD_ID_LIST;

typedef enum{
  OUT_CMD_TELE          = 0x60,
  OUT_CMD_COMPSTATE     = 0x61,
  OUT_CMD_SCIDATA       = 0x62
} OUT_CMD_ID_LIST;

// ------------------------------------- Unión del paquete a enviar -------------------------------------

/*
 * NOTA IMPORTANTE: Cuando se genera la estructura se deben emplear tipos de datos
 *                  idénticos en bloques de 32 bits.
 */

typedef union TelemetryPacket_S{
    struct{
        uint8_t Header;         // BYTE 0
        uint8_t CMD_ID;         // BYTE 1
        uint8_t Robot_ID;       // BYTE 2
        uint8_t DC;             // BYTE 3

        uint16_t Cluster_Size;  // BYTE 4,5
        uint16_t Nav_Cycles;    // BYTE 6,7

        uint16_t I_sys;         // BYTE 8,9
        uint16_t I_mot;         // BYTE 10,11

        int16_t V_uC;           // BYTE 12,13
        int16_t Tmp_uC;         // BYTE 14,15

        uint16_t Tmp_top;       // BYTE 16,17
        uint16_t Tmp_bottom;    // BYTE 18,19

        int16_t MX;             // BYTE 20,21
        int16_t MY;             // BYTE 22,23
        int16_t MZ;             // BYTE 24,25
    };
    uint8_t packet_TX[PAYLOAD_TX_LENGTH];
} TelemetryPacket;

typedef union CompleteStatePacket_S{
    struct{
        uint8_t Header;
        uint8_t CMD_ID;
        uint8_t Robot_ID;
        uint8_t DC;

        uint16_t Coil_Thld;
        uint16_t RF_Thld;

        uint16_t RF_Thld_Sigma;
        uint16_t Ipanel_Thld;

        uint16_t CiclosNav;
        uint16_t CiclosBobinaTx;

        uint16_t PWM_max;
        uint16_t RF1_Entero;

        uint16_t RF1_Frac;
        uint16_t RF2_Entero;

        uint16_t RF2_Frac;
        uint16_t RF3_Entero;

        uint16_t RF3_Frac;
        uint16_t RF4_Entero;

        uint16_t RF4_Frac;
        uint16_t RF5_Entero;

        uint16_t RF5_Frac;
        uint16_t RF6_Entero;

        uint16_t RF6_Frac;
        uint16_t RF7_Entero;

        uint16_t RF7_Frac;
        uint16_t W1;

        uint16_t W2;
        uint16_t W3;

        uint16_t W4;
        uint16_t W5;

        uint16_t W6;
    };
    uint8_t packet_TX[PAYLOAD_TX_LENGTH];
} CompleteStatePacket;

// ---------------------------------- Unión del paquete a recibir -----------------------------------

union Packet_InCMD{
    struct{
        uint8_t Header;
        uint8_t CMD_ID;
        uint8_t Mask_Robot_ID;
        uint8_t DC1;

        uint16_t B1;
        uint16_t B2;

        uint16_t B3;
        uint16_t B4;

        uint16_t B5;
        uint16_t B6;

        uint16_t B7;
        uint16_t B8;

        uint16_t B9;
        uint16_t B10;

        uint16_t B11;
        uint16_t B12;

        uint16_t B13;
        uint16_t B14;

        uint16_t B15;
        uint16_t B16;

        uint16_t B17;
        uint16_t B18;

        uint8_t B19;
        uint8_t B20;
        uint8_t B21;
        uint8_t End;

        uint16_t CRC16;
    };
    uint8_t data_RX[PAYLOAD_RX_LENGTH];
} Packet_InCMD;


#define COMMS_MAX_TIME_IN_SEC       30                                  // Cadencia de transmisión de telemetría al TTDM en segundos
#define COMMS_MAX_COUNT             COMMS_MAX_TIME_IN_SEC*4 - 1         // Número de conteos máximos (4 conteos por segundo)

#define MAX_COMMS_ATTEMPTS   3                                          // Cantidad de intentos de comunicación el TTDM

// ------------------- Prototipos de funciones -------------------

void SendTelemetryFrame();
void SendRobotStateFrame();
bool DecodeReceivedCommand(union Packet_InCMD * cmd_received);
unsigned short CRC_16(unsigned char *payload, int lenPayload);

void InitCommsTimer();
void SendTelemetry_timer2ACallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);

void NavCOMMSSequenceWtTTDM();

extern union Packet_InCMD IncomingCommand;

#endif /* PERIPHERALS_COMMS_MANAGER_H_ */
