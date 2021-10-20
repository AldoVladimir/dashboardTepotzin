
#ifndef ROBOTNAV_PARAMS_MANAGER_H_
#define ROBOTNAV_PARAMS_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>
#include "Peripherals/FlashMem.h"
#include "Peripherals/COMMS_Manager.h"

#define PARAMS1_LEN     14          // Cantidad de bytes para Parámetros 1 (Parámetros generales)
#define PARAMS2_LEN     28          // Cantidad de bytes para Parámetros 1 (Frecuencias de emisión y recepción)
#define PARAMS3_LEN     12          // Cantidad de bytes para Parámetros 1 (Pesos de cada canal)

#define  DEFAULT_PARAMS false
#define  CURRENT_PARAMS true

#define Pty1      0     // Tipos de parámetros recibidos.
#define Pty2      1
#define Pty3      2

union Packet_InCMD;

typedef union Params1_U{
    struct{
        uint16_t Coil_Thld;
        uint16_t RF_Thld;

        uint16_t RF_Thld_Sigma;
        uint16_t Ipanel_Thld;

        uint16_t CiclosNav;
        uint16_t CiclosBobinaTx;

        uint16_t PWM_max;
    };
    uint8_t byte_array[PARAMS1_LEN];
} Params1;


typedef union Params2_U{
    struct{
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
    };
    uint8_t byte_array[PARAMS2_LEN];
} Params2;


typedef union Params3_U{
    struct{
        uint16_t W1;
        uint16_t W2;

        uint16_t W3;
        uint16_t W4;

        uint16_t W5;
        uint16_t W6;
    };
    uint8_t byte_array[PARAMS3_LEN];
} Params3;


// ---------- Prototipos de funciones ----------
void WriteDefaultParametersInMemory();
void SetNewParameters(union Packet_InCMD* cmd_received, uint8_t ParamType);
void UpdateParametersFromMemory(bool origin);

extern Params1 CurrentParams1;
extern Params2 CurrentParams2;
extern Params3 CurrentParams3;

#endif /* ROBOTNAV_PARAMS_MANAGER_H_ */
