#include "Params_Manager.h"

// ------------------- Parámetros por defecto del robot -------------------
Params1 DefaultParams1 = { .Coil_Thld = 0x01, .RF_Thld = 0x02, .RF_Thld_Sigma = 0x03, .Ipanel_Thld = 0x04,
                           .CiclosNav = 0x05, .CiclosBobinaTx = 0x06, .PWM_max = 0x07};
Params2 DefaultParams2 = { .RF1_Entero = 0x09AC, .RF1_Frac = 0xC000, .RF2_Entero = 0x9AD, .RF2_Frac = 0x4000,
                           .RF3_Entero = 0x09AD, .RF3_Frac = 0xC000, .RF4_Entero = 0x9AE, .RF4_Frac = 0x4000,
                           .RF5_Entero = 0x09AE, .RF5_Frac = 0xC000, .RF6_Entero = 0x9AF, .RF6_Frac = 0x4000,
                           .RF7_Entero = 0x09AF, .RF7_Frac = 0xC000};
Params3 DefaultParams3 = { .W1 = 0x01, .W2 = 0x01, .W3 = 0x01, .W4 = 0x01, .W5 = 0x01, .W6 = 0x01,};


// ------ Parámetros actuales del robot. Inicialmente los de defecto ------
Params1 CurrentParams1 = {};
Params2 CurrentParams2 = {};
Params3 CurrentParams3 = {};


void WriteDefaultParametersInMemory(){

    /*
     * Esta función inicializa los parámetros de default de la memoria
     * flash externa. NOTA: No utilizar la función de formateo de archivo
     * debido a que corrompe la memoria.
     */

    MountFlashMemory();                 // Monta la memoria

    DeleteFile("DEF_PARAMS1"); DeleteFile("DEF_PARAMS2"); DeleteFile("DEF_PARAMS3");

    WriteFlashMemory("DEF_PARAMS1", DefaultParams1.byte_array, PARAMS1_LEN);
    WriteFlashMemory("DEF_PARAMS2", DefaultParams2.byte_array, PARAMS2_LEN);
    WriteFlashMemory("DEF_PARAMS3", DefaultParams3.byte_array, PARAMS3_LEN);

    UnmountFlashMemory();               // Desmonta la memoria

}

void SetNewParameters(union Packet_InCMD* cmd_received, uint8_t ParamType){
    /*
     *
     */

    // bool st;        // Bandera de status

    MountFlashMemory();                 // Monta la memoria

    switch(ParamType){
        // Según el tipo de parámetros que se quieran cambiar, se toma la estructura
        // especificada y los bytes requeridos.
        case Pty1:
            memcpy(CurrentParams1.byte_array, cmd_received->data_RX + 4, PARAMS1_LEN);

//            CurrentParams1.Coil_Thld        = cmd_received->B1;
//            CurrentParams1.RF_Thld          = cmd_received->B2;
//            CurrentParams1.RF_Thld_Sigma    = cmd_received->B3;
//            CurrentParams1.Ipanel_Thld      = cmd_received->B4;
//            CurrentParams1.CiclosNav        = cmd_received->B5;
//            CurrentParams1.CiclosBobinaTx   = cmd_received->B6;
//            CurrentParams1.PWM_max          = cmd_received->B7;

            WriteFlashMemory("PARAMS1", CurrentParams1.byte_array, PARAMS1_LEN);
        break;

        case Pty2:
            memcpy(CurrentParams2.byte_array, cmd_received->data_RX + 4, PARAMS2_LEN);

//            CurrentParams2.RF1_Entero   = cmd_received->B1;
//            CurrentParams2.RF1_Frac     = cmd_received->B2;
//            CurrentParams2.RF2_Entero   = cmd_received->B3;
//            CurrentParams2.RF2_Frac     = cmd_received->B4;
//            CurrentParams2.RF3_Entero   = cmd_received->B5;
//            CurrentParams2.RF3_Frac     = cmd_received->B6;
//            CurrentParams2.RF4_Entero   = cmd_received->B7;
//            CurrentParams2.RF4_Frac     = cmd_received->B8;
//            CurrentParams2.RF5_Entero   = cmd_received->B9;
//            CurrentParams2.RF5_Frac     = cmd_received->B10;
//            CurrentParams2.RF6_Entero   = cmd_received->B11;
//            CurrentParams2.RF6_Frac     = cmd_received->B12;
//            CurrentParams2.RF7_Entero   = cmd_received->B13;
//            CurrentParams2.RF7_Frac     = cmd_received->B14;

            WriteFlashMemory("PARAMS2", CurrentParams2.byte_array, PARAMS2_LEN);
        break;

        case Pty3:
            memcpy(CurrentParams3.byte_array, cmd_received->data_RX + 4, PARAMS3_LEN);

//            CurrentParams3.W1 = cmd_received->B1;
//            CurrentParams3.W2 = cmd_received->B2;
//            CurrentParams3.W3 = cmd_received->B3;
//            CurrentParams3.W4 = cmd_received->B4;
//            CurrentParams3.W5 = cmd_received->B5;
//            CurrentParams3.W6 = cmd_received->B6;

            WriteFlashMemory("PARAMS3", CurrentParams3.byte_array, PARAMS3_LEN);
        break;
    }

    UnmountFlashMemory();               // Desmonta la memoria
}

void UpdateParametersFromMemory(bool origin){

    /*
     * Esta función actualiza los parámetros de navegación actuales
     * realizando una lectura desde la memoria flash externa.
     */

    MountFlashMemory();               // Monta la memoria

    if(origin == DEFAULT_PARAMS){
        ReadFlashMemory("DEF_PARAMS1", CurrentParams1.byte_array, PARAMS1_LEN);
        ReadFlashMemory("DEF_PARAMS2", CurrentParams2.byte_array, PARAMS2_LEN);
        ReadFlashMemory("DEF_PARAMS3", CurrentParams3.byte_array, PARAMS3_LEN);
    }
    if(origin == CURRENT_PARAMS){
        ReadFlashMemory("PARAMS1", CurrentParams1.byte_array, PARAMS1_LEN);
        ReadFlashMemory("PARAMS2", CurrentParams2.byte_array, PARAMS2_LEN);
        ReadFlashMemory("PARAMS3", CurrentParams3.byte_array, PARAMS3_LEN);
    }

    UnmountFlashMemory();               // Desmonta la memoria
}


