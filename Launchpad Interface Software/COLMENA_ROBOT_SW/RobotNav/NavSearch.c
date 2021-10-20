#include "NavSearch.h"

uint8_t ACS = 0;                // Tama�o de cluster actual del robot (Actual cluster size)
uint8_t LCS = 0;                // �ltimo tama�o de cluster del robot (Last cluster size)
uint8_t CS_aux = 0;             // Variable auxiliar para guardar el tama�o de cluster
uint8_t OnSearchCS = 0;         // Tama�o del cluster cuando el robot inici� una b�squeda

float DirVector[2] = {};        // Vector de direcci�n para la b�squeda [0] -> Magnitd; [1] -> �ngulo
bool ClusterChanged = false;    // Bandera que indica que el cluster cambi�
bool SearchResult = false;      // Bandera del resultado final de la b�squeda (SUCCESS o FAILED)

uint8_t MaxStepsRot = 0;        // Cantidad m�xima de pasos de b�squeda en rotaci�n
uint8_t MaxStepsStatic = 0;     // Cantidad m�xima de pasos de b�squeda est�tica
uint8_t ss = 0;                 // Search steps (pasos de b�squeda)
float ang_step = 0;             // Paso angular
float current_angle = 0;        // �ngulo actual


void NavSearch(struct NavState * NavS, bool mode){

    /*
     * Funci�n de b�squeda de navegaci�n. Esta funci�n realiza los ciclos de b�squeda
     * del robot seg�n el modo deseado: ROTACIONAL o EST�TICO.
     */

    if(mode == ROTATIONAL_SEARCH){              // Modo rotacional
        RotationalSearch(NavS);
    }

    if(mode == STATIC_SEARCH){                  // Modo est�tico
        StaticSearch(NavS);
    }
}

void RotationalSearch(struct NavState * NavS){

    /*
     * - Modo rotacional (ROTATIONAL_MODE): En este modo el robot realiza la b�squeda rotando sobre su propio eje con
     *                                      pasos de b�squeda cada (360�/MaxStepsRot) grados. Adem�s, en cada paso de
     *                                      rotaci�n mide con la bobina para saber su tama�o de cluster.
     */

    ss = 0;                                         // Reinicia los pasos de b�squeda
    current_angle = 0;                              // �ngulo actual en grados
    MaxStepsRot = 12;                               // Cada 30�
    NavS->AngStep = 2*pi/((float)(MaxStepsRot));    // Calcula el paso angular en radianes
    DirVector[0] = 0; DirVector[1] = 0;             // Reinicia el vector de direcci�n (0: Magnitud, 1: Direcci�n [rad])
    ResetRFMeasurements();                          // Reinicia todas las mediciones de RF del modo rotacional

    while(ss < MaxStepsRot){                        // Mientras no se alcancen los pasos totales

//        if(NavS.SendTelemetry){                     // Si ya se levant� la bandera de env�o de telemetr�a
//            NavCOMMSSequenceWtTTDM();               // Realiza la secuencia de comunicaci�n con el TTDM
//        }

        DoRFMeasureInAngle(current_angle);          // Realiza una medici�n de RSSI en los canales 1 a 5
                                                    // Si el cluster no cambi� de tama�o, el robot sigue solo buscando
        SetToneInChannel(CH1);                      // Pone a transmitir al robot en el canal 1 (pues el robot est� solo)
        MoveRobotSteps(ROTATION_MOVEMENT, 1, DIR_CLOCKWISE, NavS);    // Rota al robot un paso angular en direcci�n horaria
        if(NavS->ClusterChanged){                   // Verifica si el CS cambi� (un robot se conect� o se desconect�)
            UpdateCoilTxSignal();                   // Actualiza las se�ales a trasmitir en la bobina Tx
            break;                                  // Sale del ciclo
        }

        ss += 1;                                    // Incrementa los pasos de b�squeda realizados
        current_angle += NavS->AngStep;             // Incrementa el �ngulo actual en el que se encuentra
    }

    if(NavS->ClusterChanged){                       // Si el cluster cambi� durante la b�squeda
        NavS->SearchResult = STATE_FAILED;          // La b�squeda no fue exitosa pues se encuentra ya en otro cluster
    }
    else{                                           // Sino
        NavS->SearchResult = EvaluateMeasurements(); // Eval�a las mediciones que se hicieron y establece el resultado de la b�squeda (SUCCESS/FAILED)
    }

    if(NavS->SearchResult == STATE_SUCCESS){        // Si el resultado de la b�squeda fue exitoso
        GetDirVector(DirVector);                    // Obtiene el vector de direcci�n (Magnitud, �ngulo)
        NavS->DirVector = DirVector;                // Guarda el vector de direcci�n en la estructura de par�metros
    }
}


void StaticSearch(struct NavState * NavS){

    /*
     * - Modo est�tico (STATIC_MODE): En este modo, el robot realiza la b�squeda sin rotar. Normalmente se entra a este
     *                                modo cuando un robot determina que est� en un cluster de orden 2 o superior. Se
     *                                mantiene escuchando en todos los canales para descubrir si existe un cluster mejor
     *                                que el actual.
     */

    ss = 0;
    MaxStepsStatic = 20;                            // Cantidad de pasos de b�squeda est�tica para considerar un ciclo de navegaci�n
    ResetRFMeasurements();                          // Reinicia todas las mediciones de RF del modo rotacional
    uint8_t cs_found;                               // Tama�o de cluster encontrado en la b�squeda

    while(ss < MaxStepsStatic){                     // Mientras no se alcancen los pasos totales

        DoRFMeasureStatic();                        // Realiza una medici�n de RSSI en los canales 1 a 5
        cs_found = EvaluateStaticMeasurements();    // Eval�a si se encontr� un cluster disponible y lo verifica
        if(cs_found > NavS->ACS){                   // Si se encontr� un cluster mejor que el actual
            NavS->BetterClusterFound = true;        // Activa la bandera BetterCluster
            return;                                 // Retorna inmediatamente
        }

        CoilRxMeasurement(NavS);                    // Realiza una medici�n con las bobinas para determinar tama�o de cluster
        if(NavS->ClusterChanged){                   // Verifica si el CS cambi� (un robot se conect� o se desconect�)
            if(NavS->DestroyCluster){               // Si el cluster en el que se encontraba se destruy�
                return;                             // Retorna inmediatamente. La bandera DestroyCluster queda activa y es interpretada por la RSM
            }
            else{                                   // Si no se destruy� el cluster pero s� cambi� el cluster
                UpdateCoilTxSignal();                   // Actualiza las se�ales a trasmitir en la bobina Tx
                if(NavS->ACS == 1){                 // Si el robot se qued� solo
                    return;                         // Sale del ciclo inmediatamente. Debe abandonar la b�squeda est�tica y pasar a una b�squeda rotacional
                }
            }
        }
                                                    // Si el cluster no cambi� de tama�o, el robot sigue buscando
        SetToneInChannel(CS_to_CHtype(NavS->ACS));  // Pone a transmitir al robot en el canal correspondiente al tama�o de cluster en el que se encuentra

        sleep(2);                                   // Duerme al robot durante 2 segundos. Durante ese tiempo se mantiene transmitiendo en el canal ACS

        ss += 1;                                    // Incrementa los pasos de b�squeda realizados
    }

    NavS->NavCycles += 1;                           // Incrementa la cantidad de ciclos de navegaci�n realizados

}


