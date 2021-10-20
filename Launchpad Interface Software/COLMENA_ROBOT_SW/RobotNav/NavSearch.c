#include "NavSearch.h"

uint8_t ACS = 0;                // Tamaño de cluster actual del robot (Actual cluster size)
uint8_t LCS = 0;                // Último tamaño de cluster del robot (Last cluster size)
uint8_t CS_aux = 0;             // Variable auxiliar para guardar el tamaño de cluster
uint8_t OnSearchCS = 0;         // Tamaño del cluster cuando el robot inició una búsqueda

float DirVector[2] = {};        // Vector de dirección para la búsqueda [0] -> Magnitd; [1] -> Ángulo
bool ClusterChanged = false;    // Bandera que indica que el cluster cambió
bool SearchResult = false;      // Bandera del resultado final de la búsqueda (SUCCESS o FAILED)

uint8_t MaxStepsRot = 0;        // Cantidad máxima de pasos de búsqueda en rotación
uint8_t MaxStepsStatic = 0;     // Cantidad máxima de pasos de búsqueda estática
uint8_t ss = 0;                 // Search steps (pasos de búsqueda)
float ang_step = 0;             // Paso angular
float current_angle = 0;        // Ángulo actual


void NavSearch(struct NavState * NavS, bool mode){

    /*
     * Función de búsqueda de navegación. Esta función realiza los ciclos de búsqueda
     * del robot según el modo deseado: ROTACIONAL o ESTÁTICO.
     */

    if(mode == ROTATIONAL_SEARCH){              // Modo rotacional
        RotationalSearch(NavS);
    }

    if(mode == STATIC_SEARCH){                  // Modo estático
        StaticSearch(NavS);
    }
}

void RotationalSearch(struct NavState * NavS){

    /*
     * - Modo rotacional (ROTATIONAL_MODE): En este modo el robot realiza la búsqueda rotando sobre su propio eje con
     *                                      pasos de búsqueda cada (360°/MaxStepsRot) grados. Además, en cada paso de
     *                                      rotación mide con la bobina para saber su tamaño de cluster.
     */

    ss = 0;                                         // Reinicia los pasos de búsqueda
    current_angle = 0;                              // Ángulo actual en grados
    MaxStepsRot = 12;                               // Cada 30°
    NavS->AngStep = 2*pi/((float)(MaxStepsRot));    // Calcula el paso angular en radianes
    DirVector[0] = 0; DirVector[1] = 0;             // Reinicia el vector de dirección (0: Magnitud, 1: Dirección [rad])
    ResetRFMeasurements();                          // Reinicia todas las mediciones de RF del modo rotacional

    while(ss < MaxStepsRot){                        // Mientras no se alcancen los pasos totales

//        if(NavS.SendTelemetry){                     // Si ya se levantó la bandera de envío de telemetría
//            NavCOMMSSequenceWtTTDM();               // Realiza la secuencia de comunicación con el TTDM
//        }

        DoRFMeasureInAngle(current_angle);          // Realiza una medición de RSSI en los canales 1 a 5
                                                    // Si el cluster no cambió de tamaño, el robot sigue solo buscando
        SetToneInChannel(CH1);                      // Pone a transmitir al robot en el canal 1 (pues el robot está solo)
        MoveRobotSteps(ROTATION_MOVEMENT, 1, DIR_CLOCKWISE, NavS);    // Rota al robot un paso angular en dirección horaria
        if(NavS->ClusterChanged){                   // Verifica si el CS cambió (un robot se conectó o se desconectó)
            UpdateCoilTxSignal();                   // Actualiza las señales a trasmitir en la bobina Tx
            break;                                  // Sale del ciclo
        }

        ss += 1;                                    // Incrementa los pasos de búsqueda realizados
        current_angle += NavS->AngStep;             // Incrementa el ángulo actual en el que se encuentra
    }

    if(NavS->ClusterChanged){                       // Si el cluster cambió durante la búsqueda
        NavS->SearchResult = STATE_FAILED;          // La búsqueda no fue exitosa pues se encuentra ya en otro cluster
    }
    else{                                           // Sino
        NavS->SearchResult = EvaluateMeasurements(); // Evalúa las mediciones que se hicieron y establece el resultado de la búsqueda (SUCCESS/FAILED)
    }

    if(NavS->SearchResult == STATE_SUCCESS){        // Si el resultado de la búsqueda fue exitoso
        GetDirVector(DirVector);                    // Obtiene el vector de dirección (Magnitud, Ángulo)
        NavS->DirVector = DirVector;                // Guarda el vector de dirección en la estructura de parámetros
    }
}


void StaticSearch(struct NavState * NavS){

    /*
     * - Modo estático (STATIC_MODE): En este modo, el robot realiza la búsqueda sin rotar. Normalmente se entra a este
     *                                modo cuando un robot determina que está en un cluster de orden 2 o superior. Se
     *                                mantiene escuchando en todos los canales para descubrir si existe un cluster mejor
     *                                que el actual.
     */

    ss = 0;
    MaxStepsStatic = 20;                            // Cantidad de pasos de búsqueda estática para considerar un ciclo de navegación
    ResetRFMeasurements();                          // Reinicia todas las mediciones de RF del modo rotacional
    uint8_t cs_found;                               // Tamaño de cluster encontrado en la búsqueda

    while(ss < MaxStepsStatic){                     // Mientras no se alcancen los pasos totales

        DoRFMeasureStatic();                        // Realiza una medición de RSSI en los canales 1 a 5
        cs_found = EvaluateStaticMeasurements();    // Evalúa si se encontró un cluster disponible y lo verifica
        if(cs_found > NavS->ACS){                   // Si se encontró un cluster mejor que el actual
            NavS->BetterClusterFound = true;        // Activa la bandera BetterCluster
            return;                                 // Retorna inmediatamente
        }

        CoilRxMeasurement(NavS);                    // Realiza una medición con las bobinas para determinar tamaño de cluster
        if(NavS->ClusterChanged){                   // Verifica si el CS cambió (un robot se conectó o se desconectó)
            if(NavS->DestroyCluster){               // Si el cluster en el que se encontraba se destruyó
                return;                             // Retorna inmediatamente. La bandera DestroyCluster queda activa y es interpretada por la RSM
            }
            else{                                   // Si no se destruyó el cluster pero sí cambió el cluster
                UpdateCoilTxSignal();                   // Actualiza las señales a trasmitir en la bobina Tx
                if(NavS->ACS == 1){                 // Si el robot se quedó solo
                    return;                         // Sale del ciclo inmediatamente. Debe abandonar la búsqueda estática y pasar a una búsqueda rotacional
                }
            }
        }
                                                    // Si el cluster no cambió de tamaño, el robot sigue buscando
        SetToneInChannel(CS_to_CHtype(NavS->ACS));  // Pone a transmitir al robot en el canal correspondiente al tamaño de cluster en el que se encuentra

        sleep(2);                                   // Duerme al robot durante 2 segundos. Durante ese tiempo se mantiene transmitiendo en el canal ACS

        ss += 1;                                    // Incrementa los pasos de búsqueda realizados
    }

    NavS->NavCycles += 1;                           // Incrementa la cantidad de ciclos de navegación realizados

}


