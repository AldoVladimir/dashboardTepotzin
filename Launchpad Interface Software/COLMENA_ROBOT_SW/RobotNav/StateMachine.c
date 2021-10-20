#include "StateMachine.h"

RSM_State currentState;
struct NavState NavS = { .ACS = 1, .LCS = 1, .NavCycles = 0, .SearchResult = STATE_FAILED,
                         .ClusterChanged = false, .DirVector = 0, .SendTelemetry = false};


void StateMachine(){

    SetNextState(BOOT);                                 // Siempre el primer estado es BOOT.

    while(1){                                           // LOOP INFINITO DE LA MÁQUINA DE ESTADOS

//        if(NavS.SendTelemetry){                         // Si ya se levantó la bandera de envío de telemetría
//            NavCOMMSSequenceWtTTDM();                   // Realiza la secuencia de comunicación con el TTDM
//            NavS.SendTelemetry = false;                 // Baja la bandera de comunicación
//        }

        switch(currentState){                           // Según el estado actual

        // ---------------------------------- Estado BOOT ----------------------------------

            case BOOT:

                NavBoot(&NavS);                         // Inicia todos los sistemas del robot y espera por la primera detección
                if(NavS.ACS == 1){                      // Si el tamaño de cluster actual es 1:
                    SetNextState(SEARCH_ROT);           // Comienza realizando una búsqueda rotacional
                }
                else{
                    if(NavS.ACS > 1){                   // Si el robot está en un cluster de 2 o superior
                        SetNextState(SEARCH_ST);        // Comienza realizando una búsqueda estática
                    }
                }

            break;

        // ------------------------------- Estado SEARCH ROT -------------------------------

            case SEARCH_ROT:

                NavS.ClusterChanged = false;
                NavS.DirVector = 0;
                NavS.SearchResult = STATE_FAILED;

                NavSearch(&NavS, ROTATIONAL_SEARCH);        // Realiza una búsqueda de tipo rotacional

                if(NavS.ClusterChanged){                    // Si el cluster cambió mientras se realizaba la búsqueda
                    if(NavS.ACS == 1){                      // Si el robot se quedó sólo mientras buscaba
                        SetNextState(SEARCH_ROT);           // Vuelve a hacer una búsqueda rotacional
                    }
                    else{
                        if(NavS.ACS > 1){                   // Si el robot está en un cluster de 2 o superior
                            SetNextState(SEARCH_ST);        // Realiza una búsqueda estática
                        }
                    }
                }
                else{
                    if(NavS.SearchResult == STATE_SUCCESS){ // Si el resultado de la búsqueda fue exitoso
                        SetNextState(ORIENT_N_MOVE);        // Pasa el estado de orientación y traslación
                    }
                    else{                                   // Si el estado no fue exitoso
                        SetNextState(SEARCH_ROT);           // Vuelve a hacer una búsqueda rotacional
                    }
                }

//                sleep(2);
//                SetNextState(SEARCH_ROT);           // Vuelve a hacer una búsqueda rotacional

            break;

        // ----------------------------- Estado SEARCH STATIC ------------------------------

            case SEARCH_ST:                                 // Estado Búsqueda estática

                NavS.BetterClusterFound = false;
                NavS.DestroyCluster = false;

                NavSearch(&NavS, STATIC_SEARCH);            // Realiza una búsqueda de tipo estática

                if(NavS.BetterClusterFound){                // Si en la búsqueda se encontró un mejor cluster que el actual
                    // TODO: Implementar las distintas estrategias de escape, según la situación
                    NavS.EscStr = EXIT_THE_CLUSTER;         // La estrategia a implementar será de salir del cluster actual
                    SetNextState(ESCAPE_STRATEGY);          // Siguiente estado: Seguir la estrategia de escape
                    break;
                }else{
                    if(NavS.DestroyCluster){                // Si se detectó una señal de destrucción de cluster
                        SetNextState(ESCAPE_STRATEGY);      // Siguiente estado: Seguir la estrategia de escape
                        break;
                    }
                }

                if(NavS.ACS == 1){                          // Si durante la búsqueda el robot se quedó solo:
                    // TODO: Implementar las distintas estrategias de escape, según la situación
                    SetNextState(SEARCH_ROT);               // Pasa a búsqueda rotacional
                }else{                                      // Si no ocurrió nada de lo anterior, la búsqueda no dio ningún resultado
                    SetNextState(SEARCH_ST);                // por lo que pasa a búsqueda estática nuevamente
                }

            break;

        // ---------------------------- Estado ORIENT AND MOVE -----------------------------

            case ORIENT_N_MOVE:                             // Estado ORIENT AND MOVE

                NavS.ClusterChanged = false;

                NavOrientMove(&NavS);                       // Se manda a llamar al estado

                if(NavS.MovSt == SUCCESS_MOVE_DONE){        // Si el movimiento se completó correctamente
                    if(NavS.ClusterChanged){                // Verifica si el cluster cambió
                        SetNextState(SEARCH_ST);            // En caso afirmativo, pasa a un estado de búsqueda estática
                    }
                    else{                                   // En caso de que el cluster no haya cambiado
                        SetNextState(SEARCH_ROT);           // Vuelve a hacer una búsqueda rotacional
                    }
                }
                else{                                       // Si el movimiento no se completó. Se tuvo algún error
                    SetNextState(ESCAPE_STRATEGY);          // Pasa a realizar alguna estrategia de escape
                }

            break;

        // ---------------------------- Estado ESCAPTE STRATEGY ----------------------------

            case ESCAPE_STRATEGY:                           // Estado ESCAPE SEQUENCE

                NavEscapeStrategy(&NavS);

            break;

        }

    }

}

void SetNextState(RSM_State s){
    currentState = s;
}

void SetNavCOMMSFlag(bool opt){

    NavS.SendTelemetry = opt;
}


