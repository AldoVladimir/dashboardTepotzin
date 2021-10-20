#include "StateMachine.h"

RSM_State currentState;
struct NavState NavS = { .ACS = 1, .LCS = 1, .NavCycles = 0, .SearchResult = STATE_FAILED,
                         .ClusterChanged = false, .DirVector = 0, .SendTelemetry = false};


void StateMachine(){

    SetNextState(BOOT);                                 // Siempre el primer estado es BOOT.

    while(1){                                           // LOOP INFINITO DE LA M�QUINA DE ESTADOS

//        if(NavS.SendTelemetry){                         // Si ya se levant� la bandera de env�o de telemetr�a
//            NavCOMMSSequenceWtTTDM();                   // Realiza la secuencia de comunicaci�n con el TTDM
//            NavS.SendTelemetry = false;                 // Baja la bandera de comunicaci�n
//        }

        switch(currentState){                           // Seg�n el estado actual

        // ---------------------------------- Estado BOOT ----------------------------------

            case BOOT:

                NavBoot(&NavS);                         // Inicia todos los sistemas del robot y espera por la primera detecci�n
                if(NavS.ACS == 1){                      // Si el tama�o de cluster actual es 1:
                    SetNextState(SEARCH_ROT);           // Comienza realizando una b�squeda rotacional
                }
                else{
                    if(NavS.ACS > 1){                   // Si el robot est� en un cluster de 2 o superior
                        SetNextState(SEARCH_ST);        // Comienza realizando una b�squeda est�tica
                    }
                }

            break;

        // ------------------------------- Estado SEARCH ROT -------------------------------

            case SEARCH_ROT:

                NavS.ClusterChanged = false;
                NavS.DirVector = 0;
                NavS.SearchResult = STATE_FAILED;

                NavSearch(&NavS, ROTATIONAL_SEARCH);        // Realiza una b�squeda de tipo rotacional

                if(NavS.ClusterChanged){                    // Si el cluster cambi� mientras se realizaba la b�squeda
                    if(NavS.ACS == 1){                      // Si el robot se qued� s�lo mientras buscaba
                        SetNextState(SEARCH_ROT);           // Vuelve a hacer una b�squeda rotacional
                    }
                    else{
                        if(NavS.ACS > 1){                   // Si el robot est� en un cluster de 2 o superior
                            SetNextState(SEARCH_ST);        // Realiza una b�squeda est�tica
                        }
                    }
                }
                else{
                    if(NavS.SearchResult == STATE_SUCCESS){ // Si el resultado de la b�squeda fue exitoso
                        SetNextState(ORIENT_N_MOVE);        // Pasa el estado de orientaci�n y traslaci�n
                    }
                    else{                                   // Si el estado no fue exitoso
                        SetNextState(SEARCH_ROT);           // Vuelve a hacer una b�squeda rotacional
                    }
                }

//                sleep(2);
//                SetNextState(SEARCH_ROT);           // Vuelve a hacer una b�squeda rotacional

            break;

        // ----------------------------- Estado SEARCH STATIC ------------------------------

            case SEARCH_ST:                                 // Estado B�squeda est�tica

                NavS.BetterClusterFound = false;
                NavS.DestroyCluster = false;

                NavSearch(&NavS, STATIC_SEARCH);            // Realiza una b�squeda de tipo est�tica

                if(NavS.BetterClusterFound){                // Si en la b�squeda se encontr� un mejor cluster que el actual
                    // TODO: Implementar las distintas estrategias de escape, seg�n la situaci�n
                    NavS.EscStr = EXIT_THE_CLUSTER;         // La estrategia a implementar ser� de salir del cluster actual
                    SetNextState(ESCAPE_STRATEGY);          // Siguiente estado: Seguir la estrategia de escape
                    break;
                }else{
                    if(NavS.DestroyCluster){                // Si se detect� una se�al de destrucci�n de cluster
                        SetNextState(ESCAPE_STRATEGY);      // Siguiente estado: Seguir la estrategia de escape
                        break;
                    }
                }

                if(NavS.ACS == 1){                          // Si durante la b�squeda el robot se qued� solo:
                    // TODO: Implementar las distintas estrategias de escape, seg�n la situaci�n
                    SetNextState(SEARCH_ROT);               // Pasa a b�squeda rotacional
                }else{                                      // Si no ocurri� nada de lo anterior, la b�squeda no dio ning�n resultado
                    SetNextState(SEARCH_ST);                // por lo que pasa a b�squeda est�tica nuevamente
                }

            break;

        // ---------------------------- Estado ORIENT AND MOVE -----------------------------

            case ORIENT_N_MOVE:                             // Estado ORIENT AND MOVE

                NavS.ClusterChanged = false;

                NavOrientMove(&NavS);                       // Se manda a llamar al estado

                if(NavS.MovSt == SUCCESS_MOVE_DONE){        // Si el movimiento se complet� correctamente
                    if(NavS.ClusterChanged){                // Verifica si el cluster cambi�
                        SetNextState(SEARCH_ST);            // En caso afirmativo, pasa a un estado de b�squeda est�tica
                    }
                    else{                                   // En caso de que el cluster no haya cambiado
                        SetNextState(SEARCH_ROT);           // Vuelve a hacer una b�squeda rotacional
                    }
                }
                else{                                       // Si el movimiento no se complet�. Se tuvo alg�n error
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


