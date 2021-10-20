
#ifndef ROBOTNAV_STATEMACHINE_H_
#define ROBOTNAV_STATEMACHINE_H_


#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>

#include "NavSearch.h"
#include "NavBoot.h"
#include "NavEscapeStrategy.h"
#include "NavOrientMove.h"
#include "Peripherals/CommonDeclarations.h"


// Definiciones de estados
typedef enum {
  BOOT,
  SEARCH_ROT,
  SEARCH_ST,
  ORIENT_N_MOVE,
  ESCAPE_STRATEGY,
  CAUTION,
  GO
} RSM_State;


struct NavState{

    /*
     * Estructura del estado actual de navegaci�n
     * que permite compartir informaci�n entre
     * los distintos estados de la m�quina.
     */

    uint8_t ACS;                // Tama�o de cluster en el que est� el robot actualmente
    uint8_t LCS;                // �ltimo tama�o de cluster. Anterior al actual
    uint16_t NavCycles;         // Ciclos de navegaci�n totales del robot
    bool SearchResult;          // Resultado de una b�squeda rotacional o est�tica
    bool ClusterChanged;        // Bandera que indica que el cluster cambi� durante un estado o b�squeda
    bool DestroyCluster;        // Bandera que indica que se debe destru�r el cluster
    bool BetterClusterFound;    // Bandera que indica que se encontr� un mejor cluster que el actual
    bool SendTelemetry;         // Bandera que indica que es momento de enviar una trama de telemetr�a al TTDM
    float *DirVector;           // Vector de direcci�n [0] -> Magnitud; [1] -> �ngulo
    float AngStep;              // Paso �ngular m�nimo para la rotaci�n
    Movement_Status MovSt;      // Status del movimiento realizado
    EscapeStrategy  EscStr;     // Estrategia de escape para el estado
};



void StateMachine();
void SetNextState(RSM_State s);
void SetNavCOMMSFlag(bool opt);

#endif /* ROBOTNAV_STATEMACHINE_H_ */
