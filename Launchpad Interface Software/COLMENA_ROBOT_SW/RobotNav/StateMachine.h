
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
     * Estructura del estado actual de navegación
     * que permite compartir información entre
     * los distintos estados de la máquina.
     */

    uint8_t ACS;                // Tamaño de cluster en el que está el robot actualmente
    uint8_t LCS;                // Último tamaño de cluster. Anterior al actual
    uint16_t NavCycles;         // Ciclos de navegación totales del robot
    bool SearchResult;          // Resultado de una búsqueda rotacional o estática
    bool ClusterChanged;        // Bandera que indica que el cluster cambió durante un estado o búsqueda
    bool DestroyCluster;        // Bandera que indica que se debe destruír el cluster
    bool BetterClusterFound;    // Bandera que indica que se encontró un mejor cluster que el actual
    bool SendTelemetry;         // Bandera que indica que es momento de enviar una trama de telemetría al TTDM
    float *DirVector;           // Vector de dirección [0] -> Magnitud; [1] -> Ángulo
    float AngStep;              // Paso ángular mínimo para la rotación
    Movement_Status MovSt;      // Status del movimiento realizado
    EscapeStrategy  EscStr;     // Estrategia de escape para el estado
};



void StateMachine();
void SetNextState(RSM_State s);
void SetNavCOMMSFlag(bool opt);

#endif /* ROBOTNAV_STATEMACHINE_H_ */
