/*
 * NavDefs.h
 *
 *  Created on: 4 may. 2021
 *      Author: CASTI
 */

#ifndef ROBOTNAV_NAVDEFS_H_
#define ROBOTNAV_NAVDEFS_H_

#define TX  0
#define RX  1

#define OFF 0
#define ON  1

#define STATE_SUCCESS true
#define STATE_FAILED  false

#define pi  3.14159

// Parámetros de navegación que pueden ser modificados
typedef enum {
  MAX_SEARCH_STEPS,
  MAX_ADVANCE_STEP
} NavParam;


typedef struct {
    /*
     * Estructura de parámetros de estado. Preserva información importante
     * como retorno de cada estado. Esta es una estructura general y el uso
     * de sus parámetros son responsabilidad del programador.
     */
    bool StateResult;       // SUCCESS/FAILED
    uint8_t p1;
} StateParams;

// Variables globales de navegación





#endif /* ROBOTNAV_NAVDEFS_H_ */
