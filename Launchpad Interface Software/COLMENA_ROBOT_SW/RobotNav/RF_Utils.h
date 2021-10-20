/*
 * RadioMeasurements.h
 *
 *  Created on: 4 may. 2021
 *      Author: CASTI
 */

#ifndef ROBOTNAV_RF_NAVIGATION_H_
#define ROBOTNAV_RF_NAVIGATION_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "NavDefs.h"
#include "math.h"
#include "Peripherals/Radio.h"
#include "Peripherals/utils.h"

#define NumCh                   5       // N�mero de canales activos para buscar
#define NUM_MEDS                10      // N�mero de mediciones a realizar
#define RF_THLD_dB              -90     // Umbral de RSSI para considerar que es una se�al
#define NUM_ST_VERIF_MEAS       20      // N�mero de mediciones de verificaci�n de b�squeda est�tica
#define PCT_LVL_FOR_SUCCESS     0.6     // Porcentaje de mediciones de verificaci�n exitosas necesarias para declarar un canal como detectado

enum Ch_type;

void ResetRFMeasurements();
float MeasureRSSI(enum Ch_type ch);

void DoRFMeasureInAngle(float deg);
bool EvaluateMeasurements();
void GetDirVector(float* dv);
bool IsOtherRobotActivated();

void DoRFMeasureStatic();
uint8_t EvaluateStaticMeasurements();

void SetToneInChannel(enum Ch_type ch);
enum Ch_type CS_to_CHtype(uint8_t cs);


#endif /* ROBOTNAV_RF_NAVIGATION_H_ */
