
#ifndef PERIPHERALS_COIL_H_
#define PERIPHERALS_COIL_H_

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ti/drivers/ADCBuf.h>
#include "Board.h"
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/SPI.h>
#include <ti/devices/cc26x0r2/driverlib/ssi.h>
#include <Robot.h>
#include "ScienceMission.h"
#include "../RobotNav/StateMachine.h"
#include "fft.h"

//// #include <ti/sysbios/family/arm/m3/Hwi.h>
//#include <ti/drivers/Power.h>
//#include <ti/drivers/power/PowerCC26XX.h>
//#include <ti/devices/cc26x0r2/driverlib/vims.h>


#define pi 3.14159          // pi
//#define N 512               // Número de mediciones
#define Fs 52000            // Frecuencia de muestreo
#define SCI_N 123           // Número de mediciones de ciencia

#define ADD_SIGNAL      true
#define REMOVE_SIGNAL   false

#define MAX_DET_ITER        8           // Cada iteración de detección está compuesta por MAX_POLLS_CALIB sondeos y fft's
// #define MAX_POLLS           1           // Cada sondeo consiste de leer N muestras del ADC

#define CLUST_DESTRUCTION   13

struct NavState;

typedef enum{
  none,
  f1,
  f2,
  f3,
  f4,
  f5,
  f6,
  f7,
  f8,
  f9,
  f10,
  f11,
  f12,
  f13
} CoilFreq;

typedef enum{
    Sci_f_200kHz    = 200000,
    Sci_f_100kHz    = 100000,
    Sci_f_10kHz     = 10000,
    Sci_f_1kHz      = 1000,
    Sci_f_100Hz     = 100,
    Sci_f_10Hz      = 10,
    Sci_f_1Hz       = 1
} Sci_Fs;

typedef struct DigOscilator{        // Estructura que permite manejar las características importantes de los osciladores digitales
    int8_t active;                     // Indica si la señal está activa o no
    int8_t a1;                         // Coeficiente a1 del oscilador
    int8_t yn_1;                       // Dato y(n-1) necesario para calular y(n)
    int8_t yn_2;                       // Dato y(n-2) necesario para calular y(n)
} DigOscilator;



// ------------------------------------------ Coil RX ------------------------------------------
void Init_CoilRx();
void ConvertToVolts(void* sampleBuffer, float * target_vect);
void ResetCoilMeasurements();
void CalcMeanPSD(uint8_t polls);
void CoilRxMeasurement(struct NavState *NavS);
int MedianCompare(const void *_a, const void *_b);
float GetMedian(float* V, int size);
void CheckBandsDetected();
uint8_t GetClusterSize();
uint16_t * CoilRxMeasurement_ScienceMission(Sci_Fs fs);
void CoilRx_SamplesReady_SciMission(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel);

// ------------------------------------------ Coil TX ------------------------------------------

void Init_CoilTx();
void InitSignals();
void Configure_CoilTx_SPI(bool opt);
void ChangeSignal_timer1ACallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
void SetValueDAC_timer3ACallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
void EnableCoilTxSignal(bool mode);
void UpdateCoilTxSignal();
void UpdateCoilTxFreqList(bool mode, CoilFreq fs);

extern uint16_t genNewSignalValue_Sine();



//static void SimplePeripheral_disableCache();
//static void SimplePeripheral_enableCache();


#endif /* PERIPHERALS_COIL_H_ */
