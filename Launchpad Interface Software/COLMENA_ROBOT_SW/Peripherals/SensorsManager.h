#ifndef PERIPHERALS_SENSORSMANAGER_H_
#define PERIPHERALS_SENSORSMANAGER_H_

#include <stdint.h>
#include <stdio.h>
#include "Adc.h"
#include "Gpio.h"
#include <ti/drivers/ADC.h>
#include "CC2640R2_LAUNCHXL.h"
#include <driverlib/aon_batmon.h>
#include "utils.h"
#include "LSM9DS1/LSM9DS1.h"
#include "I2c.h"

#define I_SYSTEM_pin    IOID_24         // Corriente que entrega el panel
#define I_motor_pin     IOID_27         // Corriente de ambos motores a la par
#define Tmp_top_pin     IOID_25         // Temperatura top
#define Tmp_bot_pin     IOID_26         // Temperatura bottom

#define CC2650_I2C_SDA  IOID_5
#define CC2650_I2C_SCL  IOID_4

#define ISYS_ADC_CH     CC2640R2_LAUNCHXL_ADC1
#define IMOT_ADC_CH     CC2640R2_LAUNCHXL_ADC4
#define TTOP_ADC_CH     CC2640R2_LAUNCHXL_ADC2
#define TBOT_ADC_CH     CC2640R2_LAUNCHXL_ADC3


typedef enum{
    I_SYSTEM,                   // Corriente de entrada de los paneles
    I_motor,                    // Corriente de los motores
    Tmp_top,                    // Temperatura en cara top
    Tmp_bot,                    // Temperatura en cara bot
    IMU_ACCEL,                  // Aceleración de IMU
    IMU_MAG                     // Magnetómetro de IMU
} SensorID;

typedef struct SensorRawMeas_S{
  int16_t ADCSensor_Meas;       // Espacio para medición cruda de cualquier sensor ADC
  int16_t IMU_Meas_X;           // Espacio para cualquier medición de IMU en eje X
  int16_t IMU_Meas_Y;           // Espacio para cualquier medición de IMU en eje Y
  int16_t IMU_Meas_Z;           // Espacio para cualquier medición de IMU en eje Z
} SensorRawMeas;


void InitSensors();
SensorRawMeas ReadSensor(SensorID s);

#endif /* PERIPHERALS_SENSORSMANAGER_H_ */
