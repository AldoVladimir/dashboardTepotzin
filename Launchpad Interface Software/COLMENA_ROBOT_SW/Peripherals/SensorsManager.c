#include <Peripherals/SensorsManager.h>

ADC_Handle   Isys, Imot, Ttop, Tbot;
uint16_t Isys_value, Imot_value, Ttop_value, Tbot_value;

void InitSensors(){

    /*
     * Esta función inicializa y abre los puertos de ADC
     * asignados a cada uno de los sensores del robot.
     */

    ADC_init();
    ADC_Params  ADC_params;

    ADC_Params_init(&ADC_params);
    Isys = ADC_open(ISYS_ADC_CH, &ADC_params);
    Imot = ADC_open(IMOT_ADC_CH, &ADC_params);
    Ttop = ADC_open(TTOP_ADC_CH, &ADC_params);
    Tbot = ADC_open(TBOT_ADC_CH, &ADC_params);

    AONBatMonEnable();

    initI2C(CC2650_I2C_SDA, CC2650_I2C_SCL);
    IMU_init(IMU_AG_ADDR(1), IMU_M_ADDR(1));
}

SensorRawMeas ReadSensor(SensorID s){

    /*
     * Esta función retorna la lectura del sensor seleccionado por el SensorID pasado como parámetro
     */

    SensorRawMeas measure_raw;
    //uint32_t measure_uV = 0;

    switch(s){
        case I_SYSTEM:
            ADC_convert(Isys, &Isys_value);
            measure_raw.ADCSensor_Meas = Isys_value;
            //measure_uV = ADC_convertRawToMicroVolts(Isys, Isys_value);
        break;

        case I_motor:
            ADC_convert(Imot, &Imot_value);
            measure_raw.ADCSensor_Meas = Imot_value;
            //measure_uV = ADC_convertRawToMicroVolts(Imot, Imot_value);
        break;

        case Tmp_top:
            ADC_convert(Ttop, &Ttop_value);
            measure_raw.ADCSensor_Meas = Ttop_value;
            //measure_uV = ADC_convertRawToMicroVolts(Ttop, Ttop_value);
        break;

        case Tmp_bot:
            ADC_convert(Tbot, &Tbot_value);
            measure_raw.ADCSensor_Meas = Tbot_value;
            //measure_uV = ADC_convertRawToMicroVolts(Tbot, Tbot_value);
        break;

        case IMU_ACCEL:
            IMU_readAccel();
            measure_raw.IMU_Meas_X = Imu.ax;
            measure_raw.IMU_Meas_Y = Imu.ay;
            measure_raw.IMU_Meas_Z = Imu.az;
        break;

        case IMU_MAG:
            IMU_readMag();
            measure_raw.IMU_Meas_X = Imu.ax;
            measure_raw.IMU_Meas_Y = Imu.ay;
            measure_raw.IMU_Meas_Z = Imu.az;;
        break;
    }

    return measure_raw;
}
