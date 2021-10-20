#include "Motors.h"


#if ROBOT_ID == 1                   // ROBOT 0
#define     PWM_LEFT    55
#define     PWM_RIGHT   45
#elif ROBOT_ID == 2                 // ROBOT 1
#define     PWM_LEFT    42
#define     PWM_RIGHT   39
#elif ROBOT_ID == 3                 // ROBOT 3
#define     PWM_LEFT    40
#define     PWM_RIGHT   50
#elif ROBOT_ID == 4                 // ROBOT 5
#define     PWM_LEFT    68
#define     PWM_RIGHT   68
#elif ROBOT_ID == 5
#define     PWM_LEFT    45
#define     PWM_RIGHT   50
#elif ROBOT_ID == 6
#define     PWM_LEFT    45
#define     PWM_RIGHT   50
#endif

bool COUNT_FLAG=false;
bool CONTROL = false;
uint16_t time_on = 0;
uint16_t time_step = 0;
uint8_t d_time=1;
uint8_t d_step=1;

void Init_Motors(){

    /*
     * Esta función configura los pines de los motores, así como los timers
     * que controlan la señal de PWM para cada uno de ellos.
     */

    setDigitalOutput(MOTORS_LEFT_DIR_GPIO);         // Configura pin dirección Motor izq
    setDigitalOutput(MOTORS_RIGHT_DIR_GPIO);        // Configura pin dirección Motor der
    setDigitalOutput(MOTORS_LEFT_PWM_GPIO);         // Configura pin PWM Motor izq
    setDigitalOutput(MOTORS_RIGHT_PWM_GPIO);        // Configura pin PWM Motor der

    // Inicializa todas las salidas de los motores a 0 inicialmente. Ambos motores quedan detenidos
    digitalWrite(MOTORS_LEFT_PWM_GPIO, 0);
    digitalWrite(MOTORS_RIGHT_PWM_GPIO,0);
    digitalWrite(MOTORS_LEFT_DIR_GPIO,0);
    digitalWrite(MOTORS_RIGHT_DIR_GPIO,0);

    IOCIOPortIdSet(MOTORS_LEFT_PWM_GPIO, IOC_PORT_MCU_PORT_EVENT5);
    IOCIOPortIdSet(MOTORS_RIGHT_PWM_GPIO, IOC_PORT_MCU_PORT_EVENT3);
    init16BitTimer_PWM(MOTOR_L_TIMER, MOTOR_L_SUBTIMER, 1, 100000);
    init16BitTimer_PWM(MOTOR_R_TIMER, MOTOR_R_SUBTIMER, 1, 100000);

    init16BitTimer(GPT3_BASE, TIMER_B, 250 , 1, ddata_timer_Interrupt);//Timer 4s para cambio de frecuencia
    TimerDisable(GPT3_BASE,TIMER_B);
}

void init16BitTimer_PWM(uint32_t baseTimer, uint32_t subTimer, uint32_t preescale, uint32_t freq){

    /*
     * Esta función inicializa el timer indicado por los parámetros @baseTimer y @subTimer
     * para su uso como generador de PWM para cada uno de los motores del robot.
     */

    uint32_t cuentaPWM;

    uint32_t numOfTimer = (baseTimer - 0x40010000)>>12;

    if (numOfTimer > 3 || (subTimer != TIMER_A && subTimer != TIMER_B)){
        return;
    }
    cuentaPWM = fabsf(dat/freq);

    if(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)!=PRCM_DOMAIN_POWER_ON){
       PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
       while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)!=PRCM_DOMAIN_POWER_ON);
    }

    PRCMPeripheralRunEnable(numOfTimer);
    PRCMPeripheralSleepEnable(numOfTimer);
    PRCMPeripheralDeepSleepEnable(numOfTimer);
    PRCMLoadSet();

    if(subTimer == TIMER_A){
        HWREG(baseTimer + GPT_O_CTL) &= ~(0x001);
        HWREG(baseTimer + GPT_O_CFG) = 0x04;
        HWREG(baseTimer + GPT_O_TAMR) = 0x20E;
        HWREG(baseTimer + GPT_O_TAILR) = (0x3E8);
        HWREG(baseTimer + GPT_O_TAMATCHR) = (0x384);
    }else{
        HWREG(baseTimer + GPT_O_CTL) &= ~(0x100);
        HWREG(baseTimer + GPT_O_CFG) = 0x04;
        HWREG(baseTimer + GPT_O_TBMR) =0x000A;
    }
    TimerLoadSet(baseTimer, subTimer, cuentaPWM);

    TimerMatchSet(baseTimer, subTimer, 0);
    TimerLevelControl(baseTimer, subTimer, true);

    if(subTimer == TIMER_A){
        HWREG(baseTimer + GPT_O_CTL) |= 0x001;
    }else{
        HWREG(baseTimer + GPT_O_CTL) |= 0x100;
    }
}

void motors_Control_SetMovement(uint8_t move, int8_t vL, int8_t vR){

    /*
     * Según el movimiento indicado en el parámetro @move, el robot realiza
     * un movimiento lineal, angular o se detiene. Los parámetros @vL y vR indican
     * si el movimiento se hace hacia adelante o en modo horario (si es positivo),
     * o hacia atrás o en modo antihorario (si es negativo).
     */

    bool direction;
    direction = vL >= 0 ? DIR_FORWARD : DIR_BACKWARD;
    vL = abs(vL) > 100 ? 100 : abs(vL);
    vR = abs(vR) > 100 ? 100 : abs(vR);

    switch(move){
        case CONTROL_MOTORS_MOVE_LINEAR:
            if(direction == DIR_FORWARD)
                motors_Control_Move_Forward();
            else
                motors_Control_Move_Backward();

            motors_Control_Controller(vL, vR);
            break;

        case CONTROL_MOTORS_MOVE_ANGULAR:
            if(direction == DIR_FORWARD)
                motors_Control_Move_Clockwise();
            else
                motors_Control_Move_CounterClockwise();

            motors_Control_Controller(vL, vR);
            break;

        case CONTROL_MOTORS_MOVE_STOP:
            motors_Control_Controller(0, 0);
            break;
    }
}

void motors_Control_Controller(uint8_t PWMcycle_L, uint8_t PWMcycle_R){

    /*
     * Establecen el ciclo de trabajo de cada uno de los
     * timers generadores de PWM para cada motor.
     */

    uint32_t mL = (uint32_t)(((float)(PWMcycle_L))*2.4);
    uint32_t mR = (uint32_t)(((float)(PWMcycle_R))*2.4);

    //TimerMatchSet(MOTOR_L_TIMER, MOTOR_L_SUBTIMER, (uint32_t)(((float)(PWMcycle_L))*2.4));
    TimerMatchSet(MOTOR_L_TIMER, MOTOR_L_SUBTIMER, mL);
    TimerEnable(MOTOR_L_TIMER, MOTOR_L_SUBTIMER);

    //TimerMatchSet(MOTOR_R_TIMER, MOTOR_R_SUBTIMER, (uint32_t)(((float)(PWMcycle_R))*2.4));
    TimerMatchSet(MOTOR_R_TIMER, MOTOR_R_SUBTIMER, mR);
    TimerEnable(MOTOR_R_TIMER, MOTOR_R_SUBTIMER);
}

void motors_Control_Move_Clockwise(){

    /*
     * Establece la dirección de los motores para moverse
     * en movimiento angular con sentido horario.
     */

    GPIO_writeDio(MOTORS_LEFT_DIR_GPIO, 1);
    GPIO_writeDio(MOTORS_RIGHT_DIR_GPIO, 0);
}

void motors_Control_Move_CounterClockwise(){

    /*
     * Establece la dirección de los motores para moverse
     * en movimiento angular con sentido antihorario.
     */

    GPIO_writeDio(MOTORS_LEFT_DIR_GPIO, 0);
    GPIO_writeDio(MOTORS_RIGHT_DIR_GPIO, 1);
}

void motors_Control_Move_Forward(){

    /*
     * Establece la dirección de los motores para moverse
     * en movimiento lineal hacia adelante.
     */

    GPIO_writeDio(MOTORS_LEFT_DIR_GPIO, 1);
    GPIO_writeDio(MOTORS_RIGHT_DIR_GPIO, 1);
}

void motors_Control_Move_Backward(){

    /*
     * Establece la dirección de los motores para moverse
     * en movimiento lineal hacia atrás.
     */

    GPIO_writeDio(MOTORS_LEFT_DIR_GPIO, 0);
    GPIO_writeDio(MOTORS_RIGHT_DIR_GPIO, 0);
}

void motors_Control_Move_Static(){

    /*
     * Detiene los motores y los timers generadores
     * de ciclo de trabajo por completo.
     */

   TimerMatchSet(GPT2_BASE, TIMER_B, 5);
   TimerMatchSet(GPT2_BASE, TIMER_A, 5);
   digitalWrite(MOTORS_LEFT_PWM_GPIO, 0);
   digitalWrite(MOTORS_RIGHT_PWM_GPIO,0);
   TimerDisable(MOTOR_R_TIMER, MOTOR_R_SUBTIMER);
   TimerDisable(MOTOR_L_TIMER, MOTOR_L_SUBTIMER);

}


void RotationStep(bool Dir){

    /*
     * Esta función hace que el robot rote un paso angular definido por sus
     * parámetros de rotación únicos. Rota en la dirección dada por @Dir
     */
    d_time=1;
    d_step=1;
    if(Dir == DIR_CLOCKWISE){
        COUNT_FLAG=true;
        motors_Control_SetMovement(CONTROL_MOTORS_MOVE_ANGULAR, PWM_LEFT, -PWM_RIGHT);
        TimerEnable(GPT3_BASE,TIMER_B);
    }else{
        if(Dir == DIR_COUNTERCLOCKWISE){
            COUNT_FLAG=true;
            motors_Control_SetMovement(CONTROL_MOTORS_MOVE_ANGULAR , -PWM_LEFT, PWM_RIGHT);
            TimerEnable(GPT3_BASE,TIMER_B);
        }
    }
}

void TranslationStep(bool Dir){

    /*
     * Esta función hace que el robot se traslade un paso lineal
     * en la dirección dada por el parámetro @Dir
     */
    d_time=1;
    d_step=1;
    if(Dir == DIR_FORWARD){
        COUNT_FLAG=true;
        motors_Control_SetMovement(CONTROL_MOTORS_MOVE_LINEAR, PWM_LEFT, PWM_RIGHT);
        TimerEnable(GPT3_BASE,TIMER_B);
    }else{
        if(Dir == DIR_BACKWARD){
            COUNT_FLAG=true;
            motors_Control_SetMovement(CONTROL_MOTORS_MOVE_LINEAR, -PWM_LEFT, -PWM_RIGHT);
            TimerEnable(GPT3_BASE,TIMER_B);
        }
    }
}

Movement_Status MoveRobotSteps(Movement_Type mov_type, int8_t Nsteps, bool Dir, struct NavState * NavS){

// ---------------------------- Short Movement. TODO: Remover esta sección de código ----------------------------

    uint8_t i = 0;

    for(i=0; i<Nsteps; i++){
        if(mov_type == ROTATION_MOVEMENT){
            RotationStep(Dir);
        }
        else{
            if(mov_type == TRANSLATION_MOVEMENT){
                TranslationStep(Dir);
            }
        }

        usleep(500000);
    }

    CoilRxMeasurement(NavS);                        // Verifica la presencia de otros robots con las espiras
//    if(NavS->ClusterChanged){                       // Si en la verificación se obtuvo que el cluster cambió
//        break;                                      // Se encontró un cluster y por tanto se deja de avanzar
//    }

    NavS->MovSt = SUCCESS_MOVE_DONE;
    return SUCCESS_NORMAL_SYS_CURRENT;


// --------------------------------------------------------------------------------------------------------------

    /*
     * Esta función hace que el robot se mueva una cantidad de pasos definida por @Nsteps
     * en la dirección @Dir (FORWARD/BACKWARD o CLOCKWISE/COUNTERCLOCKWISE).
     * A cada paso de movimiento realiza una medición de la corriente entregada
     * por los paneles solares por medio de la función CheckSystemCurrent(); además,
     * mide el status de las llantas por medio de la función CheckMotorsStatus()
     * y también verifica con la bobina Rx el tamaño de cluster en el que se encuentra.
     * En caso de que se presente algún error en la corriente o en el consumo de los motores,
     * se retorna un mensaje con el error correspondiende. De otra forma, se retorna @SUCCESS_NORMAL_SYS_CURRENT.
     *
     */

//    uint8_t i = 0;
//    Movement_Status MovSt_Motors, MovSt_Isys;
//
//    for(i=0; i<Nsteps; i++){
//
//        if(mov_type == ROTATION_MOVEMENT){              // Si el movimiento es rotacional
//            RotationStep(Dir);                          // Da un paso rotación
//        }
//        else{
//            if(mov_type == TRANSLATION_MOVEMENT){       // Si el movimiento es de translación
//                TranslationStep(Dir);                   // Da un paso de traslación
//            }
//        }
//
//        MovSt_Motors = CheckMotorsStatus();             // Status del movimiento de los motores
//        MovSt_Isys   = CheckSystemCurrent();            // Status de la Corriente entregada por los paneles
//
//        usleep(500000);
//
//        if(MovSt_Isys != SUCCESS_NORMAL_SYS_CURRENT){   // Si el status de la corriente no es el óptimo
//            NavS->MovSt = MovSt_Isys;                   // Retorna el error de corriente originado
//                                                        // El error de corriente de entrada tiene prioridad sobre
//                                                        // cualquier error de llantas.
//            return;
//        }
//
//        CoilRxMeasurement(NavS);                        // Verifica la presencia de otros robots con las espiras
//        if(NavS->ClusterChanged){                       // Si en la verificación se obtuvo que el cluster cambió
//            break;                                      // Se encontró un cluster y por tanto se deja de avanzar
//        }
//
//        if(MovSt_Motors != SUCCESS_MOVE_DONE){          // Si el status del movimiento no es el óptimo
//            NavS->MovSt = MovSt_Motors;                 // Retorna el status de los motores
//            return;
//        }
//    }
//
//    NavS->MovSt = SUCCESS_MOVE_DONE;                    // Si terminó de dar los pasos y no hubo ningún problema
                                                          // retorna un status de corriente normal.
}

Movement_Status CheckSystemCurrent(){

    /*
     * Esta función mide la corriente que entrega el panel solar.
     * Realiza una secuencia de NUM_MEDS_ISYS mediciones y después
     * obtiene la mediana de las mediciones. Compara dicha mediana
     * con el umbral ISYS_THLD, si es superior a este retorna el
     * estado @SUCCESS_NORMAL_SYS_CURRENT, de otra forma, retorna
     * el estado @ERROR_LOW_SYS_CURRENT.
     */

    uint8_t i;
    float ISys[NUM_MEDS_ISYS];

    for(i=0; i<NUM_MEDS_ISYS; i++){
        // Conversión: Isys = (RAW * 3.3V / 4096) * 2
        ISys[i] = ((float)ReadSensor(I_SYSTEM).ADCSensor_Meas)*0.001611328125;
    }

    if(GetMedian(ISys, NUM_MEDS_ISYS) > ISYS_THLD)      // Si la corriente entregada por el panel es mayor que la del umbral
        return SUCCESS_NORMAL_SYS_CURRENT;
    else
        return ERROR_LOW_SYS_CURRENT;

}

Movement_Status CheckMotorsStatus(){

    /*
     * Esta función mide la corriente consumida por los motores mientras están
     * funcionando y según la clasificación de consumo de sus motores identifica
     * si se encuentra en una situación de llantas patinando: ERROR_SKIDDING_TIRES;
     * llantas atoradas: ERROR_JAMMED_TIRES; o movimiento satisfactorio: SUCCESS_MOVE_DONE.
     */

    uint8_t i;
    float IMotors[NUM_MEDS_IMOTORS];
    float I_motors_median = 0;

    for(i=0; i<NUM_MEDS_IMOTORS; i++){
        // Conversión: IMotors = (RAW * 3.3V / 4096) * 2
        IMotors[i] = ((float)ReadSensor(I_motor).ADCSensor_Meas)*0.001611328125;
    }

    I_motors_median = GetMedian(IMotors, NUM_MEDS_IMOTORS);

    if(I_motors_median <= SKIDDING_TIRES_CURRENT_THLD){
        return ERROR_SKIDDING_TIRES;
    }
    else{
        if(I_motors_median >= JAMMED_TIRES_CURRENT_THLD){
            return ERROR_JAMMED_TIRES;
        }
        else{
            return SUCCESS_MOVE_DONE;
        }
    }

}

void ddata_timer_Interrupt(){
    TimerIntClear(GPT3_BASE,TIMER_TIMB_TIMEOUT);//timmer3_B
//
    if(COUNT_FLAG){//bandera para tiempo de duracion del paso
    time_on++;
    }
    if((time_on)==(d_time))//desire time
    {
        motors_Control_Controller(0, 0);
        COUNT_FLAG=false;
        TimerDisable(GPT3_BASE,TIMER_B);
        time_on=0;

        if((time_step)==(d_step-1)){//desire step
        time_step=0;//variable para numero de pasos
        }
        else{
            CONTROL=true;
            time_step++;//variable para numero de pasos
        }
    }
//    contadorM++;
}
