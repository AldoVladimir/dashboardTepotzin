
#ifndef PERIPHERALS_MOTORS_H_
#define PERIPHERALS_MOTORS_H_

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

#include <driverlib/timer.h>
#include <driverlib/ioc.h>
#include <driverlib/interrupt.h>
#include "../Peripherals/Gpio.h"
#include "SensorsManager.h"
#include "RobotNav/StateMachine.h"
#include "CommonDeclarations.h"
#include <Robot.h>
#include "../Peripherals/Timer.h"

#define MOTORS_LEFT_DIR_GPIO        IOID_13
#define MOTORS_LEFT_PWM_GPIO        IOID_14
#define MOTORS_RIGHT_DIR_GPIO       IOID_19//PHASE
#define MOTORS_RIGHT_PWM_GPIO       IOID_18//ENABLE

#define MOTOR_L_TIMER               GPT2_BASE
#define MOTOR_L_SUBTIMER            TIMER_B
#define MOTOR_L_INTERRUPT           TIMER_TIMB_TIMEOUT
#define MOTOR_R_TIMER               GPT1_BASE
#define MOTOR_R_SUBTIMER            TIMER_B
#define MOTOR_R_INTERRUPT           TIMER_TIMB_TIMEOUT

#define dat                         24000000

#define CONTROL_MOTORS_MOVE_LINEAR   0
#define CONTROL_MOTORS_MOVE_ANGULAR  1
#define CONTROL_MOTORS_MOVE_STOP     2

#define DIR_FORWARD             true
#define DIR_BACKWARD            false
#define DIR_CLOCKWISE           true
#define DIR_COUNTERCLOCKWISE    false

#define ISYS_THLD           1
#define NUM_MEDS_ISYS       20
#define NUM_MEDS_IMOTORS    20

#define SKIDDING_TIRES_CURRENT_THLD 1
#define JAMMED_TIRES_CURRENT_THLD   1

struct NavState;

typedef enum {
  ROTATION_MOVEMENT,
  TRANSLATION_MOVEMENT
} Movement_Type;


void Init_Motors();
void init16BitTimer_PWM(uint32_t baseTimer, uint32_t subTimer, uint32_t preescale, uint32_t freq);
void motors_Control_SetMovement(uint8_t move, int8_t vL, int8_t vR);
void motors_Control_Controller(uint8_t PWMcycle_L, uint8_t PWMcycle_R);
void motors_Control_Move_Clockwise();
void motors_Control_Move_CounterClockwise();
void motors_Control_Move_Forward();
void motors_Control_Move_Backward();
void motors_Control_Move_Static();
void ddata_timer_Interrupt();

void RotationStep(bool Dir);
void TranslationStep(bool Dir);
Movement_Status MoveRobotSteps(Movement_Type mov_type, int8_t Nsteps, bool Dir, struct NavState * NavS);

Movement_Status CheckSystemCurrent();
Movement_Status CheckMotorsStatus();


#endif /* PERIPHERALS_MOTORS_H_ */

